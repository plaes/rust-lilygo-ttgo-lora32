#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use esp_backtrace as _;
use esp_println as _;

use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority, Spi2DmaChannel},
    dma_descriptors,
    gpio::{any_pin::AnyPin, GpioPin, Input, Io, Level, Output, Pull, NO_PIN},
    i2c::I2C,
    peripherals::{Peripherals, I2C0},
    prelude::*,
    spi::{
        master::{dma::SpiDma, prelude::*, Spi},
        FullDuplexMode, SpiMode,
    },
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
    Async, FlashSafeDma,
};

use embedded_graphics::{
    mono_font::{
        ascii::{FONT_6X10, FONT_9X18_BOLD},
        MonoTextStyleBuilder,
    },
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::sx127x::{self, Sx1276, Sx127x};
use lora_phy::LoRa;

use lora_phy::mod_params::{Bandwidth, CodingRate, SpreadingFactor};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.uninit().write(($val))
    }};
}

const LORA_FREQUENCY_IN_HZ: u32 = 868_100_000;
const DMA_BUF: usize = 256;

type OledIface = I2CInterface<I2C<'static, I2C0, Async>>;

type SafeSpiDma = FlashSafeDma<
    SpiDma<'static, esp_hal::peripherals::SPI2, Spi2DmaChannel, FullDuplexMode, Async>,
    DMA_BUF,
>;

// TODO: Figure out AnyPin
type SxIfaceVariant =
    GenericSx127xInterfaceVariant<Output<'static, GpioPin<22>>, Input<'static, GpioPin<26>>>;

#[main]
async fn main(spawner: Spawner) {
    defmt::debug!("Init!");

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer0 = OneShotTimer::new(timg0.timer0.into());
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], [timer0]);

    defmt::debug!("Init clocks!");

    esp_hal_embassy::init(&clocks, timers);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c0 = I2C::new_async(
        peripherals.I2C0,
        io.pins.gpio4,
        io.pins.gpio15,
        100.kHz(),
        &clocks,
    );

    defmt::debug!("Init i2c!");
    let iface = I2CDisplayInterface::new(i2c0);
    let oled_reset = io.pins.gpio16;
    spawner
        .spawn(oled_task(iface, AnyPin::new(oled_reset)))
        .ok();

    let prg_button = io.pins.gpio0;
    spawner.spawn(button_detect(AnyPin::new(prg_button))).ok();

    let blue_led = io.pins.gpio2;
    spawner.spawn(led_blinker(AnyPin::new(blue_led))).ok();

    // Init SPI and LoRa
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.spi2channel;

    let (tx_descriptors, rx_descriptors) = dma_descriptors!(1024);

    let sclk = io.pins.gpio5;
    let miso = io.pins.gpio19;
    let mosi = io.pins.gpio27;
    let cs = Output::new(io.pins.gpio18, Level::Low);

    let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
        // use NO_PIN for CS as we'll going to be using the SpiDevice trait
        // via ExclusiveSpiDevice as we don't (yet) want to pull in embassy-sync
        .with_pins(Some(sclk), Some(mosi), Some(miso), NO_PIN)
        .with_dma(
            dma_channel.configure_for_async(false, DmaPriority::Priority0),
            tx_descriptors,
            rx_descriptors,
        );

    let spi: SafeSpiDma = FlashSafeDma::new(spi);

    let spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();

    let lora_reset = Output::new(io.pins.gpio22, Level::Low);
    let lora_dio0 = Input::new(io.pins.gpio26, Pull::None);
    let iv = GenericSx127xInterfaceVariant::new(lora_reset, lora_dio0, None, None).unwrap();

    let config = sx127x::Config {
        chip: Sx1276,
        tcxo_used: false,
        rx_boost: false,
        tx_boost: false,
    };

    defmt::info!("Initializing LoRa");
    let lora = {
        match LoRa::new(Sx127x::new(spi_dev, iv, config), true, Delay).await {
            Ok(r) => r,
            Err(r) => panic!("Fail: {:?}", r),
        }
    };

    spawner.spawn(lora_handler(lora)).ok();

    loop {
        defmt::info!("MAIN LOOP!");
        Timer::after(Duration::from_millis(5_000)).await;
    }
}

#[embassy_executor::task]
async fn led_blinker(pin: AnyPin<'static>) {
    let mut led = Output::new(pin, Level::High);

    loop {
        Timer::after(Duration::from_millis(500)).await;
        led.toggle();
    }
}

#[embassy_executor::task]
async fn button_detect(pin: AnyPin<'static>) {
    let mut button = Input::new(pin, Pull::Down);

    loop {
        button.wait_for_any_edge().await;
        if button.is_high() {
            defmt::info!("Button pressed!");
        } else {
            defmt::info!("Button released!");
        }
    }
}

#[embassy_executor::task]
async fn oled_task(iface: OledIface, reset: AnyPin<'static>) {
    defmt::debug!("DISPLAY INIT!");

    let mut display = Ssd1306::new(iface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    let mut reset = Output::new(reset, Level::Low);

    // For some reason, this board really needs reset..
    display.reset(&mut reset, &mut Delay).unwrap();

    display.init().unwrap();

    // Specify different text styles
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let text_style_big = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .build();

    loop {
        defmt::info!("OLED LOOP!");

        Text::with_alignment(
            "esp-hal",
            display.bounding_box().center() + Point::new(0, 0),
            text_style_big,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_alignment(
            "Chip: ESP32",
            display.bounding_box().center() + Point::new(0, 14),
            text_style,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        display.flush().unwrap();
        display.clear(BinaryColor::Off).unwrap();

        Timer::after(Duration::from_millis(3_000)).await;

        let _text = "Transmit!";

        #[cfg(feature="receiver")]
        let _text = "Receive!";

        let text = _text;

        Text::with_alignment(
            text,
            display.bounding_box().center(),
            text_style_big,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        display.flush().unwrap();
        display.clear(BinaryColor::Off).unwrap();

        Timer::after(Duration::from_millis(3_000)).await;
    }
}

#[cfg(feature = "receiver")]
#[embassy_executor::task]
async fn lora_handler(
    mut lora: LoRa<
        Sx127x<
            ExclusiveDevice<SafeSpiDma, Output<'static, GpioPin<18>>, embassy_time::Delay>,
            SxIfaceVariant,
            Sx1276,
        >,
        Delay,
    >,
) {
    const RX_BUF_LEN: u8 = 255;

    defmt::info!("Initializing modulation parameters");

    let mdltn_params = {
        match lora.create_modulation_params(
            SpreadingFactor::_10,
            Bandwidth::_250KHz,
            CodingRate::_4_8,
            LORA_FREQUENCY_IN_HZ,
        ) {
            Ok(mp) => mp,
            Err(err) => {
                defmt::info!("Radio error = {}", err);
                return;
            }
        }
    };

    let rx_pkt_params = {
        match lora.create_rx_packet_params(4, false, RX_BUF_LEN, true, false, &mdltn_params) {
            Ok(pp) => pp,
            Err(err) => {
                defmt::info!("Radio error: {}", err);
                return;
            }
        }
    };

    defmt::info!("Sleeping!");
    match lora.sleep(false).await {
        Ok(()) => defmt::info!("Sleep successful"),
        Err(err) => defmt::info!("Sleep unsuccessful = {}", err),
    }

    match lora
        .prepare_for_rx(lora_phy::RxMode::Continuous, &mdltn_params, &rx_pkt_params)
        .await
    {
        Ok(()) => {}
        Err(err) => {
            defmt::info!("Radio error = {}", err);
            return;
        }
    };

    loop {
        defmt::info!("RX LOOP!");

        let mut receiving_buffer = [0u8; RX_BUF_LEN as usize];
        match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
            Ok((len, status)) => {
                defmt::info!(
                    "rx successful (got {} bytes): {:?}",
                    len,
                    receiving_buffer[0..len as usize]
                );
                defmt::info!("RSSI: {}, SNR: {}", status.rssi, status.snr);
            }
            Err(err) => defmt::info!("rx unsuccessful = {}", err),
        }
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[cfg(not(feature = "receiver"))]
#[embassy_executor::task]
async fn lora_handler(
    mut lora: LoRa<
        Sx127x<
            ExclusiveDevice<SafeSpiDma, Output<'static, GpioPin<18>>, embassy_time::Delay>,
            SxIfaceVariant,
            Sx1276,
        >,
        Delay,
    >,
) {
    const MAX_TX_POWER: i32 = 14;

    let mdltn_params = {
        match lora.create_modulation_params(
            SpreadingFactor::_10,
            Bandwidth::_250KHz,
            CodingRate::_4_8,
            LORA_FREQUENCY_IN_HZ,
        ) {
            Ok(mp) => mp,
            Err(err) => {
                defmt::info!("Radio error = {}", err);
                return;
            }
        }
    };

    defmt::info!("Initializing packet parameters");
    let mut tx_pkt_params = {
        match lora.create_tx_packet_params(4, false, true, false, &mdltn_params) {
            Ok(pp) => pp,
            Err(err) => {
                defmt::info!("Radio error = {}", err);
                return;
            }
        }
    };
    loop {
        // NB! Seems like transfers of 3, 5, 6 and 8 bytes fail
        // https://github.com/esp-rs/esp-hal/issues/1798
        let send = [4, 3, 2, 1];

        defmt::info!("Preparing radio!");
        match lora
            .prepare_for_tx(&mdltn_params, &mut tx_pkt_params, MAX_TX_POWER, &send)
            .await
        {
            Ok(()) => {
                defmt::info!("Radio prepared!");
            }
            Err(err) => {
                defmt::info!("Radio error = {}", err);
                return;
            }
        };

        match lora.tx().await {
            Ok(()) => {
                defmt::info!("TX DONE");
            }
            Err(err) => {
                defmt::info!("Radio error = {}", err);
                return;
            }
        };
        Timer::after(Duration::from_millis(5_000)).await;
    }
}
