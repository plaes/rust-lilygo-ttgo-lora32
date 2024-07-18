#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use esp_backtrace as _;
use esp_println as _;

use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_descriptors,
    gpio::{GpioPin, Input, Io, Level, Output, Pull, NO_PIN},
    i2c::I2C,
    peripherals::{Peripherals, I2C0},
    prelude::*,
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
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

const MAX_TX_POWER: i32 = 14;
const LORA_FREQUENCY_IN_HZ: u32 = 868_100_000;

type OledIface = I2CInterface<I2C<'static, I2C0, esp_hal::Async>>;

#[main]
async fn main(spawner: Spawner) {
    defmt::debug!("Init!");

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
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
    let oled_reset = Output::new(io.pins.gpio16, Level::Low);
    spawner.spawn(oled_task(iface, oled_reset)).ok();

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

    let spi = esp_hal::FlashSafeDma::<_, 256>::new(spi);

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
    let mut lora = {
        match LoRa::new(Sx127x::new(spi_dev, iv, config), true, Delay).await {
            Ok(r) => r,
            Err(r) => panic!("Fail: {:?}", r),
        }
    };

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

    defmt::info!("Sleeping!");
    match lora.sleep(false).await {
        Ok(()) => defmt::info!("Sleep successful"),
        Err(err) => defmt::info!("Sleep unsuccessful = {}", err),
    }

    let send = [1, 2, 3, 4];

    loop {
        defmt::info!("MAIN LOOP!");
        Timer::after(Duration::from_millis(5_000)).await;

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
    }
}

#[embassy_executor::task]
async fn oled_task(
    iface: OledIface,
    // TODO: Figure out how to pass generic "anypin"
    mut reset: Output<'static, GpioPin<16>>,
) {
    defmt::debug!("DISPLAY INIT!");

    let mut display = Ssd1306::new(iface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

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

        Text::with_alignment(
            "Demo!",
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
