#![no_std]
#![no_main]

use core::fmt::{Display, Formatter};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use esp_backtrace as _;
use esp_println as _;

use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Input, Level, Output, Pin, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    spi::{
        master::{Config as SpiConfig, Spi, SpiDmaBus},
        Mode,
    },
    time::RateExtU32,
    timer::timg::TimerGroup,
    Async,
};

use embedded_graphics::{
    mono_font::{
        ascii::{FONT_5X8, FONT_9X18_BOLD},
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

use static_cell::StaticCell;

use no_std_strings::{str64, str_format};

struct Message(u8);

static CHANNEL: StaticCell<Channel<NoopRawMutex, Message, 1>> = StaticCell::new();

struct LoRaBw(Bandwidth);
impl Display for LoRaBw {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        let m = match self.0 {
            Bandwidth::_7KHz => 7,
            Bandwidth::_10KHz => 10,
            Bandwidth::_15KHz => 15,
            Bandwidth::_20KHz => 20,
            Bandwidth::_31KHz => 31,
            Bandwidth::_41KHz => 41,
            Bandwidth::_62KHz => 62,
            Bandwidth::_125KHz => 125,
            Bandwidth::_250KHz => 250,
            Bandwidth::_500KHz => 500,
        };
        write!(f, "{}KHz", m)
    }
}

struct LoRaCr(CodingRate);
impl Display for LoRaCr {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        let m = match self.0 {
            CodingRate::_4_5 => "4/5",
            CodingRate::_4_6 => "4/6",
            CodingRate::_4_7 => "4/7",
            CodingRate::_4_8 => "4/8",
        };
        write!(f, "{}", m)
    }
}

struct LoRaSf(SpreadingFactor);
impl Display for LoRaSf {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.0.factor())
    }
}

const LORA_BW: LoRaBw = LoRaBw(Bandwidth::_125KHz);
const LORA_SF: LoRaSf = LoRaSf(SpreadingFactor::_12);
const LORA_CR: LoRaCr = LoRaCr(CodingRate::_4_5);
const LORA_FREQUENCY_IN_HZ: u32 = 868_100_000;
const DMA_BUF_SIZE: usize = 1024;

type OledIface = I2CInterface<I2c<'static, Async>>;

type SxIfaceVariant = GenericSx127xInterfaceVariant<Output<'static>, Input<'static>>;
type SpiBus = SpiDmaBus<'static, Async>;
type LoraSpiDev = ExclusiveDevice<SpiBus, Output<'static>, Delay>;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    defmt::debug!("Init!");

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    esp_hal_embassy::init(timg0.timer0);

    let channel = CHANNEL.init(Channel::new());

    let i2c0 = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(peripherals.GPIO4)
        .with_scl(peripherals.GPIO15)
        .into_async();

    defmt::debug!("Init i2c!");
    let iface = I2CDisplayInterface::new(i2c0);
    let oled_reset = peripherals.GPIO16;
    spawner
        .spawn(oled_task(iface, oled_reset.degrade(), channel.receiver()))
        .ok();

    let prg_button = peripherals.GPIO0.degrade();
    spawner.spawn(button_detect(prg_button)).ok();

    let blue_led = peripherals.GPIO2.degrade();
    spawner.spawn(led_blinker(blue_led)).ok();

    // Init SPI and LoRa
    let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUF_SIZE);
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    let sclk = peripherals.GPIO5;
    let miso = peripherals.GPIO19;
    let mosi = peripherals.GPIO27;
    let cs = Output::new(peripherals.GPIO18, Level::Low);

    let spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(100.kHz())
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_miso(miso)
    .with_dma(peripherals.DMA_SPI2)
    .with_buffers(dma_rx_buf, dma_tx_buf)
    .into_async();

    let spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();

    let lora_reset = Output::new(peripherals.GPIO22, Level::Low);
    let lora_dio0 = Input::new(peripherals.GPIO26, Pull::None);
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

    spawner.spawn(lora_handler(lora, channel.sender())).ok();

    loop {
        defmt::info!("MAIN LOOP!");
        Timer::after(Duration::from_millis(5_000)).await;
    }
}

#[embassy_executor::task]
async fn led_blinker(pin: AnyPin) {
    let mut led = Output::new(pin, Level::High);

    loop {
        Timer::after(Duration::from_millis(500)).await;
        led.toggle();
    }
}

#[embassy_executor::task]
async fn button_detect(pin: AnyPin) {
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
async fn oled_task(
    iface: OledIface,
    reset: AnyPin,
    channel: Receiver<'static, NoopRawMutex, Message, 1>,
) {
    defmt::debug!("DISPLAY INIT!");

    let mut display = Ssd1306::new(iface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    let mut reset = Output::new(reset, Level::Low);

    // For some reason, this board really needs reset..
    display.reset(&mut reset, &mut Delay).unwrap();

    display.init().unwrap();

    loop {
        // Specify different text styles
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_5X8)
            .text_color(BinaryColor::On)
            .build();
        let text_style_big = MonoTextStyleBuilder::new()
            .font(&FONT_9X18_BOLD)
            .text_color(BinaryColor::On)
            .build();

        // Show banner
        Text::with_alignment(
            "esp-hal",
            display.bounding_box().center() + Point::new(0, -14),
            text_style_big,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_alignment(
            "Chip: ESP32 + SX1276",
            display.bounding_box().center() + Point::new(0, 0),
            text_style,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_alignment(
            "LoRa P2P TX!",
            display.bounding_box().center() + Point::new(0, 14),
            text_style,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();
        display.flush().unwrap();

        let data = channel.receive().await;

        Text::with_alignment(
            &str_format!(str64, "Sent packet: {}!", data.0),
            display.bounding_box().center() + Point::new(0, 28),
            text_style,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();
        display.flush().unwrap();

        Timer::after(Duration::from_millis(3_000)).await;
        display.clear(BinaryColor::Off).unwrap();
    }
}

#[embassy_executor::task]
async fn lora_handler(
    mut lora: LoRa<Sx127x<LoraSpiDev, SxIfaceVariant, Sx1276>, Delay>,
    channel: Sender<'static, NoopRawMutex, Message, 1>,
) {
    const MAX_TX_POWER: i32 = 14;

    let mdltn_params = {
        match lora.create_modulation_params(LORA_SF.0, LORA_BW.0, LORA_CR.0, LORA_FREQUENCY_IN_HZ) {
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
    let mut packet: u8 = 1;
    loop {
        // NB! Seems like transfers of 3, 5, 6 and 8 bytes fail
        // https://github.com/esp-rs/esp-hal/issues/1798
        let send = [packet, 0xaa, 0x3a, 0xa3, packet];

        (packet, _) = packet.overflowing_add(1);

        defmt::info!("Preparing radio for packet: {}", packet);
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
                channel.send(Message(packet)).await;
            }
            Err(err) => {
                defmt::info!("Radio error = {}", err);
                return;
            }
        }

        Timer::after(Duration::from_millis(5_000)).await;
    }
}
