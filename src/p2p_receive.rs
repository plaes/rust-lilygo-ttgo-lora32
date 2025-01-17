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
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Input, Level, Output, Pin, Pull},
    i2c::master::I2c,
    prelude::*,
    spi::{
        master::{Spi, SpiDmaBus},
        SpiMode,
    },
    timer::{timg::TimerGroup, AnyTimer, OneShotTimer},
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

use hex_display::HexDisplayExt;

const RX_BUF_LEN: usize = 255;

struct Message {
    len: usize,
    buf: [u8; RX_BUF_LEN],
    rssi: i16,
    snr: i16,
}

static CHANNEL: StaticCell<Channel<NoopRawMutex, Message, 1>> = StaticCell::new();

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        STATIC_CELL.uninit().write(($val))
    }};
}

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

const LORA_BW: LoRaBw = LoRaBw(Bandwidth::_250KHz);
const LORA_SF: LoRaSf = LoRaSf(SpreadingFactor::_10);
const LORA_CR: LoRaCr = LoRaCr(CodingRate::_4_8);
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
    let timer0: AnyTimer = timg0.timer0.into();
    let timers = [OneShotTimer::new(timer0)];
    let timers = mk_static!([OneShotTimer<AnyTimer>; 1], timers);

    esp_hal_embassy::init(timers);

    let channel = CHANNEL.init(Channel::new());

    let i2c0 = I2c::new(peripherals.I2C0, {
        let mut cfg = esp_hal::i2c::master::Config::default();
        cfg.frequency = 100.kHz();
        cfg
    })
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
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.spi2channel;

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUF_SIZE);
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    let sclk = peripherals.GPIO5;
    let miso = peripherals.GPIO19;
    let mosi = peripherals.GPIO27;
    let cs = Output::new(peripherals.GPIO18, Level::Low);

    let spi = Spi::new_with_config(peripherals.SPI2, {
        let mut cfg = esp_hal::spi::master::Config::default();
        cfg.frequency = 100.kHz();
        cfg.mode = SpiMode::Mode0;
        cfg
    })
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_miso(miso)
    .with_dma(dma_channel.configure(false, DmaPriority::Priority0))
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
            Err(r) => {
                defmt::info!("Failed to init sx127x chip!");
                panic!("Fail: {:?}", r);
            }
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
        "LoRa P2P RX!",
        display.bounding_box().center() + Point::new(0, 14),
        text_style,
        Alignment::Center,
    )
    .draw(&mut display)
    .unwrap();

    display.flush().unwrap();

    Timer::after(Duration::from_millis(1_000)).await;

    loop {
        use no_std_strings::{str256, str64, str_format};

        let data = channel.receive().await;

        display.clear(BinaryColor::Off).unwrap();

        Text::new(
            &str_format!(
                str64,
                "RX Freq: {}MHz\nBW{}, CR{}, SF{}",
                LORA_FREQUENCY_IN_HZ as f64 / 1_000_000.0,
                LORA_BW,
                LORA_CR,
                LORA_SF,
            ),
            Point::new(0, 8),
            text_style,
        )
        .draw(&mut display)
        .unwrap();

        Text::new(
            &str_format!(str64, "RSSI/SNR: {}/{}, data:", data.rssi, data.snr),
            Point::new(0, 28),
            text_style,
        )
        .draw(&mut display)
        .unwrap();

        // Format packet into chunks (we can only fit 12 bytes on a row)
        let mut row = 0;
        const ROW_LEN: usize = 12;
        for chunk in data.buf.chunks(ROW_LEN) {
            if row > 3 {
                // TODO: Num bytes rx? Data overflow notification?
                break;
            }

            // Make sure we only take as much as packet contains...
            if ((1 + row) * ROW_LEN) > data.len {
                Text::new(
                    &str_format!(str256, "{}", &data.buf[(row * ROW_LEN)..data.len].hex()),
                    Point::new(2, (36 + (row * 9)).try_into().unwrap()),
                    text_style,
                )
                .draw(&mut display)
                .unwrap();
                break;
            } else {
                Text::new(
                    &str_format!(str256, "{}", chunk.hex()),
                    Point::new(2, (36 + (row * 9)).try_into().unwrap()),
                    text_style,
                )
                .draw(&mut display)
                .unwrap();
            }

            row += 1;
        }

        display.flush().unwrap();
    }
}

#[embassy_executor::task]
async fn lora_handler(
    mut lora: LoRa<Sx127x<LoraSpiDev, SxIfaceVariant, Sx1276>, Delay>,
    channel: Sender<'static, NoopRawMutex, Message, 1>,
) {
    defmt::info!("Initializing modulation parameters");

    let mdltn_params = {
        match lora.create_modulation_params(LORA_SF.0, LORA_BW.0, LORA_CR.0, LORA_FREQUENCY_IN_HZ) {
            Ok(mp) => mp,
            Err(err) => {
                defmt::info!("Radio error = {}", err);
                return;
            }
        }
    };

    let rx_pkt_params = {
        match lora.create_rx_packet_params(4, false, RX_BUF_LEN as u8, true, false, &mdltn_params) {
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

        let mut rx_buf = [0xa; RX_BUF_LEN];
        match lora.rx(&rx_pkt_params, &mut rx_buf).await {
            Ok((len, status)) => {
                defmt::info!("{:?}", rx_buf);
                let mut buf = [0xa; RX_BUF_LEN];
                buf.copy_from_slice(&rx_buf);
                /*
                let mut buf = [
                    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6,
                    7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                ];
                */

                defmt::info!(
                    "rx successful (got {} bytes): {:?}",
                    len,
                    rx_buf[0..len as usize]
                );
                defmt::info!("RSSI: {}, SNR: {}", status.rssi, status.snr);

                let msg = Message {
                    // len: RX_BUF_LEN as usize,
                    len: len.into(),
                    buf,
                    rssi: status.rssi,
                    snr: status.snr,
                };

                channel.send(msg).await;
            }
            Err(err) => defmt::info!("rx unsuccessful = {}", err),
        }
        Timer::after(Duration::from_millis(1_000)).await;
    }
}
