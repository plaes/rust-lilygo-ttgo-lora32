#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_async::spi::{Operation, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;

use esp_backtrace as _;
use esp_println as _;

use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority, Spi2DmaChannel},
    dma_descriptors,
    gpio::{Io, Level, Output, NO_PIN},
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{dma::SpiDma, prelude::*, Spi},
        FullDuplexMode, SpiMode,
    },
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
    Async, FlashSafeDma,
};

use static_cell::StaticCell;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        STATIC_CELL.uninit().write(($val))
    }};
}

const DMA_BUF: usize = 256;

type SafeSpiDma = FlashSafeDma<
    SpiDma<'static, esp_hal::peripherals::SPI2, Spi2DmaChannel, FullDuplexMode, Async>,
    DMA_BUF,
>;

#[main]
async fn main(_spawner: Spawner) {
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

    // Init SPI and LoRa
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.spi2channel;

    let (tx_descriptors, rx_descriptors) = dma_descriptors!(1024);

    /*
    let sclk = io.pins.gpio5;
    let miso = io.pins.gpio19;
    let mosi = io.pins.gpio27;
    */
    let sclk = io.pins.gpio5;
    let miso = io.pins.gpio12;
    let mosi = io.pins.gpio13;
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

    let mut spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();

    let write = [0xde, 0xad, 0xbe, 0xef];
    let mut read: [u8; 4] = [0x00u8; 4];

    defmt::info!("TX...");
    // TODO: defmt::unwrap!(...)
    let _ = spi_dev.transfer(&mut read[..], &write[..]).await;

    defmt::info!("...done");
    assert_eq!(write, read);

    let write_buf = &[0xaa];
    //let mut payload = &[0xd1, 0xd2, 0xd3, 0xd4];
    // Works with 1..4 bytes
    // Hangs when payload is > 4 bytes??
    let payload = &[0xd1, 0xd2, 0xd3, 0xd5, 0xd5];

    defmt::info!("TX... ");
    let mut ops = [Operation::Write(write_buf), Operation::Write(payload)];
    let _ = spi_dev.transaction(&mut ops).await;
    defmt::info!("...done");

    loop {
        defmt::info!("MAIN LOOP!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}
