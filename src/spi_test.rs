#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_async::spi::{Operation, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;

use esp_backtrace as _;
use esp_println as _;

use esp_hal::{
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Io, Level, NoPin, Output},
    prelude::*,
    spi::{master::Spi, SpiMode},
    timer::{timg::TimerGroup, AnyTimer, OneShotTimer},
};

use static_cell::StaticCell;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        STATIC_CELL.uninit().write(($val))
    }};
}

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    defmt::debug!("Init!");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();
    let timers = [OneShotTimer::new(timer0)];
    let timers = mk_static!([OneShotTimer<AnyTimer>; 1], timers);

    esp_hal_embassy::init(timers);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Init SPI and LoRa
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.spi2channel;

    let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(1024);
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    /*
    let sclk = io.pins.gpio5;
    let miso = io.pins.gpio19;
    let mosi = io.pins.gpio27;
    */
    let sclk = io.pins.gpio5;
    let miso = io.pins.gpio12;
    let mosi = io.pins.gpio13;
    let cs = Output::new(io.pins.gpio18, Level::Low);

    let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0)
        // use NoPin for CS as we'll going to be using the SpiDevice trait
        // via ExclusiveSpiDevice as we don't (yet) want to pull in embassy-sync
        .with_pins(sclk, mosi, miso, NoPin)
        .with_dma(dma_channel.configure_for_async(false, DmaPriority::Priority0))
        .with_buffers(dma_rx_buf, dma_tx_buf);

    let mut spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();

    let write = [0xde, 0xad, 0xbe, 0xef];
    let mut read: [u8; 4] = [0x00u8; 4];

    defmt::info!("SPI 1...");
    // TODO: defmt::unwrap!(...)
    let _ = spi_dev.transfer(&mut read[..], &write[..]).await;

    defmt::info!("...done");
    assert_eq!(write, read);

    let write_buf = &[1];
    //let mut payload = &[0xd1, 0xd2, 0xd3, 0xd4];
    // Works with 1..4 bytes
    // Hangs when payload is > 4 bytes??
    let payload = &[5, 4, 3, 2, 1];

    defmt::info!("SPI 2 (write buffer)... ");
    let mut ops = [Operation::Write(write_buf), Operation::Write(payload)];
    let _ = spi_dev.transaction(&mut ops).await;
    defmt::info!("...done");

    defmt::info!("SPI 3 (read previous buffer)...");
    let mut read: [u8; 6] = [0x00u8; 6];
    let mut ops = [Operation::Read(&mut read)];
    let _ = spi_dev.transaction(&mut ops).await;
    defmt::info!("... result: {:?}", read);
    // TODO: Assert that write_buf + payload == read ?

    loop {
        defmt::info!("MAIN LOOP!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}
