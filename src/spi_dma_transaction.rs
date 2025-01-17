#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_async::spi::{Operation, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;

use esp_backtrace as _;
use esp_println as _;

use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Level, Output},
    spi::{
        master::{Config, Spi},
        Mode,
    },
    time::RateExtU32,
    timer::{timg::TimerGroup},
};

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    defmt::debug!("Init!");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Init SPI and LoRa
    let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(1024);
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    /*
    let sclk = peripherals.GPIO5;
    let miso = peripherals.GPIO19;
    let mosi = peripherals.GPIO27;
    */
    let sclk = peripherals.GPIO5;
    let miso = peripherals.GPIO12;
    let mosi = peripherals.GPIO13;
    let cs = Output::new(peripherals.GPIO18, Level::Low);

    let spi = Spi::new(
        peripherals.SPI2,
        Config::default()
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

    let mut spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();

    let write_buf = &[0xaa];
    let mut read: [u8; 2] = [0xab; 2];
    let mut ops = [Operation::Write(write_buf), Operation::Read(&mut read)];
    let _ = spi_dev.transaction(&mut ops).await;
    defmt::info!("Test 1... result: {:?}", read);
    // why do we get 0xff, 0xff here?
    assert_eq!(read, [0xab, 0xab]);

    let write_buf = &[0xaa, 0xaa];
    let mut read: [u8; 1] = [0xabu8; 1];
    let mut ops = [Operation::Write(write_buf), Operation::Read(&mut read)];
    let _ = spi_dev.transaction(&mut ops).await;
    defmt::info!("Test 2... result: {:?}", read);
    // ... but 0x00 here?
    //assert_eq!(read, [0xff]);

    let write_buf = &[0xaa, 0xaa, 0xaa];
    let mut read: [u8; 2] = [0xab; 2];
    let mut ops = [Operation::Write(write_buf), Operation::Read(&mut read)];
    let _ = spi_dev.transaction(&mut ops).await;
    defmt::info!("Test 1... result: {:?}", read);
    // why do we get 0xff, 0xff here?
    assert_eq!(read, [0xab, 0xab]);

    loop {
        defmt::info!("MAIN LOOP!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}
