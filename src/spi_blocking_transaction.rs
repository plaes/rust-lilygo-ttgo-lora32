#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println as _;

use embedded_hal::spi::{Operation, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    // dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    // dma_buffers,
    gpio::{Io, NO_PIN},
    prelude::*,
    spi::{master::Spi, SpiMode},
};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    /*
    let sclk = io.pins.gpio0;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio4;
    let cs = io.pins.gpio5;
    */

    let sclk = io.pins.gpio5;
    let miso = io.pins.gpio12;
    let mosi = io.pins.gpio13;
    let cs = esp_hal::gpio::Output::new(io.pins.gpio18, esp_hal::gpio::Level::Low);

    /*
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.spi2channel;

    let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(32000);
    let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    */

    let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0).with_pins(
        Some(sclk),
        Some(mosi),
        Some(miso),
        NO_PIN,
    );
    // .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

    let delay = Delay::new();

    let mut spi = ExclusiveDevice::new(spi, cs, delay).unwrap();

    loop {
        let write_buf = &[0xaa];
        let mut read_buf: [u8; 2] = [0xab; 2];
        let mut ops = [Operation::Write(write_buf), Operation::Read(&mut read_buf)];
        spi.transaction(&mut ops).unwrap();
        defmt::info!("Test 1... result: {:?}", read_buf);

        let write_buf = &[0xaa, 0xaa];
        let mut read_buf: [u8; 1] = [0xabu8; 1];
        let mut ops = [Operation::Write(write_buf), Operation::Read(&mut read_buf)];
        spi.transaction(&mut ops).unwrap();
        defmt::info!("Test 2... result: {:?}", read_buf);

        let write_buf = &[0xaa, 0xaa, 0xaa];
        let mut read_buf: [u8; 2] = [0xab; 2];
        let mut ops = [Operation::Write(write_buf), Operation::Read(&mut read_buf)];
        spi.transaction(&mut ops).unwrap();
        defmt::info!("Test 3... result: {:?}", read_buf);

        delay.delay_millis(2000);
    }
}
