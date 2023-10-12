#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    mono_font::{
        ascii::{FONT_6X10, FONT_9X18_BOLD},
        MonoTextStyleBuilder,
    },
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Text},
};
use esp_backtrace as _;
use esp_println::println;
use hal::dma::DmaPriority;
use hal::spi::master::dma::SpiDma;
use hal::{
    clock::ClockControl,
    embassy,
    gpio::{AnyPin, Output, PushPull, IO},
    i2c::I2C,
    interrupt, pdma,
    peripherals::{Interrupt, Peripherals, I2C0, SPI2},
    prelude::*,
    spi::{master::Spi, FullDuplexMode, SpiMode},
    timer::TimerGroup,
    Delay,
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use static_cell::StaticCell;

#[embassy_executor::task]
async fn handle_display(
    i2c: I2C<'static, I2C0>,
    mut reset: AnyPin<Output<PushPull>>,
    mut delay: Delay,
) {
    let iface = I2CDisplayInterface::new(i2c);
    println!("Starting display loop!");

    let mut display = Ssd1306::new(iface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.reset(&mut reset, &mut delay).unwrap();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let text_style_big = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .build();

    loop {
        esp_println::println!("task: display!");

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

        // Write buffer to display
        display.flush().unwrap();
        // Clear display buffer
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

        // Write buffer to display
        display.flush().unwrap();
        // Clear display buffer
        display.clear(BinaryColor::Off).unwrap();

        Timer::after(Duration::from_millis(3_000)).await;
    }
}

/*
#[embassy_executor::task]
async fn run2(spi: SpiDma<'static, SPI2, FullDuplexMode>) {
    let send_buffer = [0, 1, 2, 3, 4, 5, 6, 7];
    loop {
        let mut buffer = [0; 8];
        esp_println::println!("Sending bytes");
        embedded_hal_async::spi::SpiBus::transfer(&mut spi, &mut buffer, &send_buffer)
            .await
            .unwrap();
        esp_println::println!("Bytes recieved: {:?}", buffer);
        Timer::after(Duration::from_millis(5_000)).await;
    }
}
*/

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    esp_println::println!("Init!");

    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0.timer0);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Initialize I2C for display
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio4,
        io.pins.gpio15,
        100u32.kHz(),
        &clocks,
    );

    let oled_rst = io.pins.gpio16.into_push_pull_output();
    let oled_dly = Delay::new(&clocks);

    let dma = pdma::Dma::new(system.dma);
    let dma_channel = dma.spi2channel;

    let mut descriptors = [0u32; 8 * 3];
    let mut rx_descriptors = [0u32; 8 * 3];

    // Initialize SPI for LoRa
    /*
     * #define SCK 5
     * #define MISO 19
     * #define MOSI 27
     * #define SS 18
     * #define RST 14
     * #define DIO0 26
     */
    let mut spi = Spi::new(
        peripherals.SPI2,
        io.pins.gpio5,  // sck
        io.pins.gpio27, // mosi
        io.pins.gpio19, // miso
        io.pins.gpio18, // ss
        100u32.kHz(),
        SpiMode::Mode0,
        &clocks,
    )
    .with_dma(dma_channel.configure(
        false,
        &mut descriptors,
        &mut rx_descriptors,
        DmaPriority::Priority0,
    ));

    // spi.send(0x42).unwrap();

    // let x:u8 = spi.read().unwrap();

    // esp_println::println!("Data: {}!", x);

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner
            .spawn(handle_display(i2c, oled_rst.into(), oled_dly))
            .ok();
        // spawner.spawn(run2(spi)).ok();
    });
    /*
    let send_buffer = [0, 1, 2, 3, 4, 5, 6, 7];
    loop {
        let mut buffer = [0; 8];
        esp_println::println!("Sending bytes");
        embedded_hal_async::spi::SpiBus::transfer(&mut spi, &mut buffer, &send_buffer)
            .await
            .unwrap();
        esp_println::println!("Bytes recieved: {:?}", buffer);
        Timer::after(Duration::from_millis(5_000)).await;
    }
    */
}
