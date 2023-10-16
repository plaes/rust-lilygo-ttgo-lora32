#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]

use embassy_executor::Executor;

use embassy_time::{Delay as EDelay, Duration, Timer};
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
use hal::{
    clock::ClockControl,
    embassy,
    gpio::{AnyPin, GpioPin, Input, Output, PushPull, IO},
    i2c::I2C,
    interrupt,
    pdma::{self, Spi2DmaChannelCreator},
    peripherals::{Interrupt, Peripherals, I2C0, SPI2},
    prelude::*,
    spi::{master::Spi, FullDuplexMode, SpiMode},
    timer::TimerGroup,
    Delay,
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use static_cell::StaticCell;

mod lora_iface;
use crate::lora_iface::LoRaInterfaceVariant;
use lora_phy::mod_params::{Bandwidth, BoardType, CodingRate, ModulationParams, SpreadingFactor};
use lora_phy::sx1276_7_8_9::SX1276_7_8_9;
use lora_phy::LoRa;

#[embassy_executor::task]
async fn oled_task(i2c: I2C<'static, I2C0>, mut reset: AnyPin<Output<PushPull>>, mut delay: Delay) {
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

#[embassy_executor::task]
async fn lora_task(
    spi: Spi<'static, SPI2, FullDuplexMode>,
    dma_channel: Spi2DmaChannelCreator,
    reset: AnyPin<Output<hal::gpio::PushPull>>,
    dio0: GpioPin<Input<hal::gpio::PullDown>, 26>,
) {
    println!("SPI start...");
    let mut descriptors = [0u32; 8 * 3];
    let mut rx_descriptors = [0u32; 8 * 3];
    let mut spibus = spi.with_dma(dma_channel.configure(
        false,
        &mut descriptors,
        &mut rx_descriptors,
        DmaPriority::Priority0,
    ));

    let iv = LoRaInterfaceVariant::new(reset, dio0).unwrap();
    let mut lora = {
        match LoRa::new(
            SX1276_7_8_9::new(BoardType::GenericSx1276, spibus, iv),
            true,
            EDelay,
        )
        .await
        {
            Ok(l) => l,
            Err(err) => {
                println!("Radio error: {:#?}", err);
                return;
            }
        }
    };

    /*
    let send_buffer = [0x42];
    loop {
        let mut buffer = [0; 8];
        embedded_hal_async::spi::SpiBus::transfer(&mut bus, &mut buffer, &send_buffer)
            .await
            .unwrap();
        println!("task: {:#x?}", &buffer);

        Timer::after(Duration::from_millis(5_000)).await;
    }
    */
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    println!("Init!");

    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0.timer0);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    interrupt::enable(Interrupt::SPI2_DMA, interrupt::Priority::Priority1).unwrap();

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

    // Initialize SPI for LoRa chip
    /*
     * #define SCK 5
     * #define MISO 19
     * #define MOSI 27
     * #define SS 18
     * #define RST 14
     * #define DIO0 26
     */
    // let lora_nss = io.pins.gpio18.into_push_pull_output();
    let lora_rst = io.pins.gpio14.into_push_pull_output();
    let lora_di0 = io.pins.gpio26.into_pull_down_input();
    let spi = Spi::new(
        peripherals.SPI2,
        io.pins.gpio5,  // sck
        io.pins.gpio27, // mosi
        io.pins.gpio19, // miso
        io.pins.gpio18, // ss
        100u32.kHz(),
        SpiMode::Mode0,
        &clocks,
    );
    let dma = pdma::Dma::new(system.dma);
    let dma_channel = dma.spi2channel;

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner
            .spawn(oled_task(i2c, oled_rst.into(), oled_dly))
            .ok();
        spawner
            .spawn(lora_task(spi, dma_channel, lora_rst.into(), lora_di0))
            .ok();
    });
}
