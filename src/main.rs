#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};

use esp_backtrace as _;
use esp_println as _;

use esp_hal::{
    clock::ClockControl,
    gpio::{GpioPin, Io, Level, Output},
    i2c::I2C,
    peripherals::{Peripherals, I2C0},
    prelude::*,
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

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

type OledIface = I2CInterface<I2C<'static, I2C0, esp_hal::Async>>;

#[main]
async fn main(spawner: Spawner) {
    defmt::debug!("Init!");

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let timer0 = OneShotTimer::new(timg0.timer0.into());
    let timers = [timer0];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);

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

    let iface = I2CDisplayInterface::new(i2c0);

    defmt::debug!("Init i2c!");

    let oled_reset = Output::new(io.pins.gpio16, Level::Low);

    spawner.spawn(oled_task(iface, oled_reset)).ok();

    loop {
        defmt::info!("MAIN LOOP!");
        Timer::after(Duration::from_millis(1_000)).await;
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
