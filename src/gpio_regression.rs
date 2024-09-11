#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use esp_backtrace as _;
use esp_println as _;

use esp_hal::{
    gpio::{AnyPin, Input, Io, Level, Output, Pull},
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
};

use static_cell::StaticCell;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        STATIC_CELL.uninit().write(($val))
    }};
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    defmt::debug!("Init!");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: ErasedTimer = timg0.timer0.into();
    let timers = [OneShotTimer::new(timer0)];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);

    esp_hal_embassy::init(timers);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    defmt::debug!("Prepare GPIO!");

    let prg_button = io.pins.gpio0;
    spawner.spawn(button_detect(AnyPin::new(prg_button))).ok();

    let blue_led = io.pins.gpio2;
    spawner.spawn(led_blinker(AnyPin::new(blue_led))).ok();

    loop {
        defmt::info!("MAIN LOOP!");
        Timer::after(Duration::from_millis(5_000)).await;
    }
}

#[embassy_executor::task]
async fn led_blinker(pin: AnyPin<'static>) {
    let mut led = Output::new(pin, Level::High);

    loop {
        Timer::after(Duration::from_millis(500)).await;
        led.toggle();
    }
}

#[embassy_executor::task]
async fn button_detect(pin: AnyPin<'static>) {
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
