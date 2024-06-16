#![no_std]
#![no_main]

use defmt::{info, println};
use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Level, Output, Pin, Pull, Speed};
use embassy_time::Timer;

use defmt_rtt as _;
use panic_probe as _;

// Declare async tasks
#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, Speed::High);

    loop {
        // Timekeeping is globally available, no need to mess with hardware timers.
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        Timer::after_millis(900).await;
    }
}

// Main is itself an async task as well.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    println!("Starting blinking program");
    spawner.spawn(blink(p.PA5.degrade())).unwrap();

    let mut button = embassy_stm32::gpio::Input::new(p.PC13, Pull::Up);
    loop {
        // Asynchronously wait for GPIO events, allowing other tasks
        // to run, or the core to sleep.
        while button.is_high() {
            Timer::after_millis(10).await;
        }
        println!("Button pressed!");
        while button.is_low() {
            Timer::after_millis(10).await;
        }
        println!("Button released!");
    }
}
