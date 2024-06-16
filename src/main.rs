#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, Speed, Pin, Pull};
use embassy_stm32::Peripherals;

use panic_probe as _;
use defmt_rtt as _;

// Declare async tasks
#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, Speed::High);

    loop {
        // Timekeeping is globally available, no need to mess with hardware timers.
        led.set_high();
        Timer::after_millis(500).await;
        led.set_low();
        Timer::after_millis(200).await;
    }
}

// Main is itself an async task as well.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    // Spawned tasks run in the background, concurrently.
    spawner.spawn(blink(p.PA5.degrade())).unwrap();
    //spawner.spawn(blink(p.PB13.degrade())).unwrap();
    loop {
        Timer::after_secs(5).await;
    }

    // let mut button = Input::new(p.PC13, Pull::Up);
    // loop {
    //     // Asynchronously wait for GPIO events, allowing other tasks
    //     // to run, or the core to sleep.
    //     button.wait_for_low().await;
    //     info!("Button pressed!");
    //     button.wait_for_high().await;
    //     info!("Button released!");
    // }
}
