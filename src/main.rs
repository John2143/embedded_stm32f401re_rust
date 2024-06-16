#![no_std]
#![no_main]

use defmt::println;
use embassy_executor::Spawner;
use embassy_stm32::{gpio::{AnyPin, Level, Output, Pin, Pull, Speed}};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::Timer;

use core::{borrow::BorrowMut, cell::{OnceCell, RefCell}, sync::atomic::{AtomicU32, Ordering}};

use defmt_rtt as _;
use panic_probe as _;

// Declare async tasks
#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, Speed::High);

    loop {
        let timing_a = SIGNAL_A.load(Ordering::SeqCst) as u64;
        let timing_b = SIGNAL_B.load(Ordering::SeqCst) as u64;
        // Timekeeping is globally available, no need to mess with hardware timers.
        led.set_high();
        Timer::after_millis(timing_a).await;
        led.set_low();
        Timer::after_millis(timing_b).await;
    }
}

static SIGNAL_A: AtomicU32 = AtomicU32::new(100);
static SIGNAL_B: AtomicU32 = AtomicU32::new(900);

// Main is itself an async task as well.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    println!("Starting blinking program");
    SIGNAL_A.store(100, Ordering::SeqCst);
    spawner.spawn(blink(p.PA5.degrade())).unwrap();

    let button = embassy_stm32::gpio::Input::new(p.PC13, Pull::Up);
    loop {
        while button.is_high() {
            Timer::after_millis(10).await;
        }
        println!("Button pressed!");
        SIGNAL_A.store(50, Ordering::SeqCst);
        SIGNAL_B.store(50, Ordering::SeqCst);

        while button.is_low() {
            Timer::after_millis(10).await;
        }
        println!("Button released!");
        SIGNAL_A.store(100, Ordering::SeqCst);
        SIGNAL_B.store(900, Ordering::SeqCst);
    }
}
