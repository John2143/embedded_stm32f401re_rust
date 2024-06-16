#![no_std]
#![no_main]

use defmt::println;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts, dma::NoDma, gpio::{AnyPin, Level, Output, Pin, Pull, Speed}, i2c, pac::i2c::I2c, peripherals, time::Hertz, usart::{self, UartTx}
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::Timer;

use core::{
    borrow::BorrowMut,
    cell::{OnceCell, RefCell},
    sync::atomic::{AtomicU32, Ordering},
};

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

bind_interrupts!(struct Irqs {
    //USB => usb::InterruptHandler<peripherals::USB>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

// Main is itself an async task as well.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let spd = Hertz::hz(115200);
    let mut cfg = embassy_stm32::i2c::Config::default();
    cfg.sda_pullup = true;
    cfg.scl_pullup = true;
    let mut i2c = embassy_stm32::i2c::I2c::new(p.I2C1, p.PB8, p.PB9, Irqs, p.DMA1_CH6, p.DMA1_CH1, spd, cfg);

    println!("Starting blinking program");
    SIGNAL_A.store(100, Ordering::SeqCst);
    spawner.spawn(blink(p.PA5.degrade())).unwrap();

    let button = embassy_stm32::gpio::Input::new(p.PC13, Pull::Up);
    loop {
        //let _ = usart.blocking_write(b"Hello World\n").unwrap();
        //Timer::after_millis(10).await;
        while button.is_high() {
            Timer::after_millis(10).await;
        }

        let x = i2c.write(0x25, b"yey").await;
        println!("I2C result {:?}", x);
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
