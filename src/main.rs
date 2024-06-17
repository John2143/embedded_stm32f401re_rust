#![no_std]
#![no_main]

use bme280::i2c::AsyncBME280;
use defmt::println;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::{
    bind_interrupts, gpio::{AnyPin, Level, Output, Pin, Pull, Speed}, i2c, peripherals, time::Hertz, PeripheralRef
};
use embassy_time::{Delay, Timer};

use core::sync::atomic::{AtomicU32, Ordering};

use defmt_rtt as _;
use panic_probe as _;

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, Speed::High);

    loop {
        let timing_a = SIGNAL_A.load(Ordering::SeqCst) as u64;
        let timing_b = SIGNAL_B.load(Ordering::SeqCst) as u64;

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

    let mut bme = {
        let spd = Hertz::hz(250_000);
        let mut cfg = embassy_stm32::i2c::Config::default();
        cfg.sda_pullup = true;
        cfg.scl_pullup = true;

        let i2c = embassy_stm32::i2c::I2c::new(PeripheralRef::new(p.I2C1), PeripheralRef::new(p.PB8), PeripheralRef::new(p.PB9), Irqs, PeripheralRef::new(p.DMA1_CH7), PeripheralRef::new(p.DMA1_CH5), spd, cfg);
        AsyncBME280::new_secondary(i2c)
    };


    let bme_init_res = bme.init(&mut Delay).await;
    if bme_init_res.is_err() {
        println!("BME280 init failed!");
        return;
    }

    println!("Starting blinking program");
    spawner.spawn(blink(p.PA5.degrade())).unwrap();

    // Measure BME280 (temp, pressure, humid) data once
    let m = bme.measure(&mut Delay).await.unwrap();
    let temp_f = m.temperature * 9.0 / 5.0 + 32.0;
    println!("BME: \n\tpressure: {}\n\ttemp: {}C ({}f)\n\thumid: {}", m.pressure, m.temperature, temp_f, m.humidity);

    let button = embassy_stm32::gpio::Input::new(p.PC13, Pull::Up);
    loop {
        // we use a pullup resistor, so the button is active low
        while button.is_high() {
            Timer::after_millis(10).await;
        }

        println!("Button pressed!");
        // Blink faster
        SIGNAL_A.store(50, Ordering::SeqCst);
        SIGNAL_B.store(50, Ordering::SeqCst);

        //while button.is_low() {
            //join(async {
                //let m = bme.measure(&mut Delay).await.unwrap();
                //println!("BME: \n\tpressure: {}\n\ttemp: {}\n\thumid: {}", m.pressure, m.temperature, m.humidity);
            //}, async {
                //Timer::after_millis(1000).await
            //}).await;
        //}
        while button.is_low() {
            Timer::after_millis(10).await;
        }

        println!("Button released!");
        // Blink slower
        SIGNAL_A.store(100, Ordering::SeqCst);
        SIGNAL_B.store(900, Ordering::SeqCst);
    }
}
