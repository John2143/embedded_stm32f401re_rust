#![no_std]
#![no_main]

use bme280::i2c::AsyncBME280;
use defmt::println;
use embassy_embedded_hal::shared_bus::asynch::{i2c::I2cDevice};
use embassy_executor::Spawner;
use embassy_futures::join::{join, join3, join4};
use embassy_stm32::{
    bind_interrupts, gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed}, i2c, peripherals::{self, TIM1}, time::Hertz, timer::simple_pwm::{PwmPin, SimplePwm}
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Timer};
use icm20948_async::{AccRange, GyrUnit, Icm20948};

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
static SIGNAL_C: AtomicU32 = AtomicU32::new(900);

bind_interrupts!(struct Irqs {
    //USB => usb::InterruptHandler<peripherals::USB>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

// Main is itself an async task as well.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let cs_icm20948 = Output::new(p.PB6, Level::High, Speed::High);
    let led_in = Input::new(p.PA9, Pull::None);
    // Internally, the led has a pullup resistor

    let shared_i2c_bus = {
        let spd = Hertz::hz(250_000);
        let mut cfg = embassy_stm32::i2c::Config::default();
        cfg.sda_pullup = true;
        cfg.scl_pullup = true;

        let i2c = embassy_stm32::i2c::I2c::new(p.I2C1, p.PB8, p.PB9, Irqs, p.DMA1_CH7, p.DMA1_CH5, spd, cfg);
        Mutex::<NoopRawMutex, _>::new(i2c)
    };

    let shared_spi_bus = {
        let mut cfg = embassy_stm32::spi::Config::default();
        cfg.frequency = Hertz::hz(1_000_000); // up to 7mhz
        cfg.bit_order = embassy_stm32::spi::BitOrder::MsbFirst;
        cfg.mode = embassy_stm32::spi::MODE_0;
        let spi = embassy_stm32::spi::Spi::new(p.SPI1, p.PB3, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH0, cfg);
        Mutex::<NoopRawMutex, _>::new(spi)
    };

    // We need to write something to the bus so is it left high (mode0)
    shared_spi_bus.lock().await.write(&[0u8]).await.unwrap();
    Timer::after_millis(1).await;
    shared_spi_bus.lock().await.write(&[2u8, 1, 4, 3]).await.unwrap();

    // BME280 example
    let get_sensor_bme = async {
        let bus = I2cDevice::new(&shared_i2c_bus);
        let mut bme = AsyncBME280::new_secondary(bus);
        let bme_init_res = bme.init(&mut Delay).await;
        if bme_init_res.is_err() {
            println!("BME280 init failed!");
            return None;
        }

        let m = bme.measure(&mut Delay).await.unwrap();
        let temp_f = m.temperature * 9.0 / 5.0 + 32.0;
        println!("BME: \n\tpressure: {}\n\ttemp: {}C ({}f)\n\thumid: {}", m.pressure, m.temperature, temp_f, m.humidity);

        Some(bme)
    };


    // ICM20948 example
    let get_sensor_imu_i2c = async {
        let bus = I2cDevice::new(&shared_i2c_bus);
        match Icm20948::new_i2c(bus, Delay)
            .gyr_unit(GyrUnit::Dps)
            .gyr_dlp(icm20948_async::GyrDlp::Hz24)
            .acc_range(AccRange::Gs8)
            .set_address(0x69)
            .initialize_9dof()
            .await {
                Ok(mut icm) => {
                    let stuff = icm.read_9dof().await.unwrap();
                    println!("ICM20948: \n acc: {} {} {}\n gyr: {} {} {}\n mag: {} {} {}", stuff.acc.x, stuff.acc.y, stuff.acc.z, stuff.gyr.x, stuff.gyr.y, stuff.gyr.z, stuff.mag.x, stuff.mag.y, stuff.mag.z);
                    Some(icm)
                },
                Err(_) => {
                    println!("ICM20948 i2c init failed!");
                    None
                }
        }
    };

    //let cs_icm20948 = Output::new(p.PA3, Level::High, Speed::High);
    let imu_bus = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&shared_spi_bus, cs_icm20948);
    let get_sensor_imu_spi = async {
        match Icm20948::new_spi(imu_bus, Delay)
            .gyr_unit(GyrUnit::Dps)
            .gyr_dlp(icm20948_async::GyrDlp::Hz24)
            .acc_range(AccRange::Gs8)
            .initialize_6dof()
            .await {
                Ok(mut icm) => {
                    let stuff = icm.read_6dof().await.unwrap();
                    println!("ICM20948 spi: \n acc: {} {} {}\n gyr: {} {} {}\n", stuff.acc.x, stuff.acc.y, stuff.acc.z, stuff.gyr.x, stuff.gyr.y, stuff.gyr.z); //, stuff.mag.x, stuff.mag.y, stuff.mag.z);
                    Some(icm)
                },
                Err(e) => {
                    println!("ICM20948 spi init failed:");
                    match e {
                        icm20948_async::IcmError::BusError(_e) => println!("Bus error"),
                        icm20948_async::IcmError::ImuSetupError => println!("IMU setup error"),
                        icm20948_async::IcmError::MagSetupError => println!("MAG setup error"),
                    };
                    None
                }
        }
    };


    // PWM servo example
    let servo = async {
        // D6 / PB10 = TIM2CH3
        // D5 / PB4 = TIM3CH1
        // D3 / PB3 = TIM2CH2
        //
        // D7 / PA8 = TIM1CH1
        let mut pwm = embassy_stm32::timer::simple_pwm::SimplePwm::new(p.TIM1, Some(PwmPin::new_ch1(p.PA8, embassy_stm32::gpio::OutputType::PushPull)), None, None, None, Hertz::khz(38), embassy_stm32::timer::CountingMode::EdgeAlignedUp);

        let max = pwm.get_max_duty();
        pwm.set_duty(embassy_stm32::timer::Channel::Ch1, max/2);
        pwm.enable(embassy_stm32::timer::Channel::Ch1);

        async fn set0(pwm: &mut SimplePwm<'_, TIM1>) {
            pwm.set_duty(embassy_stm32::timer::Channel::Ch1, 0);
        }

        async fn set1(pwm: &mut SimplePwm<'_, TIM1>) {
            let max = pwm.get_max_duty();
            pwm.set_duty(embassy_stm32::timer::Channel::Ch1, max/2);
        }

        const INSTR_TIME: i64 = 60_000;
        const A: u64 = (562_500i64 - INSTR_TIME) as u64;
        const B: u64 = (1_687_500i64 - INSTR_TIME * 3) as u64;

        async fn send0(pwm: &mut SimplePwm<'_, TIM1>) {
            set1(pwm).await;
            Timer::after_nanos(A).await;
            set0(pwm).await;
            Timer::after_nanos(A).await;
        }

        async fn send1(pwm: &mut SimplePwm<'_, TIM1>) {
            set1(pwm).await;
            Timer::after_nanos(A).await;
            set0(pwm).await;
            Timer::after_nanos(B).await;
        }


        loop {
            //let data: u32 = 0b1110_0110_0000_1001_0110_0111_1001_1000; // power on command
            //let data = !data;
            if SIGNAL_A.load(Ordering::SeqCst) > 0{
                join(
                    set1(&mut pwm),
                    Timer::after_millis(8)
                ).await;
                join(
                    set0(&mut pwm),
                    Timer::after_micros(4_500)
                ).await;

                for _ in 0..10 {
                    let data: u32 = 0b0000_0000_0000_0000_1100_0000_0111_1000; // vol up
                    let range = if data < 0x0001_0000 { 16 } else { 32 };
                    for i in (0..range).rev() {
                        let bit = (data >> i) & 1;
                        if bit == 1 {
                            send1(&mut pwm).await;
                        } else {
                            send0(&mut pwm).await;
                        }
                    }

                    send0(&mut pwm).await;
                    Timer::after_millis(20).await;
                }
            }

            Timer::after_millis(100).await;
        }
    };

    let button = async {

        let button = embassy_stm32::gpio::Input::new(p.PC13, Pull::Up);
        loop {
            // we use a pullup resistor, so the button is active low
            while button.is_high() {
                Timer::after_millis(10).await;
            }
            println!("Button pressed!");

            if SIGNAL_A.load(Ordering::SeqCst) > 0 {
                SIGNAL_A.store(0, Ordering::SeqCst);
            } else {
                SIGNAL_A.store(100, Ordering::SeqCst);
            }

            while button.is_low() {
                Timer::after_millis(10).await;
            }

            println!("Button released!");
        }
    };

    let get_led = async {
        loop {
            let mut ticks_h = 0;
            while led_in.is_high() {
                ticks_h += 1;
                Timer::after_micros(10).await;
            };

            let mut ticks_l = 0;
            while led_in.is_low() {
                ticks_l += 1;
                Timer::after_micros(10).await;
            };

            SIGNAL_B.store(ticks_h, Ordering::SeqCst);
            SIGNAL_C.store(ticks_l, Ordering::SeqCst);
        }
    };

    let prints = async {
        loop {
            let ticks_h = SIGNAL_B.load(Ordering::SeqCst);
            let ticks_l = SIGNAL_C.load(Ordering::SeqCst);
            println!("stayed h/l for {}0/{}0us", ticks_h, ticks_l);
            Timer::after_millis(1000).await;
        }
    };

    // Blinking LED example
    //println!("Starting blinking program");
    //_spawner.spawn(blink(p.PA5.degrade())).unwrap();

    //let bme = get_sensor_bme.await;
    //let imu = get_sensor_imu_i2c.await;
    //let imu_spi = get_sensor_imu_spi.await;

    //Timer::after_millis(100).await;

    join4(button, servo, get_led, prints).await;
    //servo.await;
    //let ptr = shared_spi_bus.lock().await;
}
