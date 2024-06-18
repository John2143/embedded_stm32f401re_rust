#![no_std]
#![no_main]

use bme280::i2c::AsyncBME280;
use defmt::println;
use embassy_embedded_hal::shared_bus::asynch::{i2c::I2cDevice, spi::SpiDevice};
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts, gpio::{AnyPin, Level, Output, Pin, Pull, Speed}, i2c, peripherals::{self}, time::Hertz, timer::simple_pwm::PwmPin
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

bind_interrupts!(struct Irqs {
    //USB => usb::InterruptHandler<peripherals::USB>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

// Main is itself an async task as well.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    // Blinking LED example
    //println!("Starting blinking program");
    //_spawner.spawn(blink(p.PA5.degrade())).unwrap();

    let shared_i2c_bus = {
        let spd = Hertz::hz(250_000);
        let mut cfg = embassy_stm32::i2c::Config::default();
        cfg.sda_pullup = true;
        cfg.scl_pullup = true;

        let i2c = embassy_stm32::i2c::I2c::new(p.I2C1, p.PB8, p.PB9, Irqs, p.DMA1_CH7, p.DMA1_CH5, spd, cfg);
        Mutex::<NoopRawMutex, _>::new(i2c)
    };

    let cs_icm20948 = Output::new(p.PB6, Level::High, Speed::High);
    let shared_spi_bus = {
        let mut cfg = embassy_stm32::spi::Config::default();
        cfg.frequency = Hertz::hz(5_000_000); // up to 7mhz
        cfg.bit_order = embassy_stm32::spi::BitOrder::MsbFirst;
        cfg.mode = embassy_stm32::spi::MODE_0;
        let spi = embassy_stm32::spi::Spi::new(p.SPI1, p.PB3, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH0, cfg);
        Mutex::<NoopRawMutex, _>::new(spi)
    };

    Timer::after_millis(10).await;

    {
        let mut bus = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&shared_spi_bus, Output::new(p.PC7, Level::High, Speed::High));

        use embedded_hal_async::spi::SpiDevice;
        //bus.write(b"test").await.unwrap();
        println!("Test written over bus");

        let button = embassy_stm32::gpio::Input::new(p.PC13, Pull::Up);
        loop {
            // we use a pullup resistor, so the button is active low
            while button.is_high() {
                Timer::after_millis(10).await;
            }
            println!("Button pushed!");

            while button.is_low() {
                bus.write(&[1,3,8]).await.unwrap();
                //let acc = _sensor_imu.read_acc().await.unwrap();
                //println!("ICM20948: \n acc: {} {} {}", acc.x, acc.y, acc.z);
                Timer::after_millis(100).await;
            }
            println!("Button released!");
        }
    }

    Timer::after_millis(10).await;
    // BME280 example
    let _sensor_bme = {
        let bus = I2cDevice::new(&shared_i2c_bus);
        let mut bme = AsyncBME280::new_secondary(bus);
        let bme_init_res = bme.init(&mut Delay).await;
        if bme_init_res.is_err() {
            println!("BME280 init failed!");
            return;
        }

        let m = bme.measure(&mut Delay).await.unwrap();
        let temp_f = m.temperature * 9.0 / 5.0 + 32.0;
        println!("BME: \n\tpressure: {}\n\ttemp: {}C ({}f)\n\thumid: {}", m.pressure, m.temperature, temp_f, m.humidity);

        bme
    };

    Timer::after_millis(10).await;

    // ICM20948 example
    let mut sensor_imu_i2c = {
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

    let mut sensor_imu_spi = {
        let bus = SpiDevice::new(&shared_spi_bus, cs_icm20948);
        match Icm20948::new_spi(bus, Delay)
            .gyr_unit(GyrUnit::Dps)
            .gyr_dlp(icm20948_async::GyrDlp::Hz24)
            .acc_range(AccRange::Gs8)
            .initialize_9dof()
            .await {
                Ok(mut icm) => {
                    let stuff = icm.read_9dof().await.unwrap();
                    println!("ICM20948 spi: \n acc: {} {} {}\n gyr: {} {} {}\n mag: {} {} {}", stuff.acc.x, stuff.acc.y, stuff.acc.z, stuff.gyr.x, stuff.gyr.y, stuff.gyr.z, stuff.mag.x, stuff.mag.y, stuff.mag.z);
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

    Timer::after_millis(10).await;


    // PWM servo example
    {
        // D6 / PB10 = TIM2CH3
        // D5 / PB4 = TIM3CH1
        // D3 / PB3 = TIM2CH2
        //
        // D7 / PA8 = TIM1CH1
        let mut pwm = embassy_stm32::timer::simple_pwm::SimplePwm::new(p.TIM1, Some(PwmPin::new_ch1(p.PA8, embassy_stm32::gpio::OutputType::PushPull)), None, None, None, Hertz::hz(50), embassy_stm32::timer::CountingMode::EdgeAlignedUp);

        let max = pwm.get_max_duty();
        let lowest = max / 25; // 800us at 50Hz
        let highest = max / 10; // 2ms at 50Hz

        let target_percent = 0.25;
        // lerp between lowest and highest based on target_percent
        let target = lowest + ((highest - lowest) as f32 * target_percent) as u16;

        pwm.set_duty(embassy_stm32::timer::Channel::Ch1, target);
        pwm.enable(embassy_stm32::timer::Channel::Ch1);

        for i in 0..10 {
            let target_percent = i as f32 / 10.0;
            // lerp between lowest and highest based on target_percent
            let target = lowest + ((highest - lowest) as f32 * target_percent) as u16;

            pwm.set_duty(embassy_stm32::timer::Channel::Ch1, target);

            Timer::after_millis(250).await;

            if let Some(sens) = &mut sensor_imu_i2c {
                let stuff = sens.read_9dof().await.unwrap();
                println!("ICM20948: \n acc: {} {} {}\n gyr: {} {} {}\n mag: {} {} {}", stuff.acc.x, stuff.acc.y, stuff.acc.z, stuff.gyr.x, stuff.gyr.y, stuff.gyr.z, stuff.mag.x, stuff.mag.y, stuff.mag.z);
            }
        }
    };

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

        while button.is_low() {
            //let acc = _sensor_imu.read_acc().await.unwrap();
            //println!("ICM20948: \n acc: {} {} {}", acc.x, acc.y, acc.z);
            Timer::after_millis(100).await;
        }

        println!("Button released!");
        // Blink slower
        SIGNAL_A.store(100, Ordering::SeqCst);
        SIGNAL_B.store(900, Ordering::SeqCst);
    }
}
