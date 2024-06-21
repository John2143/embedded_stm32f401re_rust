#![no_std]
#![no_main]

use bme280::i2c::AsyncBME280;
use defmt::println;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::InterruptExecutor;
use embassy_futures::{
    join::{join, join3},
    select::{select, Either},
};
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed},
    i2c,
    peripherals::{
        self, DMA1_CH5, DMA1_CH7, DMA2_CH0, DMA2_CH3, EXTI9, I2C1, PA6, PA7, PA8, PA9, PB3, PB6,
        PB8, PB9, PC13, SPI1, TIM1,
    },
    time::Hertz,
    timer::simple_pwm::{PwmPin, SimplePwm},
    Peripheral, PeripheralRef,
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Receiver, Sender},
};
use embassy_time::{Delay, Duration, Instant, Timer};
use icm20948_async::{AccRange, GyrUnit, Icm20948};
use static_cell::StaticCell;

use core::sync::atomic::{AtomicU32, Ordering};

use defmt_rtt as _;
use panic_probe as _;

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

type MUTEX = CriticalSectionRawMutex;

static SIGNAL_A: AtomicU32 = AtomicU32::new(1);
static SIGNAL_B: AtomicU32 = AtomicU32::new(900);
//static SIGNAL_C: AtomicU32 = AtomicU32::new(900);

bind_interrupts!(struct Irqs {
    //USB => usb::InterruptHandler<peripherals::USB>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_NORMAL: InterruptExecutor = InterruptExecutor::new();
static IR_CHANNEL: StaticCell<IrChannel> = StaticCell::new();

#[cortex_m_rt::entry]
fn main() -> ! {

    let p = embassy_stm32::init(Default::default());

    let timing_channel = IR_CHANNEL.init(IrChannel::new());
    let rx = timing_channel.receiver();
    let tx = timing_channel.sender();

    let ir_input = InputIRLoop {
        pin: p.PA9.into_ref(),
        exti: p.EXTI9.into_ref(),
        tx,
    };

    let chan = embassy_stm32::pac::interrupt::EXTI2;
    let spawner = EXECUTOR_HIGH.start(chan);
    spawner.spawn(ir(ir_input)).unwrap();

    let main_input = InputMainLoop {
        cs_spi1: p.PB6.into_ref(),

        i2c_channel: p.I2C1.into_ref(),
        i2c_sda: p.PB9.into_ref(),
        i2c_scl: p.PB8.into_ref(),
        i2c_dma_tx: p.DMA1_CH7.into_ref(),
        i2c_dma_rx: p.DMA1_CH5.into_ref(),

        spi_channel: p.SPI1.into_ref(),
        spi_sck: p.PB3.into_ref(),
        spi_mosi: p.PA7.into_ref(),
        spi_miso: p.PA6.into_ref(),
        spi_dma_tx: p.DMA2_CH3.into_ref(),
        spi_dma_rx: p.DMA2_CH0.into_ref(),

        button: p.PC13.into_ref(),
        ir_output_timer: p.TIM1.into_ref(),
        ir_output_pin: p.PA8.into_ref(),

        rx,
    };

    let spawner = EXECUTOR_NORMAL.start(chan);
    spawner.spawn(main2(main_input)).unwrap();

    let mut i = 0u32;
    loop {
        i += 1;
        if i % 100 == 0 {
            println!("main loop {}", i);
        }
        cortex_m::asm::wfi();
    }
}

type IrType = (Duration, Duration);
const IR_COUNT: usize = 100;
type IrChannel = embassy_sync::channel::Channel<MUTEX, IrType, IR_COUNT>;

struct InputIRLoop {
    pin: PeripheralRef<'static, PA9>,
    exti: PeripheralRef<'static, EXTI9>,
    tx: Sender<'static, MUTEX, IrType, IR_COUNT>,
}

async fn ir(ins: InputIRLoop) {
    let led_in = Input::new(ins.pin, Pull::None);
    let mut led_in = ExtiInput::new(led_in, ins.exti);

    println!("Running IR Program");

    let mut t2 = Instant::now();
    loop {
        led_in.wait_for_low().await;
        let t1 = Instant::now();
        let time_high = t1 - t2;
        led_in.wait_for_high().await;
        t2 = Instant::now();
        let time_low = t2 - t1;

        let _ = ins.tx.try_send((time_high, time_low));
    }
}

struct InputMainLoop {
}

#[embassy_executor::task]
async fn main2(ins: InputMainLoop) {
    println!("Running main Program");
    let cs_icm20948 = Output::new(ins.cs_spi1, Level::High, Speed::High);
    // Internally, the led has a pullup resistor

    let shared_i2c_bus = {
        let spd = Hertz::hz(250_000);
        let mut cfg = embassy_stm32::i2c::Config::default();
        cfg.sda_pullup = true;
        cfg.scl_pullup = true;

        let i2c = embassy_stm32::i2c::I2c::new(
            ins.i2c_channel,
            ins.i2c_scl,
            ins.i2c_sda,
            Irqs,
            ins.i2c_dma_tx,
            ins.i2c_dma_rx,
            spd,
            cfg,
        );
        embassy_sync::mutex::Mutex::<CriticalSectionRawMutex, _>::new(i2c)
    };

    let shared_spi_bus = {
        let mut cfg = embassy_stm32::spi::Config::default();
        cfg.frequency = Hertz::hz(1_000_000); // up to 7mhz
        cfg.bit_order = embassy_stm32::spi::BitOrder::MsbFirst;
        cfg.mode = embassy_stm32::spi::MODE_0;
        let spi = embassy_stm32::spi::Spi::new(
            ins.spi_channel,
            ins.spi_sck,
            ins.spi_mosi,
            ins.spi_miso,
            ins.spi_dma_tx,
            ins.spi_dma_rx,
            cfg,
        );
        embassy_sync::mutex::Mutex::<CriticalSectionRawMutex, _>::new(spi)
    };

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
        println!(
            "BME: \n\tpressure: {}\n\ttemp: {}C ({}f)\n\thumid: {}",
            m.pressure, m.temperature, temp_f, m.humidity
        );

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
            .await
        {
            Ok(mut icm) => {
                let stuff = icm.read_9dof().await.unwrap();
                println!(
                    "ICM20948: \n acc: {} {} {}\n gyr: {} {} {}\n mag: {} {} {}",
                    stuff.acc.x,
                    stuff.acc.y,
                    stuff.acc.z,
                    stuff.gyr.x,
                    stuff.gyr.y,
                    stuff.gyr.z,
                    stuff.mag.x,
                    stuff.mag.y,
                    stuff.mag.z
                );
                Some(icm)
            }
            Err(_) => {
                println!("ICM20948 i2c init failed!");
                None
            }
        }
    };

    //let cs_icm20948 = Output::new(p.PA3, Level::High, Speed::High);
    let imu_bus =
        embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&shared_spi_bus, cs_icm20948);
    let get_sensor_imu_spi = async {
        match Icm20948::new_spi(imu_bus, Delay)
            .gyr_unit(GyrUnit::Dps)
            .gyr_dlp(icm20948_async::GyrDlp::Hz24)
            .acc_range(AccRange::Gs8)
            .initialize_6dof()
            .await
        {
            Ok(mut icm) => {
                let stuff = icm.read_6dof().await.unwrap();
                println!(
                    "ICM20948 spi: \n acc: {} {} {}\n gyr: {} {} {}\n",
                    stuff.acc.x, stuff.acc.y, stuff.acc.z, stuff.gyr.x, stuff.gyr.y, stuff.gyr.z
                ); //, stuff.mag.x, stuff.mag.y, stuff.mag.z);
                Some(icm)
            }
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
        let mut pwm = embassy_stm32::timer::simple_pwm::SimplePwm::new(
            ins.ir_output_timer,
            Some(PwmPin::new_ch1(
                ins.ir_output_pin,
                embassy_stm32::gpio::OutputType::PushPull,
            )),
            None,
            None,
            None,
            Hertz::khz(38),
            embassy_stm32::timer::CountingMode::EdgeAlignedUp,
        );

        let max = pwm.get_max_duty();
        pwm.set_duty(embassy_stm32::timer::Channel::Ch1, max / 2);
        pwm.enable(embassy_stm32::timer::Channel::Ch1);

        async fn set0(pwm: &mut SimplePwm<'_, TIM1>) {
            pwm.set_duty(embassy_stm32::timer::Channel::Ch1, 0);
        }

        async fn set1(pwm: &mut SimplePwm<'_, TIM1>) {
            let max = pwm.get_max_duty();
            pwm.set_duty(embassy_stm32::timer::Channel::Ch1, max / 2);
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
            if SIGNAL_A.load(Ordering::SeqCst) > 0 {
                SIGNAL_A.fetch_sub(1, Ordering::SeqCst);
                join(set1(&mut pwm), Timer::after_millis(8)).await;
                join(set0(&mut pwm), Timer::after_micros(4_500)).await;

                for _ in 0..1 {
                    //let data: u32 = 0b1110_0110_0000_1001_0110_0111_1001_1000; // power on command
                    //let data = !data;
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

            Timer::after_millis(1000).await;
        }
    };

    let button = async {
        let button = embassy_stm32::gpio::Input::new(ins.button, Pull::Up);
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

    let prints = async {
        loop {
            //println!("new_ticks {}, {:?}", read.len(), read.get(0..20));
            let mut buf = [(0, 0); 1024];
            let mut i = 0;
            let mut start = Instant::now();
            let final_buf = loop {
                if i == 1024 {
                    // buffer full
                    break &buf[0..1024];
                }
                match select(ins.rx.receive(), Timer::after_millis(10)).await {
                    // We got a IR packet
                    Either::First(a) => {
                        if i == 1 {
                            start = Instant::now();
                        }
                        buf[i] = (a.0.as_micros() as u32, a.1.as_micros() as u32);
                        i += 1;
                    }
                    // Timeout
                    Either::Second(_) => {
                        if i > 0 {
                            break &buf[0..i];
                        } else {
                            Timer::after_millis(250).await;
                        }
                    }
                }
            };
            let time = Instant::now() - start;
            println!("ticks: {} {} =  {}", final_buf.len(), time, final_buf);
            Timer::after_millis(100).await;
        }
    };

    // Blinking LED example
    //println!("Starting blinking program");
    //_spawner.spawn(blink(p.PA5.degrade())).unwrap();

    //let bme = get_sensor_bme.await;
    //let imu = get_sensor_imu_i2c.await;
    //let imu_spi = get_sensor_imu_spi.await;

    //Timer::after_millis(100).await;

    join3(button, servo, prints).await;
    //servo.await;
    //let ptr = shared_spi_bus.lock().await;
}