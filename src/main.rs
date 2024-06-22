#![no_std]
#![no_main]

use bme280::i2c::AsyncBME280;
use defmt::println;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::InterruptExecutor;
use embassy_futures::{
    join::{join, join3, join4},
    select::{select, Either},
};
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed},
    i2c,
    interrupt::{self, InterruptExt},
    peripherals::{
        self, DMA1_CH5, DMA1_CH7, DMA2_CH0, DMA2_CH3, EXTI9, I2C1, PA11, PA5, PA6, PA7, PA8, PA9, PB10, PB3, PB4, PB6, PB8, PB9, PC13, SPI1, TIM1, TIM2, TIM3
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
use num_traits::real::Real;
use static_cell::StaticCell;

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

type MUTEX = CriticalSectionRawMutex;

static SIGNAL_A: AtomicU32 = AtomicU32::new(0);
static SIGNAL_B: AtomicU32 = AtomicU32::new(900);
//static SIGNAL_C: AtomicU32 = AtomicU32::new(900);


#[embassy_stm32::interrupt]
fn I2C2_EV() {
    unsafe {EXECUTOR_HIGH.on_interrupt()};
}

#[embassy_stm32::interrupt]
fn I2C2_ER() {
    unsafe {EXECUTOR_NORMAL.on_interrupt()};
}

bind_interrupts!(struct Irqs {
    //USB => usb::InterruptHandler<peripherals::USB>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

// The high priority executor is used for the GPIO interrupts
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

        onboard_led_pin: p.PA5.into_ref(),

        other_output_timer: p.TIM3.into_ref(),
        other_led_pin: p.PB4.into_ref(),

        rx,
    };

    // We use I2C2 flags for our event loops
    drop(p.I2C2);

    // Setup our two executors
    {
        let chan = embassy_stm32::pac::interrupt::I2C2_EV;
        chan.set_priority(embassy_stm32::interrupt::Priority::P2);
        let spawner = EXECUTOR_HIGH.start(chan);
        spawner.spawn(high_prio_event_lop(ir_input)).unwrap();

        let chan = embassy_stm32::pac::interrupt::I2C2_ER;
        chan.set_priority(embassy_stm32::interrupt::Priority::P8);
        let spawner = EXECUTOR_NORMAL.start(chan);
        spawner.spawn(normal_prio_event_loop(main_input)).unwrap();
    }

    loop {
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

#[embassy_executor::task]
async fn high_prio_event_lop(ins: InputIRLoop) {
    println!("Running IR Program");
    let led_in = Input::new(ins.pin, Pull::None);
    let mut led_in = ExtiInput::new(led_in, ins.exti);

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
    cs_spi1: PeripheralRef<'static, PB6>,

    i2c_channel: PeripheralRef<'static, I2C1>,
    i2c_sda: PeripheralRef<'static, PB9>,
    i2c_scl: PeripheralRef<'static, PB8>,
    i2c_dma_tx: PeripheralRef<'static, DMA1_CH7>,
    i2c_dma_rx: PeripheralRef<'static, DMA1_CH5>,

    // let spi = embassy_stm32::spi::Spi::new(p.SPI1, p.PB3, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH0, cfg);
    spi_channel: PeripheralRef<'static, SPI1>,
    spi_sck: PeripheralRef<'static, PB3>,
    spi_mosi: PeripheralRef<'static, PA7>,
    spi_miso: PeripheralRef<'static, PA6>,
    spi_dma_tx: PeripheralRef<'static, DMA2_CH3>,
    spi_dma_rx: PeripheralRef<'static, DMA2_CH0>,

    ir_output_timer: PeripheralRef<'static, TIM1>,
    ir_output_pin: PeripheralRef<'static, PA8>,

    onboard_led_pin: PeripheralRef<'static, PA5>,

    other_output_timer: PeripheralRef<'static, TIM3>,
    other_led_pin: PeripheralRef<'static, PB4>,

    button: PeripheralRef<'static, PC13>,

    rx: Receiver<'static, MUTEX, IrType, IR_COUNT>,
}

#[embassy_executor::task]
async fn normal_prio_event_loop(ins: InputMainLoop) {
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

    let normal_out = async {
        let mut pwm_a = embassy_stm32::timer::simple_pwm::SimplePwm::new(
            ins.other_output_timer,
            Some(PwmPin::new_ch1(
                ins.other_led_pin,
                embassy_stm32::gpio::OutputType::OpenDrain,
            )),
            None,
            None,
            None,
            Hertz::hz(100),
            embassy_stm32::timer::CountingMode::EdgeAlignedUp,
        );

        let max = pwm_a.get_max_duty();
        let chan = embassy_stm32::timer::Channel::Ch1;
        pwm_a.set_duty(chan, max);
        pwm_a.enable(chan);
        let mut f = 0.0;
        loop {
            for _ in 0..100 {
                f += 0.03;
                let loc = (f.sin() + 1.0) / 2.0;
                let loc = loc * loc * loc * loc;

                let loc = loc * 0.4 + 0.01;
                pwm_a.set_duty(chan, ((max as f32) * loc) as u16);
                Timer::after_millis(50).await;
            }

            // pwm_a.set_duty(chan, max);
            // Timer::after_millis(15).await;
            // pwm_a.set_duty(chan, 0);
            // Timer::after_millis(0).await;
            // pwm_a.set_duty(chan, max / 15);
            // Timer::after_millis(10).await;
        }
    };


    // 38khz IR output
    let ir_output = async {
        // D6 / PB10 = TIM2CH3
        // D5 / PB4 = TIM3CH1
        // D3 / PB3 = TIM2CH2
        //
        // D7 / PA8 = TIM1CH1
        let mut pwm_b = embassy_stm32::timer::simple_pwm::SimplePwm::new(
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

        let max = pwm_b.get_max_duty();
        pwm_b.set_duty(embassy_stm32::timer::Channel::Ch1, max / 2);
        pwm_b.enable(embassy_stm32::timer::Channel::Ch1);

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

        set0(&mut pwm_b).await;
        loop {
            if SIGNAL_A.load(Ordering::SeqCst) > 0 {
                println!("IR output");
                SIGNAL_A.fetch_sub(1, Ordering::SeqCst);
                join(set1(&mut pwm_b), Timer::after_millis(8)).await;
                join(set0(&mut pwm_b), Timer::after_micros(4_500)).await;

                for _ in 0..3 {
                    //let data: u32 = 0b1110_0110_0000_1001_0110_0111_1001_1000; // power on command
                    //let data = !data;
                    let data: u32 = 0b0000_0000_0000_0000_1100_0000_0111_1000; // vol up
                    let range = if data < 0x0001_0000 { 16 } else { 32 };
                    for i in (0..range).rev() {
                        let bit = (data >> i) & 1;
                        if bit == 1 {
                            send1(&mut pwm_b).await;
                        } else {
                            send0(&mut pwm_b).await;
                        }
                    }

                    send0(&mut pwm_b).await;
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
                SIGNAL_A.store(3, Ordering::SeqCst);
            }

            while button.is_low() {
                Timer::after_millis(10).await;
            }

            println!("Button released!");
        }
    };

    let prints = async {
        let mut onboard_led = embassy_stm32::gpio::Output::new(ins.onboard_led_pin, Level::Low, Speed::Low);
        loop {
            //println!("new_ticks {}, {:?}", read.len(), read.get(0..20));
            let mut buf = [0; 2048];
            let mut i = 0;
            let mut start = Instant::now();
            let mut final_buf = loop {
                if i == 1024 {
                    // buffer full
                    break &mut buf[0..1024];
                }
                match select(ins.rx.receive(), Timer::after_millis(10)).await {
                    // We got a IR packet
                    Either::First(a) => {
                        if i == 1 {
                            start = Instant::now();
                        }
                        buf[i * 2] = a.0.as_micros() as u32;
                        buf[i * 2 + 1] = a.1.as_micros() as u32;
                        i += 1;
                    }
                    // Timeout
                    Either::Second(_) => {
                        if i > 0 {
                            break &mut buf[0..i];
                        } else {
                            i = 0;
                            Timer::after_millis(250).await;
                        }
                    }
                }
            };
            onboard_led.set_high();
            //let time = Instant::now().duration_since(start);
            println!("ticks: {}", final_buf.len());
            final_buf.sort_unstable();

            #[repr(u8)]
            #[derive(Hash, Copy, Clone)]
            enum Categories {
                LowPulse,
                ZeroPulse,
                OnePulse,
                InitialLow,
            }

            struct TimingCharistics {
                low: u32,
                zero: u32,
                one: u32,
                initial: u32,
            }

            let mut tot_low_pulse = 0;
            let mut count_low_pulse = 0;

            let mut tot_zero_pulse = 0;
            let mut count_zero_pulse = 0;

            let mut tot_one_pulse = 0;
            let mut count_one_pulse = 0;

            let mut tot_initial_low = 0;
            let mut count_initial_low = 0;

            let mut cur_cat = Categories::LowPulse;

            let mut last = 0;
            for &pulse_width in final_buf.iter() {
                if pulse_width < 400 {
                    continue;
                }

                if pulse_width > 15000 {
                    continue;
                }

                if last == 0 {
                    last = pulse_width;
                }
                match cur_cat {
                    Categories::LowPulse => {
                        if pulse_width - last > 200 {
                            cur_cat = Categories::ZeroPulse;

                        }
                    },
                    Categories::ZeroPulse => {
                        if pulse_width - last > 500 {
                            cur_cat = Categories::OnePulse;
                        }
                    },
                    Categories::OnePulse => {
                        if pulse_width - last > 2000 {
                            cur_cat = Categories::InitialLow;
                        }
                    },
                    Categories::InitialLow => {
                    }
                };

                let (map, map_cnt) = match cur_cat {
                    Categories::LowPulse => (&mut tot_low_pulse, &mut count_low_pulse),
                    Categories::ZeroPulse => (&mut tot_zero_pulse, &mut count_zero_pulse),
                    Categories::OnePulse => (&mut tot_one_pulse, &mut count_one_pulse),
                    Categories::InitialLow => (&mut tot_initial_low, &mut count_initial_low),
                };

                *map += pulse_width;
                *map_cnt += 1;

                last = pulse_width;
            }

            struct TimingCharistics {
                low: u32,
                zero: u32,
                one: u32,
                initial: u32,
            }

            let low = tot_low_pulse / count_low_pulse.max(1);
            let zero = tot_zero_pulse / count_zero_pulse.max(1);
            let one = tot_one_pulse / count_one_pulse.max(1);
            let initial = tot_initial_low / count_initial_low.max(1);

            println!("ticks: {} {} {} {}", low, zero, one, initial);
            Timer::after_millis(100).await;
            onboard_led.set_low();
        }
    };

    // Blinking LED example
    //println!("Starting blinking program");
    //_spawner.spawn(blink(p.PA5.degrade())).unwrap();

    //let bme = get_sensor_bme.await;
    //let imu = get_sensor_imu_i2c.await;
    //let imu_spi = get_sensor_imu_spi.await;

    //Timer::after_millis(100).await;

    join4(button, ir_output, prints, normal_out).await;
    //servo.await;
    //let ptr = shared_spi_bus.lock().await;
}
