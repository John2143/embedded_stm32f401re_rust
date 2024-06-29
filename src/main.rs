#![no_std]
#![no_main]

use defmt::debug;
use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    bind_interrupts,
    interrupt::{self, InterruptExt},
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;

use defmt_rtt as _;
use panic_probe as _;

type MUTEX = CriticalSectionRawMutex;

//use core::sync::atomic::AtomicU32;
//static SIGNAL_A: AtomicU32 = AtomicU32::new(0);
//static SIGNAL_B: AtomicU32 = AtomicU32::new(900);
//static SIGNAL_C: AtomicU32 = AtomicU32::new(900);

#[embassy_stm32::interrupt]
fn I2C3_ER() {
    unsafe { EXECUTOR_HIGH.on_interrupt() };
}

#[embassy_stm32::interrupt]
fn I2C3_EV() {
    unsafe { EXECUTOR_LOW.on_interrupt() };
}

// Bind the ebassy interrupt handlers
bind_interrupts!(
    struct Irqs {
        //USB => usb::InterruptHandler<peripherals::USB>;
        //USART6 => usart::InterruptHandler<peripherals::USART6>;
        //I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
        //I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;
    }
);

// DMA/hardware interrupts > HIGH > NORMAL > LOW
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: InterruptExecutor = InterruptExecutor::new();

//static IR_CHANNEL_RECEIVE: StaticCell<IrChannelReceive> = StaticCell::new();
//static IR_CHANNEL_TRANSMIT: StaticCell<IrChannelTransmit> = StaticCell::new();

#[global_allocator]
static HEAP: embedded_alloc::Heap = embedded_alloc::Heap::empty();

#[cortex_m_rt::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let p = embassy_stm32::init(Default::default());

    //let ir_recv_channel = IR_CHANNEL_RECEIVE.init(IrChannelReceive::new());
    //let ir_recv_rx = ir_recv_channel.receiver();
    //let ir_recv_tx = ir_recv_channel.sender();

    let high_input = InputHighLoop {};

    let low_input = InputMainLoop {};

    // We use I2C flags for our event loops
    // This could be any set of interrupts that are not used by the peripherals
    drop(p.I2C3);

    // Setup our two executors
    {
        let chan = embassy_stm32::pac::interrupt::I2C3_ER;
        chan.set_priority(embassy_stm32::interrupt::Priority::P2);
        let spawner = EXECUTOR_HIGH.start(chan);
        spawner.spawn(high_prio_loop(high_input)).unwrap();

        let chan = embassy_stm32::pac::interrupt::I2C3_EV;
        chan.set_priority(embassy_stm32::interrupt::Priority::P8);
        let spawner = EXECUTOR_LOW.start(chan);
        spawner.spawn(low_prio_loop(low_input)).unwrap();
    }

    // Wait for interrupts
    loop {
        cortex_m::asm::wfi();
    }
}

struct InputHighLoop {}

#[embassy_executor::task]
async fn high_prio_loop(ins: InputHighLoop) {
    debug!("Running IR Transmitter program");

    loop {
        Timer::after_millis(5000).await;
    }
}

struct InputMainLoop {
    //cs_spi1: PeripheralRef<'static, PB6>,
    //cs_spi2: PeripheralRef<'static, PC4>,

    //i2c_channel: PeripheralRef<'static, I2C1>,
    //i2c_sda: PeripheralRef<'static, PB9>,
    //i2c_scl: PeripheralRef<'static, PB8>,
    //i2c_dma_tx: PeripheralRef<'static, DMA1_CH7>,
    //i2c_dma_rx: PeripheralRef<'static, DMA1_CH5>,
    //
    //spi_sck: PeripheralRef<'static, PB3>,
    //spi_mosi: PeripheralRef<'static, PA7>,
    //spi_miso: PeripheralRef<'static, PA6>,
    //spi_dma_tx: PeripheralRef<'static, DMA2_CH3>,
    //spi_dma_rx: PeripheralRef<'static, DMA2_CH0>,

    //uart: USart,

    //onboard_led_pin: PeripheralRef<'static, PA5>,

    //other_output_timer: PeripheralRef<'static, TIM3>,
    //other_led_pin: PeripheralRef<'static, PB4>,

    //button: PeripheralRef<'static, PC13>,

    //rx: Receiver<'static, MUTEX, IrReceiveType, IR_RECEIVE_COUNT>,
    //ir_transmit_tx: Sender<'static, MUTEX, IrTransmitType, IR_TRANSMIT_COUNT>,
}

struct USart {
    //uart: PeripheralRef<'static, USART6>,
    //rx: PeripheralRef<'static, PA12>,
    //tx: PeripheralRef<'static, PA11>,
    //dma_tx: PeripheralRef<'static, DMA2_CH6>,
    //dma_rx: PeripheralRef<'static, DMA2_CH1>,
}

#[embassy_executor::task]
async fn low_prio_loop(ins: InputMainLoop) {
    debug!("Running main Program");
}
