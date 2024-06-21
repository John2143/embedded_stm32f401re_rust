#![no_std]
#![no_main]

use embassy_stm32::{
    peripherals::{
        DMA1_CH5, DMA1_CH7, DMA2_CH0, DMA2_CH3, EXTI9, I2C1, PA6, PA7, PA8, PA9, PB3, PB6,
        PB8, PB9, PC13, SPI1, TIM1,
    }, PeripheralRef,
};

use defmt_rtt as _;
use panic_probe as _;

pub mod pac {
    pub use cortex_m_rt::interrupt;
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
    pub use embassy_stm32::NVIC_PRIO_BITS;
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

    button: PeripheralRef<'static, PC13>,

    //rx: Receiver<'static, MUTEX, IrType, IR_COUNT>,
}

struct InputIRLoop {
    pin: PeripheralRef<'static, PA9>,
    exti: PeripheralRef<'static, EXTI9>,
    //tx: Sender<'static, MUTEX, IrType, IR_COUNT>,
}

#[rtic::app(device = crate::pac, peripherals = false, dispatchers = [EXTI2, EXTI3])]
mod app {
    use defmt::println;
    use super::{InputMainLoop, InputIRLoop};

    use embassy_stm32::Peripheral;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        input_main_loop: InputMainLoop,
        input_ir_loop: InputIRLoop,
    }

    #[init()]
    fn init(cx: init::Context) -> (Shared, Local) {
        println!("Hello world");
        let p = embassy_stm32::init(Default::default());

        let input_ir_loop = InputIRLoop {
            pin: p.PA9.into_ref(),
            exti: p.EXTI9.into_ref(),
        };

        let input_main_loop = InputMainLoop {
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

        };

        (Shared {}, Local {
            input_main_loop,
            input_ir_loop,
        })
    }

    #[task(local = [input_main_loop], priority = 5)]
    async fn main_loop(cx: main_loop::Context) {
    }

    #[task(local = [input_ir_loop], priority = 2)]
    async fn ir_loop(cx: ir_loop::Context) {
    }
}

