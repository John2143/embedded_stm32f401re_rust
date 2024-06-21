#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

pub mod pac {
    pub use cortex_m_rt::interrupt;
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
    pub use embassy_stm32::NVIC_PRIO_BITS;
}

#[rtic::app(device = crate::pac, peripherals = false, dispatchers = [])]
mod app {
    use defmt::println;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init()]
    fn init(cx: init::Context) -> (Shared, Local) {
        println!("Hello world");
        let p = embassy_stm32::init(Default::default());

        (Shared {}, Local {})
    }

    #[task()]
    async fn foo(cx: foo::Context) {
    }
}

