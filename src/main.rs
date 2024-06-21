#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [])]
mod app {
    use defmt::println;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init()]
    fn init(cx: init::Context) -> (Shared, Local) {
        println!("Hello world");
        (Shared {}, Local {})
    }

    #[task()]
    async fn foo(cx: foo::Context) {
    }
}

