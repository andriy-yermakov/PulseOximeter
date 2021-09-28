//#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]

use panic_semihosting as _;
use rtic::app;

pub mod display;

#[app(device = stm32f0xx_hal::pac, peripherals = true)]
mod app {
    use cortex_m;
    use display_interface_i2c::I2CInterface;
    use max3010x::{marker, Led, Max3010x, SampleAveraging};
    use shared_bus_rtic::SharedBus;
    use stm32f0xx_hal::{
        gpio::gpioa::{PA10, PA9},
        gpio::{Alternate, AF4},
        i2c::I2c,
        pac::I2C1,
        prelude::*,
        rcc::HSEBypassMode,
    };

    pub use crate::display::i2c_interface::I2CDisplayInterface;
    pub use crate::display::rotation::DisplayRotation;
    pub use crate::display::size::{DisplaySize, DisplaySize128x64};
    pub use crate::display::ssd1306::Ssd1306;

    type BusType = I2c<I2C1, PA9<Alternate<AF4>>, PA10<Alternate<AF4>>>;

    pub struct SharedBusResources<T: 'static> {
        display: Ssd1306<I2CInterface<SharedBus<T>>, DisplaySize128x64>,
        sensor: SensorType<T>,
    }

    pub enum SensorType<T: 'static> {
        NoneMode(Max3010x<SharedBus<T>, marker::ic::Max30102, marker::mode::None>),
        HeartRateMode(Max3010x<SharedBus<T>, marker::ic::Max30102, marker::mode::HeartRate>),
        OximeterMode(Max3010x<SharedBus<T>, marker::ic::Max30102, marker::mode::Oximeter>),
        MultiLED(Max3010x<SharedBus<T>, marker::ic::Max30102, marker::mode::MultiLED>),
    }

    #[shared]
    struct Shared {
        shared_i2c_resources: SharedBusResources<BusType>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let _device: stm32f0xx_hal::pac::Peripherals = cx.device;

        // Obtain Clocks
        let mut flash = _device.FLASH;
        let mut rcc = _device
            .RCC
            .configure()
            .hse(8.mhz(), HSEBypassMode::NotBypassed)
            .sysclk(48.mhz())
            .freeze(&mut flash);

        // Init timer
        //let mut timer = Timer::tim14(_device.TIM2, &clocks).start_count_down(2.hz());
        //timer.listen(Event::Update);

        // Init I2C
        let gpioa = _device.GPIOA.split(&mut rcc);

        let (scl, sda) = cortex_m::interrupt::free(move |cs| {
            (
                // SI2C  pins
                gpioa.pa9.into_alternate_af4(cs),
                gpioa.pa10.into_alternate_af4(cs),
            )
        });

        let i2c = I2c::i2c1(_device.I2C1, (scl, sda), 400.khz(), &mut rcc);
        let manager = shared_bus_rtic::new!(i2c, BusType);

        // Init display
        let i2c_display = I2CDisplayInterface::new(manager.acquire());
        let mut display = Ssd1306::new(i2c_display, DisplaySize128x64, DisplayRotation::Rotate0);

        display.init().unwrap();
        display.clear().unwrap();

        // Print initial screen
        display.set_position(0, 0).write_char('H').unwrap();
        display.set_position(16, 0).write_char('R').unwrap();
        display.set_position(32, 0).write_char(':').unwrap();

        display.set_position(0, 32).write_char('O').unwrap();
        display.set_position(16, 32).write_char('2').unwrap();
        display.set_position(32, 32).write_char(':').unwrap();

        // Init sensor
        let sensor = Max3010x::new_max30102(manager.acquire());
        let mut sensor = sensor.into_oximeter().unwrap();

        sensor.set_sample_averaging(SampleAveraging::Sa4).unwrap();
        sensor.set_pulse_amplitude(Led::Led1, 1).unwrap();
        sensor.set_pulse_amplitude(Led::Led2, 0).unwrap();
        sensor.enable_fifo_rollover().unwrap();

        (
            Shared {
                shared_i2c_resources: SharedBusResources {
                    display,
                    sensor: SensorType::OximeterMode(sensor),
                },
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[idle(shared = [shared_i2c_resources])]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
            // cx.shared
            //     .shared_i2c_resources
            //     .lock(|r| r.display.clear().unwrap());
        }
    }

    // #[task(binds = TIM14, local = [led, timer], shared = [shared_i2c_resources], priority=1)]
    // fn blink(mut cx: blink::Context) {
    //     //let _ = cx.local.led.toggle();
    //     //let _ = cx.local.timer.wait();
    //     cx.shared
    //         .shared_i2c_resources
    //         .lock(|shared_i2c_resources| shared_i2c_resources.display.clear().unwrap());
    // }
}
