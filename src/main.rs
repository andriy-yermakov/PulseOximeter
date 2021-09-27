//#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]

use panic_semihosting as _;
use rtic::app;

pub mod display;

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {
    use core::cell::UnsafeCell;

    use cortex_m;
    use display_interface_i2c::I2CInterface;
    use max3010x::{marker, Led, Max3010x, SampleAveraging};
    use shared_bus_rtic::SharedBus;
    use stm32f1xx_hal::{
        device::I2C1,
        gpio,
        gpio::{Alternate, OpenDrain, Pin, CRH},
        i2c::{BlockingI2c, DutyCycle, Mode},
        pac::TIM2,
        prelude::*,
        timer::{CountDownTimer, Event, Timer},
    };

    pub use crate::display::i2c_interface::I2CDisplayInterface;
    pub use crate::display::rotation::DisplayRotation;
    pub use crate::display::size::{DisplaySize, DisplaySize128x64};
    pub use crate::display::ssd1306::Ssd1306;

    type LEDPIN = gpio::gpioc::PC13<gpio::Output<gpio::OpenDrain>>;

    type BusType = stm32f1xx_hal::i2c::blocking::BlockingI2c<
        I2C1,
        (
            Pin<Alternate<OpenDrain>, CRH, 'B', 8>,
            Pin<Alternate<OpenDrain>, CRH, 'B', 9>,
        ),
    >;

    pub struct SharedBusResources<T: 'static> {
        display: Ssd1306<I2CInterface<SharedBus<T>>, DisplaySize128x64>,
        //sensor: SensorType<T>,
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
    struct Local {
        led: LEDPIN,
        timer: CountDownTimer<TIM2>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let _device: stm32f1xx_hal::pac::Peripherals = cx.device;

        // Obtain Clocks
        let mut flash = _device.FLASH.constrain();
        let rcc = _device.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.mhz()).freeze(&mut flash.acr);
        let mut afio = _device.AFIO.constrain();

        // Init timer
        let mut timer = Timer::tim2(_device.TIM2, &clocks).start_count_down(2.hz());
        timer.listen(Event::Update);

        // Init led pin
        let mut gpioc = _device.GPIOC.split();
        let mut led = gpioc.pc13.into_open_drain_output(&mut gpioc.crh);
        let _ = led.set_high(); // Turn led off

        // Init I2C
        let mut gpiob = _device.GPIOB.split();
        let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);
        let i2c = BlockingI2c::i2c1(
            _device.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400_000.hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            1000,
            10,
            1000,
            1000,
        );

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

        let sensor = Max3010x::new_max30102(manager.acquire());
        // let mut sensor = sensor.into_heart_rate().unwrap();

        // sensor.set_sample_averaging(SampleAveraging::Sa4).unwrap();
        // sensor.set_pulse_amplitude(Led::All, 15).unwrap();
        // sensor.enable_fifo_rollover().unwrap();
        // let mut data = [0; 3];
        // let samples_read = sensor.read_fifo(&mut data).unwrap();

        (
            Shared {
                shared_i2c_resources: SharedBusResources {
                    display,
                    //sensor: SensorType::HeartRateMode(sensor),
                },
            },
            Local { led, timer },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = TIM2, local = [led, timer], shared = [shared_i2c_resources], priority=1)]
    fn blink(mut cx: blink::Context) {
        let _ = cx.local.led.toggle();
        let _ = cx.local.timer.wait();
        cx.shared
            .shared_i2c_resources
            .lock(|shared_i2c_resources| shared_i2c_resources.display.clear().unwrap());
    }
}
