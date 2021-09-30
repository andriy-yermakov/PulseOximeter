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
    use cortex_m_semihosting::hprint;
    use display_interface_i2c::I2CInterface;
    use max3010x::{
        marker::{self, ic::Max30102, mode::Oximeter},
        AdcRange, FifoAlmostFullLevelInterrupt, InterruptStatus, Led, LedPulseWidth, Max3010x,
        SampleAveraging, SamplingRate,
    };
    use shared_bus_rtic::{CommonBus, SharedBus};
    use stm32f0xx_hal::{
        gpio::gpioa::{PA10, PA9},
        gpio::{Alternate, AF4},
        i2c::I2c,
        pac::{Interrupt, Peripherals, EXTI, I2C1},
        prelude::*,
        rcc::HSEBypassMode,
    };

    use crate::display;
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
        is_finger_on_sensor: bool,
    }

    #[local]
    struct Local {
        exti: EXTI,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let _device: stm32f0xx_hal::pac::Peripherals = cx.device;

        // Obtain Clocks
        let rcc = _device.RCC;
        rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());
        rcc.ahbenr.modify(|_, w| w.iopben().set_bit());

        let mut flash = _device.FLASH;
        let mut rcc = rcc
            .configure()
            .hse(8.mhz(), HSEBypassMode::NotBypassed)
            .sysclk(48.mhz())
            .freeze(&mut flash);

        // Configure EXTI IRQ
        let gpiob = _device.GPIOB.split(&mut rcc);
        let syscfg = _device.SYSCFG;
        let exti = _device.EXTI;

        cortex_m::interrupt::free(move |cs| {
            let _ = gpiob.pb1.into_floating_input(cs);
        });

        // Enable external interrupt for PB1
        syscfg.exticr1.modify(|_, w| unsafe { w.exti1().bits(1) });
        // Set interrupt request mask for line 1
        exti.imr.modify(|_, w| w.mr1().set_bit());
        // Set interrupt falling trigger for line 1
        exti.ftsr.modify(|_, w| w.tr1().set_bit());

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
        let mut sensor = Max3010x::new_max30102(manager.acquire());
        sensor.reset().unwrap();
        let sensor = sensor.into_oximeter().unwrap();
        let sensor = init_oximeter(sensor);

        let is_finger_on_sensor = false;

        (
            Shared {
                shared_i2c_resources: SharedBusResources {
                    display,
                    sensor: SensorType::OximeterMode(sensor),
                },
                is_finger_on_sensor,
            },
            Local { exti },
            init::Monotonics(),
        )
    }

    #[idle(shared = [shared_i2c_resources, is_finger_on_sensor])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            let is_finger_on_sensor = cx.shared.is_finger_on_sensor.lock(|f| *f);

            if is_finger_on_sensor {
                cx.shared.shared_i2c_resources.lock(|r| {
                    r.display.display_on(true).unwrap();
                });
            } else {
                cx.shared.shared_i2c_resources.lock(|r| {
                    r.display.display_on(false).unwrap();
                });
            }
        }
    }

    #[task(binds = EXTI0_1, shared = [shared_i2c_resources, is_finger_on_sensor], local = [exti], priority=1)]
    fn handle_sensor(mut cx: handle_sensor::Context) {
        let mut samples = [0; 2];

        cx.shared
            .shared_i2c_resources
            .lock(|r| match &mut r.sensor {
                SensorType::OximeterMode(sensor) => {
                    if sensor.read_interrupt_status().unwrap().new_fifo_data_ready {
                        sensor.read_fifo(&mut samples).unwrap();
                    }
                }
                _ => (),
            });

        cx.shared
            .is_finger_on_sensor
            .lock(|is_finger_in_sensor| match *is_finger_in_sensor {
                true => {
                    // if samples[0] < 50000 {
                    if samples[0] < 1600 {
                        *is_finger_in_sensor = false
                    }
                }
                false => {
                    if samples[0] > 1600 {
                        *is_finger_in_sensor = true
                    }
                }
            });

        // Clear Interrupt Pending Bit
        cx.local.exti.pr.write(|w| w.pr1().set_bit());
    }

    fn init_oximeter(
        mut sensor: Max3010x<
            &CommonBus<I2c<I2C1, PA9<Alternate<AF4>>, PA10<Alternate<AF4>>>>,
            Max30102,
            Oximeter,
        >,
    ) -> Max3010x<
        &CommonBus<I2c<I2C1, PA9<Alternate<AF4>>, PA10<Alternate<AF4>>>>,
        Max30102,
        Oximeter,
    > {
        sensor.read_interrupt_status().unwrap();
        sensor.clear_fifo().unwrap();
        sensor.disable_fifo_rollover().unwrap();
        sensor.set_sample_averaging(SampleAveraging::Sa1).unwrap();
        sensor
            .set_fifo_almost_full_level_interrupt(FifoAlmostFullLevelInterrupt::L7)
            .unwrap();

        sensor.set_adc_range(AdcRange::Fs4k).unwrap();
        sensor.set_sampling_rate(SamplingRate::Sps50).unwrap();
        sensor.set_pulse_width(LedPulseWidth::Pw411).unwrap();
        sensor.set_pulse_amplitude(Led::Led1, 1).unwrap(); // IR
        sensor.set_pulse_amplitude(Led::Led2, 0).unwrap(); // RED
        sensor.disable_fifo_almost_full_interrupt().unwrap();
        sensor.enable_new_fifo_data_ready_interrupt().unwrap();
        sensor
    }
}
