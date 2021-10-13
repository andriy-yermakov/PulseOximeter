//#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]

use panic_semihosting as _;
use rtic::app;

pub mod display;
pub mod algorithm;
pub mod bbqueue;

#[app(device = stm32f0xx_hal::pac, peripherals = true)]
mod app {
    use compat_no_std::{self as std, result};
    use std::prelude::v1::*;
    

    use cortex_m::{self, register::control::read};
    // use cortex_m_semihosting::hprint;
    use display_interface_i2c::I2CInterface;
    use max3010x::{
        marker::{self, ic::Max30102, mode::Oximeter},
        AdcRange, FifoAlmostFullLevelInterrupt, Led, LedPulseWidth, Max3010x,
        SampleAveraging, SamplingRate,
    };
    use shared_bus_rtic::{CommonBus, SharedBus};
    use stm32f0xx_hal::{
        gpio::gpioa::{PA10, PA9},
        gpio::{Alternate, AF4},
        i2c::I2c,
        pac::{EXTI, I2C1},
        prelude::*,
        rcc::HSEBypassMode,
    };
    use num_traits::float::FloatCore;

    pub use crate::display::i2c_interface::I2CDisplayInterface;
    pub use crate::display::rotation::DisplayRotation;
    pub use crate::display::size::{DisplaySize, DisplaySize128x64};
    pub use crate::display::ssd1306::Ssd1306;
    
    use crate::bbqueue::{BBBuffer, Producer, Consumer};

    pub use crate::algorithm;

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
    
    pub enum State {
        Begin,
        Calibrate,
        CalculateHr,
        CollectNextPortion,
    }

    #[shared]
    struct Shared {
        shared_i2c_resources: SharedBusResources<BusType>,
        is_finger_on_sensor: bool,
        samples_collected: usize,
    }
    
    #[local]
    struct Local {
        exti: EXTI,
        ir_prod: Producer<'static, 800>,   
        ir_cons: Consumer<'static, 800>,
        red_prod: Producer<'static, 800>,   
        red_cons: Consumer<'static, 800>,

    }

    #[init(local = [ir_buffer: BBBuffer<800> = BBBuffer::new(), red_buffer: BBBuffer<800> = BBBuffer::new()])]
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

        let (ir_prod, ir_cons) = cx.local.ir_buffer.try_split().unwrap();
        let (red_prod, red_cons) = cx.local.red_buffer.try_split().unwrap();

        (
            Shared {
                shared_i2c_resources: SharedBusResources {
                    display,
                    sensor: SensorType::OximeterMode(sensor),
                },
                is_finger_on_sensor: false,
                samples_collected: 0,
            },
            Local {
                exti, 
                ir_prod,
                ir_cons,
                red_prod,
                red_cons,
            },
            init::Monotonics(),
        )
    }

    #[idle(shared = [shared_i2c_resources, is_finger_on_sensor, samples_collected], local = [ir_cons, red_cons])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut state = State::Begin;
        let mut saved_heart_rate: f32 = 0.0;
        let mut saved_spo2_value: f32 = 0.0;
        let mut hr_is_valid: bool = true;
        let mut spo2_is_valid: bool = true;



        loop {
            let is_finger_on_sensor = cx.shared.is_finger_on_sensor.lock(|f| *f);
            match state {
                State::Begin => {
                    saved_heart_rate = 0.0;
                    saved_spo2_value = 0.0;
                    
                    if is_finger_on_sensor {
                        // Reset buffers;
                        let result = cx.local.ir_cons.read();
                        match result {
                            Ok(rd) => {
                                let len = rd.buf.len();
                                rd.release(len);
                            },
                            _ => (),

                        }

                        let result = cx.local.red_cons.read();
                        match result {
                            Ok(rd) => {
                                let len = rd.buf.len();
                                rd.release(len);
                            },
                            _ => (),

                        }

                        cx.shared.samples_collected.lock(|sc| {*sc = 0;});
                        
                        // Turn LCDs on
                        cx.shared.shared_i2c_resources.lock(|r| {
                            match &mut r.sensor {
                                SensorType::OximeterMode(sensor) => {
                                    sensor.set_pulse_amplitude(Led::Led1, 0x24).unwrap(); // IR
                                    sensor.set_pulse_amplitude(Led::Led2, 0x24).unwrap(); // RED
                                }
                                _ => ()
                            }
                        });
                        state = State::Calibrate;
                    } else {
                        // Reset buffers
                        let result = cx.local.ir_cons.read();
                        match result {
                            Ok(rd) => {
                                let len = rd.buf.len();
                                rd.release(len);
                            },
                            _ => (),

                        }

                        let result = cx.local.red_cons.read();
                        match result {
                            Ok(rd) => {
                                let len = rd.buf.len();
                                rd.release(len);
                            },
                            _ => (),

                        }

                        cx.shared.samples_collected.lock(|sc| {*sc = 0;});
                    }
                }
                State::Calibrate => {
                    if is_finger_on_sensor {
                        if cx.shared.samples_collected.lock(|sc| *sc) > 150 {
                            state = State::CalculateHr;
                        }
                    } else {
                        // Turn LCDs off
                        cx.shared.shared_i2c_resources.lock(|r| {
                            match &mut r.sensor {
                                SensorType::OximeterMode(sensor) => {
                                    sensor.set_pulse_amplitude(Led::Led1, 1).unwrap(); // IR
                                    sensor.set_pulse_amplitude(Led::Led2, 0).unwrap(); // RED
                                }
                                _ => ()
                            }
                        });
                        state = State::Begin;
                    }
                }
                State::CalculateHr => {
                    if is_finger_on_sensor {
                        /*
                        ToDo: call spo2 and hert rate calculation function
                        read 50 samples for calculation                                                
                        */
                        cx.shared.samples_collected.lock(|sc| { *sc = 0; });
                
                        state = State::CollectNextPortion;
                    } else {
                        // Turn LCDs off
                        cx.shared.shared_i2c_resources.lock(|r| {
                            match &mut r.sensor {
                                SensorType::OximeterMode(sensor) => {
                                    sensor.set_pulse_amplitude(Led::Led1, 1).unwrap(); // IR
                                    sensor.set_pulse_amplitude(Led::Led2, 0).unwrap(); // RED
                                }
                                _ => ()
                            }
                        });
                        state = State::Begin;
                    }
                }
                State::CollectNextPortion => {
                    if is_finger_on_sensor {
                        if cx.shared.samples_collected.lock(|sc| *sc) > 50 {
                            state = State::CalculateHr;
                        }
                    } else {
                        cx.shared.shared_i2c_resources.lock(|r| {
                            match &mut r.sensor {
                                SensorType::OximeterMode(sensor) => {
                                    sensor.set_pulse_amplitude(Led::Led1, 1).unwrap(); // IR
                                    sensor.set_pulse_amplitude(Led::Led2, 0).unwrap(); // RED
                                }
                                _ => ()
                            }
                        });
                        state = State::Begin;
                    }
                }
            }

            let mut heart_rate = 0.0;
            let mut spo2_value = 0.0;

            // Update display
            if is_finger_on_sensor {
                cx.shared.shared_i2c_resources.lock(|r| r.display.display_on(true).unwrap());

                if hr_is_valid && (heart_rate != saved_heart_rate) && (heart_rate != 0.0) {
                    saved_heart_rate = heart_rate;
                    let heart_rate = heart_rate.floor() as u32;
                    cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(48, 0).write_char(' ').unwrap());
                    let d = (heart_rate % 1000) / 100;
                    
                    if 0 == d {
                        cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(64, 0).write_char(' ').unwrap());
                    } else {
                        let c = char(d);                        
                        cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(64, 0).write_char(c).unwrap());
                    }
                    
                    let c = char((heart_rate % 100) / 10);
                    cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(80, 0).write_char(c).unwrap());
                    let c = char(heart_rate % 10);
                    cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(96, 0).write_char(c).unwrap());
                }

                if spo2_is_valid && (spo2_value != saved_spo2_value) && (spo2_value != 0.0) {
                    saved_spo2_value = spo2_value;

                    let spo2_value = spo2_value.floor() as u32;
                    cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(48, 32).write_char(' ').unwrap());
                    cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(64, 32).write_char(' ').unwrap());
                    let c = char((spo2_value % 100) / 10);
                    cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(80, 32).write_char(c).unwrap());
                    let c = char(spo2_value % 10);
                    cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(96, 32).write_char(c).unwrap());
                }

            } else {
                cx.shared.shared_i2c_resources.lock(|r| r.display.display_on(false).unwrap());
                cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(48, 0).write_char(' ').unwrap());
                cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(64, 0).write_char('-').unwrap());
                cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(80, 0).write_char('-').unwrap());
                cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(96, 0).write_char(' ').unwrap());

                cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(48, 32).write_char(' ').unwrap());
                cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(64, 32).write_char('-').unwrap());
                cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(80, 32).write_char('-').unwrap());
                cx.shared.shared_i2c_resources.lock(|r| r.display.set_position(96, 32).write_char(' ').unwrap());
            }
        }
    }

    #[task(binds = EXTI0_1, 
        shared = [shared_i2c_resources, is_finger_on_sensor, samples_collected], 
        local = [ exti, ir_prod, red_prod ])]
    fn handle_sensor(mut cx: handle_sensor::Context) {
        let mut samples = [0; 2];

        // Read a data from a sensor
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

        // Check FingerOnSensor status
        cx.shared
            .is_finger_on_sensor
            .lock(|is_finger_on_sensor| match *is_finger_on_sensor {
                true => {
                    if samples[0] < 50000 {
                        *is_finger_on_sensor = false
                    }
                }
                false => {
                    if samples[0] > 1600 {
                        *is_finger_on_sensor = true
                    }
                }
            });

        // Clear Interrupt Pending Bit
        cx.local.exti.pr.write(|w| w.pr1().set_bit());


        if cx.shared.is_finger_on_sensor.lock(|f| *f) == true {
            // Store the data in the buffers
            let result = cx.local.ir_prod.grant_exact(4);
            match result {
                Ok(mut wr) => {
                    let bytes = samples[0].to_le_bytes();
                    for i in 0..4 {
                        wr[i] = bytes[i];
                    }
                    wr.commit(4);
                },
                _ => (),
            }
            
            let result = cx.local.red_prod.grant_exact(4);
            match result {
                Ok(mut wr) => {
                    let bytes = samples[1].to_le_bytes();
                    for i in 0..4 {
                        wr[i] = bytes[i];
                    }
                    wr.commit(4);
                },
                _ => (),
            }

            cx.shared.samples_collected.lock(|sc| { *sc += 1; });
        };

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

    fn char(b: u32) -> char {
        match b {
            0 => '0',
            1 => '1',
            2 => '2',
            3 => '3',
            4 => '4',
            5 => '5',
            6 => '6',
            7 => '7',
            8 => '8',
            9 => '9',
            _ => '*',
        }
    }
}
