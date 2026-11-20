#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use barefoc::common::filter::LowPassFilter;
use barefoc::common::time::Clock;
use barefoc::kinematics::{ActuatorAssembly, MotorControl, TopologyPreset};
use barefoc::motor::enums::{MotionControlType, TorqueControlType};
use barefoc::traits::adc::AdcPin;
use barefoc::traits::sensor::{Direction, Sensor, SensorBase};
use barefoc_commander::kinematics::{Commander4Axis, KinematicsCommander};
use barefoc_drivers::comms::registers::{RegisterError, RegisterId, RegisterValue};
use barefoc_drivers::comms::RegisterBridge;
use barefoc_drivers::dc::traits::{DCDriver, DCDriverParams};
use barefoc_drivers::dc::{CurrentMirrorSense, DCDriver2PWM, DCMotor, DummySensor};
use barefoc::control_table::{
    error_flags, ControlTable,
    REG_PRESENT_BUS_VOLTAGE, REG_PRESENT_CURRENT,
    REG_PRESENT_POSITION, REG_PRESENT_TEMPERATURE, REG_PRESENT_VELOCITY,
    REG_REBOOT_CMD, REG_SAVE_CONFIG_CMD, CMD_SAVE_CONFIG, CMD_REBOOT_BOOTLOADER,
};
use barefoc::traits::storage::ConfigStorage;
use barefoc_stm32::stm32h5::H5FlashStorage;
use barefoc_multibus::profile::{
    multi_motor_profile, refresh_telemetry_multi, MultiMotorState,
};
use barefoc_multibus::table::{HookResult, RegEntry};
use barefoc_multibus::transport::RegServer;
use barefoc_stm32::bus_target_i2c::{ServiceOutcome, Stm32I2cTarget};

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{ADC1, I2C2, PA2, PA3, PA4, PA5, PB1, TIM1, TIM3};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use panic_rtt_target as _;
use rtt_target::{rprintln, DownChannel};
use tmag_rs::barefoc::Tmag5273Sensor;
use tmag_rs::{AngleEnable, DeviceVariant, MagneticChannel, OperatingMode, Tmag5273};

static TMAG_DETECTED: AtomicBool = AtomicBool::new(false);

static GLOBAL_STORAGE: Mutex<
    CriticalSectionRawMutex,
    RefCell<Option<H5FlashStorage<embassy_stm32::flash::Flash<'static, Blocking>>>>,
> = Mutex::new(RefCell::new(None));

static SHARED_CONTROL_TABLE: Mutex<CriticalSectionRawMutex, RefCell<ControlTable>> =
    Mutex::new(RefCell::new(ControlTable::empty()));

/// Custom register offset for runtime topology switching over MultiBus.
pub const REG_TOPOLOGY_MODE: u8 = 0x78;


// ---------------------------------------------------------------------------
// System Clock implementing barefoc::common::time::Clock via Embassy Instant
// ---------------------------------------------------------------------------
#[derive(Clone, Copy)]
pub struct EmbassyClock;

impl Clock for EmbassyClock {
    fn now_us(&self) -> u32 {
        Instant::now().as_micros() as u32
    }

    fn delay_ms(&self, ms: u32) {
        embassy_time::block_for(Duration::from_millis(ms as u64));
    }
}

static CLOCK: EmbassyClock = EmbassyClock;

// ---------------------------------------------------------------------------
// ADC1 Channel Multiplexer & AdcPin Adapter
// ---------------------------------------------------------------------------
static GLOBAL_ADC: Mutex<CriticalSectionRawMutex, RefCell<Option<Adc<'static, ADC1, Blocking>>>> =
    Mutex::new(RefCell::new(None));

pub enum AdcInputPin {
    ChA(Peri<'static, PA2>),
    ChB(Peri<'static, PA3>),
    ChC(Peri<'static, PA4>),
    ChD(Peri<'static, PA5>),
    Vsens(Peri<'static, PB1>),
}

pub struct Stm32AdcPin {
    pin: AdcInputPin,
    sample_time: SampleTime,
}

impl Stm32AdcPin {
    pub fn new(pin: AdcInputPin, sample_time: SampleTime) -> Self {
        Self { pin, sample_time }
    }
}

impl AdcPin for Stm32AdcPin {
    fn read(&mut self) -> u16 {
        let mut sum: u32 = 0;
        const SAMPLES: u32 = 8;
        GLOBAL_ADC.lock(|cell| {
            if let Some(ref mut adc) = *cell.borrow_mut() {
                for _ in 0..SAMPLES {
                    let val = match &mut self.pin {
                        AdcInputPin::ChA(pin) => adc.blocking_read(pin, self.sample_time),
                        AdcInputPin::ChB(pin) => adc.blocking_read(pin, self.sample_time),
                        AdcInputPin::ChC(pin) => adc.blocking_read(pin, self.sample_time),
                        AdcInputPin::ChD(pin) => adc.blocking_read(pin, self.sample_time),
                        AdcInputPin::Vsens(pin) => adc.blocking_read(pin, self.sample_time),
                    };
                    sum += val as u32;
                }
            }
        });
        (sum / SAMPLES) as u16
    }
}

/// Read internal MCU core temperature sensor in degrees Celsius.
pub fn read_internal_temperature() -> f32 {
    let mut sum: u32 = 0;
    const SAMPLES: u32 = 4;
    GLOBAL_ADC.lock(|cell| {
        if let Some(ref mut adc) = *cell.borrow_mut() {
            let mut temp_ch = adc.enable_temperature();
            for _ in 0..SAMPLES {
                sum += adc.blocking_read(&mut temp_ch, SampleTime::Cycles65) as u32;
            }
        }
    });
    let avg = (sum / SAMPLES) as f32;
    // Conversion for STM32H5 internal temp sensor: ~0.70V at 25C, 2.5mV/C
    let v_sense = (avg * 3.3) / 4095.0;
    ((v_sense - 0.70) / 0.0025) + 25.0
}


// ---------------------------------------------------------------------------
// Bridge A Sign-Magnitude H-Bridge Driver (PA15 DIR, PB5 PWM on TIM1_CH3)
// ---------------------------------------------------------------------------
pub struct BridgeADriver {
    params: DCDriverParams,
    dir_pin: Output<'static>,
    pwm_ch: SimplePwmChannel<'static, TIM1>,
}

impl BridgeADriver {
    pub fn new(dir_pin: Output<'static>, pwm_ch: SimplePwmChannel<'static, TIM1>) -> Self {
        Self {
            params: DCDriverParams::new(),
            dir_pin,
            pwm_ch,
        }
    }
}

impl DCDriver for BridgeADriver {
    fn init(&mut self) -> Result<(), ()> {
        self.params.initialized = true;
        if self.params.voltage_power_supply > 0.0 && self.params.voltage_limit > self.params.voltage_power_supply {
            self.params.voltage_limit = self.params.voltage_power_supply;
        }
        self.dir_pin.set_low();
        self.pwm_ch.set_duty_cycle(0);
        Ok(())
    }

    fn enable(&mut self) {}

    fn disable(&mut self) {
        self.set_pwm(0.0);
    }

    fn set_pwm(&mut self, voltage: f32) {
        let v_limit = self.params.voltage_limit;
        let v_supply = if self.params.voltage_power_supply > 0.0 {
            self.params.voltage_power_supply
        } else {
            1.0
        };
        let max_duty = self.pwm_ch.max_duty_cycle() as f32;

        if voltage > 0.0 {
            let u = if voltage > v_limit { v_limit } else { voltage };
            let duty_fraction = (u / v_supply).clamp(0.0, 1.0);
            self.dir_pin.set_high();
            let duty = ((1.0 - duty_fraction) * max_duty) as u32;
            self.pwm_ch.set_duty_cycle(duty);
        } else if voltage < 0.0 {
            let abs_v = -voltage;
            let u = if abs_v > v_limit { v_limit } else { abs_v };
            let duty_fraction = (u / v_supply).clamp(0.0, 1.0);
            self.dir_pin.set_low();
            let duty = (duty_fraction * max_duty) as u32;
            self.pwm_ch.set_duty_cycle(duty);
        } else {
            self.dir_pin.set_low();
            self.pwm_ch.set_duty_cycle(0);
        }
    }

    fn params(&self) -> &DCDriverParams {
        &self.params
    }

    fn params_mut(&mut self) -> &mut DCDriverParams {
        &mut self.params
    }
}

// ---------------------------------------------------------------------------
// Quad H-Bridge Unified DCDriver Enum
// ---------------------------------------------------------------------------
pub enum QuadDCDriver {
    A(BridgeADriver),
    B(DCDriver2PWM<SimplePwmChannel<'static, TIM1>, SimplePwmChannel<'static, TIM3>>),
    C(DCDriver2PWM<SimplePwmChannel<'static, TIM3>, SimplePwmChannel<'static, TIM3>>),
    D(DCDriver2PWM<SimplePwmChannel<'static, TIM1>, SimplePwmChannel<'static, TIM3>>),
}

impl DCDriver for QuadDCDriver {
    fn init(&mut self) -> Result<(), ()> {
        match self {
            Self::A(d) => d.init(),
            Self::B(d) => d.init(),
            Self::C(d) => d.init(),
            Self::D(d) => d.init(),
        }
    }

    fn enable(&mut self) {
        match self {
            Self::A(d) => d.enable(),
            Self::B(d) => d.enable(),
            Self::C(d) => d.enable(),
            Self::D(d) => d.enable(),
        }
    }

    fn disable(&mut self) {
        match self {
            Self::A(d) => d.disable(),
            Self::B(d) => d.disable(),
            Self::C(d) => d.disable(),
            Self::D(d) => d.disable(),
        }
    }

    fn set_pwm(&mut self, voltage: f32) {
        match self {
            Self::A(d) => d.set_pwm(voltage),
            Self::B(d) => d.set_pwm(voltage),
            Self::C(d) => d.set_pwm(voltage),
            Self::D(d) => d.set_pwm(voltage),
        }
    }

    fn params(&self) -> &DCDriverParams {
        match self {
            Self::A(d) => d.params(),
            Self::B(d) => d.params(),
            Self::C(d) => d.params(),
            Self::D(d) => d.params(),
        }
    }

    fn params_mut(&mut self) -> &mut DCDriverParams {
        match self {
            Self::A(d) => d.params_mut(),
            Self::B(d) => d.params_mut(),
            Self::C(d) => d.params_mut(),
            Self::D(d) => d.params_mut(),
        }
    }
}

// ---------------------------------------------------------------------------
// Shared Motor State & MultiBus Register Bridge
// ---------------------------------------------------------------------------
#[derive(Clone, Copy)]
pub struct SharedServoState {
    pub target: f32,
    pub current_angle: f32,
    pub current_velocity: f32,
    pub current_ma: f32,
    pub p_angle: f32,
    pub pid_vel_p: f32,
    pub pid_vel_i: f32,
    pub pid_vel_d: f32,
    pub voltage_limit: f32,
    pub velocity_limit: f32,
    pub enabled: bool,
    pub status: u8,
    pub controller_mode: u8, // 0 = angle, 1 = velocity_openloop, 2 = voltage
    pub homing_requested: bool,
    pub homing_voltage: f32,
    pub stall_threshold_ma: f32,
}

impl SharedServoState {
    pub const fn new() -> Self {
        Self {
            target: 0.0,
            current_angle: 0.0,
            current_velocity: 0.0,
            current_ma: 0.0,
            p_angle: 20.0,
            pid_vel_p: 0.2,
            pid_vel_i: 2.0,
            pid_vel_d: 0.001,
            voltage_limit: 5.0,
            velocity_limit: 50.0,
            enabled: true,
            status: 1, // Ready
            controller_mode: 0,
            homing_requested: false,
            homing_voltage: 4.0,
            stall_threshold_ma: 100.0,
        }
    }
}

impl RegisterBridge for SharedServoState {
    fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, RegisterError> {
        match id {
            RegisterId::Status => Ok(RegisterValue::U8(self.status)),
            RegisterId::Target => Ok(RegisterValue::Float(self.target)),
            RegisterId::Enable => Ok(RegisterValue::U8(if self.enabled { 1 } else { 0 })),
            RegisterId::ControlMode => Ok(RegisterValue::U8(MotionControlType::angle as u8)),
            RegisterId::Angle | RegisterId::SensorAngle => Ok(RegisterValue::Float(self.current_angle)),
            RegisterId::Velocity | RegisterId::SensorVelocity => {
                Ok(RegisterValue::Float(self.current_velocity))
            }
            RegisterId::CurrentQ | RegisterId::CurrentDC | RegisterId::CurrentA => {
                Ok(RegisterValue::Float(self.current_ma))
            }
            RegisterId::AngPidP => Ok(RegisterValue::Float(self.p_angle)),
            RegisterId::VelPidP => Ok(RegisterValue::Float(self.pid_vel_p)),
            RegisterId::VelPidI => Ok(RegisterValue::Float(self.pid_vel_i)),
            RegisterId::VelPidD => Ok(RegisterValue::Float(self.pid_vel_d)),
            RegisterId::VoltageLimit => Ok(RegisterValue::Float(self.voltage_limit)),
            RegisterId::VelocityLimit => Ok(RegisterValue::Float(self.velocity_limit)),
            _ => Err(RegisterError::InvalidPayload),
        }
    }

    fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), RegisterError> {
        match (id, value) {
            (RegisterId::Target, RegisterValue::Float(v)) => {
                self.target = v;
                Ok(())
            }
            (RegisterId::Enable, RegisterValue::U8(v)) => {
                self.enabled = v > 0;
                Ok(())
            }
            (RegisterId::AngPidP, RegisterValue::Float(v)) => {
                self.p_angle = v;
                Ok(())
            }
            (RegisterId::VelPidP, RegisterValue::Float(v)) => {
                self.pid_vel_p = v;
                Ok(())
            }
            (RegisterId::VelPidI, RegisterValue::Float(v)) => {
                self.pid_vel_i = v;
                Ok(())
            }
            (RegisterId::VelPidD, RegisterValue::Float(v)) => {
                self.pid_vel_d = v;
                Ok(())
            }
            (RegisterId::VoltageLimit, RegisterValue::Float(v)) => {
                self.voltage_limit = v;
                Ok(())
            }
            (RegisterId::VelocityLimit, RegisterValue::Float(v)) => {
                self.velocity_limit = v;
                Ok(())
            }
            _ => Ok(()),
        }
    }
}

// ---------------------------------------------------------------------------
// Quad Multi-Motor Shared State
// ---------------------------------------------------------------------------
#[derive(Clone, Copy)]
pub struct SharedQuadState {
    pub motors: [SharedServoState; 4],
    pub topology_mode: u8,
    pub sensor_map: [u8; 4],
    pub joint_targets: [f32; 4],
    pub joint_angles: [f32; 4],
    pub joint_velocities: [f32; 4],
    pub selected_motor: u8,
}

impl SharedQuadState {
    pub const fn new() -> Self {
        Self {
            motors: [SharedServoState::new(); 4],
            topology_mode: 0, // Four1Dof
            sensor_map: [0, 1, 2, 3],
            joint_targets: [0.0; 4],
            joint_angles: [0.0; 4],
            joint_velocities: [0.0; 4],
            selected_motor: 0,
        }
    }
}

static SHARED_QUAD: Mutex<CriticalSectionRawMutex, RefCell<SharedQuadState>> =
    Mutex::new(RefCell::new(SharedQuadState::new()));

// ---------------------------------------------------------------------------
// Concrete Type Aliases & Unified Sensor Enum
// ---------------------------------------------------------------------------
#[allow(dead_code)]
pub enum N20Sensor<I2C: embedded_hal::i2c::I2c> {
    Tmag(Tmag5273Sensor<I2C>),
    Fallback(DummySensor<EmbassyClock>),
}

impl<I2C: embedded_hal::i2c::I2c> Sensor<EmbassyClock> for N20Sensor<I2C> {
    fn base(&self) -> &SensorBase {
        match self {
            Self::Tmag(s) => <Tmag5273Sensor<I2C> as Sensor<EmbassyClock>>::base(s),
            Self::Fallback(s) => <DummySensor<EmbassyClock> as Sensor<EmbassyClock>>::base(s),
        }
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        match self {
            Self::Tmag(s) => <Tmag5273Sensor<I2C> as Sensor<EmbassyClock>>::base_mut(s),
            Self::Fallback(s) => <DummySensor<EmbassyClock> as Sensor<EmbassyClock>>::base_mut(s),
        }
    }

    fn get_sensor_angle(&self) -> f32 {
        match self {
            Self::Tmag(s) => <Tmag5273Sensor<I2C> as Sensor<EmbassyClock>>::get_sensor_angle(s),
            Self::Fallback(s) => <DummySensor<EmbassyClock> as Sensor<EmbassyClock>>::get_sensor_angle(s),
        }
    }
}

type MotorSensorType = N20Sensor<I2c<'static, Blocking, Master>>;
type MotorCurrentSenseType = CurrentMirrorSense<Stm32AdcPin, EmbassyClock>;
type DCMotorType = DCMotor<'static, MotorSensorType, QuadDCDriver, MotorCurrentSenseType, EmbassyClock>;

// ---------------------------------------------------------------------------
// Assembly Motor Adapter implementing MotorControl
// ---------------------------------------------------------------------------
pub struct AssemblyDCMotor<'a>(pub &'a mut DCMotorType);

impl<'a> MotorControl<EmbassyClock> for AssemblyDCMotor<'a> {
    fn shaft_angle(&self) -> f32 {
        self.0.shaft_angle()
    }

    fn shaft_velocity(&mut self) -> f32 {
        self.0.shaft_velocity()
    }

    fn velocity_limit(&self) -> f32 {
        self.0.velocity_limit
    }

    fn move_motor(&mut self, target: f32, _clock: &EmbassyClock) {
        self.0.move_motor(target);
    }

    fn loop_foc(&mut self, _clock: &EmbassyClock) {}
}

// ---------------------------------------------------------------------------
// 1 kHz Quad Motor Control & Kinematics Task
// ---------------------------------------------------------------------------
#[embassy_executor::task]
async fn motor_task(
    mut motors: [DCMotorType; 4],
    mut led_b: Output<'static>,
    mut led_r: Output<'static>,
    has_sensor: bool,
    mut down: DownChannel,
    mut uart: Option<Uart<'static, Blocking>>,
    mut vsens_pin: Stm32AdcPin,
) {

    TMAG_DETECTED.store(has_sensor, Ordering::Relaxed);
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut loop_count: u32 = 0;

    let mut assembly = ActuatorAssembly::<4, 4>::new().with_downsample(10);
    assembly.set_topology(TopologyPreset::Four1Dof);

    let mut lpf_currents = [
        LowPassFilter::new(0.015, 0.001, &CLOCK),
        LowPassFilter::new(0.015, 0.001, &CLOCK),
        LowPassFilter::new(0.015, 0.001, &CLOCK),
        LowPassFilter::new(0.015, 0.001, &CLOCK),
    ];

    let mut commander4 = Commander4Axis::new_with_clock(&CLOCK);

    let mut rtt_buf = [0u8; 64];
    let mut rtt_line: heapless::Vec<u8, 64> = heapless::Vec::new();

    rprintln!("Quad motor task started at 1 kHz with multi-topology kinematics.");

    loop {
        ticker.next().await;
        loop_count = loop_count.wrapping_add(1);

        // 1. Fetch updated state & topology from SHARED_QUAD
        let (topology_mode, sensor_map, joint_targets, motor_states) = SHARED_QUAD.lock(|s| {
            let s = s.borrow();
            (s.topology_mode, s.sensor_map, s.joint_targets, s.motors)
        });

        // 2. Check for dynamic topology changes
        let current_preset = assembly.topology();
        let target_preset = TopologyPreset::from_u8(topology_mode);
        if current_preset != target_preset {
            assembly.set_topology(target_preset);
            rprintln!("Dynamic topology switched to {:?} (mode={})", target_preset, topology_mode);
        }

        // 3. Apply motor parameters & targets
        for i in 0..4 {
            let s = &motor_states[i];
            let mot = &mut motors[i];

            if s.enabled && mot.enabled == 0 {
                mot.enable();
            } else if !s.enabled && mot.enabled != 0 {
                mot.disable();
            }

            mot.voltage_limit = s.voltage_limit;
            mot.velocity_limit = s.velocity_limit;
            mot.P_angle.set_p(s.p_angle);
            mot.PID_velocity.set_p(s.pid_vel_p);
            mot.PID_velocity.set_i(s.pid_vel_i);
            mot.PID_velocity.set_d(s.pid_vel_d);

            match s.controller_mode {
                0 => mot.controller = MotionControlType::angle,
                1 => mot.controller = MotionControlType::velocity_openloop,
                2 => mot.controller = MotionControlType::torque,
                _ => {}
            }

            assembly.set_joint_target(i, joint_targets[i]);
            assembly.set_sensor_mapping(i, sensor_map[i] as usize);
        }

        // 4. Update sensor channel feedback in assembly pool
        for i in 0..4 {
            let (angle, vel) = if let Some(s) = motors[i].sensor_mut() {
                (s.get_angle(), s.get_velocity())
            } else {
                (0.0, 0.0)
            };
            assembly.set_sensor_state(i, angle, vel);
        }

        // 5. Execute 1 kHz actuator assembly control loops
        {
            let (m0, rest) = motors.split_at_mut(1);
            let (m1, rest) = rest.split_at_mut(1);
            let (m2, m3) = rest.split_at_mut(1);

            let mut wrapped = [
                AssemblyDCMotor(&mut m0[0]),
                AssemblyDCMotor(&mut m1[0]),
                AssemblyDCMotor(&mut m2[0]),
                AssemblyDCMotor(&mut m3[0]),
            ];

            if assembly.topology() == TopologyPreset::Four1Dof {
                // In independent 1-DOF mode, drive motors directly from target setpoints
                for i in 0..4 {
                    let tgt = if motor_states[i].controller_mode == 0 && joint_targets[i] != 0.0 {
                        joint_targets[i]
                    } else {
                        motor_states[i].target
                    };
                    wrapped[i].move_motor(tgt, &CLOCK);
                }
            } else {
                // In coupled 2-DOF or mixed topologies, execute outer joint kinematics
                assembly.step_motors(&mut wrapped, &CLOCK);
            }
        }

        // 6. Current sensing, emergency trips & telemetry publish
        let mut measured_currents = [0.0f32; 4];
        let mut shaft_angles = [0.0f32; 4];
        let mut shaft_vels = [0.0f32; 4];
        let mut statuses = [0u8; 4];

        for i in 0..4 {
            let shaft_ang = motors[i].shaft_angle();
            let shaft_vel = motors[i].shaft_velocity();
            let raw_ma = motors[i]
                .current_sense_mut()
                .map(|cs| cs.get_current_magnitude() * 1000.0)
                .unwrap_or(0.0);
            let cur_ma = lpf_currents[i].update(raw_ma, &CLOCK);
            let stat = motors[i].motor_status as u8;

            // Emergency trip (> 600 mA)
            if cur_ma > 600.0 && motors[i].enabled != 0 {
                rprintln!("EMERGENCY OVERCURRENT TRIP ON MOTOR {}! I={:.1} mA > 600 mA. Disabled.", i, cur_ma);
                motors[i].disable();
                SHARED_QUAD.lock(|sq| {
                    sq.borrow_mut().motors[i].enabled = false;
                });
            }

            measured_currents[i] = cur_ma;
            shaft_angles[i] = shaft_ang;
            shaft_vels[i] = shaft_vel;
            statuses[i] = stat;
        }

        // 7. Measure hardware telemetry: VSENS (PB1) bus voltage & internal MCU temperature
        let raw_vsens = vsens_pin.read() as f32;
        let bus_voltage = (raw_vsens * 3.3 / 4095.0) * 2.0;
        let mcu_temp = read_internal_temperature();
        let max_current_a = measured_currents.iter().cloned().fold(0.0f32, f32::max) / 1000.0;

        // 8. Update unified Control Table with telemetry and evaluate error status
        SHARED_CONTROL_TABLE.lock(|ct| {
            let mut table = ct.borrow_mut();
            table.evaluate_errors(max_current_a, bus_voltage, mcu_temp);
            table.enforce_limits();

            table.write_f32(REG_PRESENT_BUS_VOLTAGE, bus_voltage);
            table.write_f32(REG_PRESENT_TEMPERATURE, mcu_temp);
            table.write_f32(REG_PRESENT_CURRENT, max_current_a);
            table.write_f32(REG_PRESENT_POSITION, shaft_angles[0]);
            table.write_f32(REG_PRESENT_VELOCITY, shaft_vels[0]);

            // SaveConfig trigger (0x55 written to 0x64)
            if table.read_u8(REG_SAVE_CONFIG_CMD) == CMD_SAVE_CONFIG {
                table.write_u8(REG_SAVE_CONFIG_CMD, 0);
                GLOBAL_STORAGE.lock(|gs| {
                    if let Some(ref mut st) = *gs.borrow_mut() {
                        let _ = st.save_config(&table);
                    }
                });
                rprintln!("Flash configuration saved to Bank 2 sectors.");
            }

            // RebootCommand trigger (0xF0 written to 0x63)
            if table.read_u8(REG_REBOOT_CMD) == CMD_REBOOT_BOOTLOADER {
                table.write_u8(REG_REBOOT_CMD, 0);
                // Disarm all drivers into Coast mode
                for m in motors.iter_mut() {
                    m.disable();
                }
                barefoc_stm32::bootloader::enter_bareboot(barefoc_stm32::bootloader::default_target());
            }

            // Red status LED reflects hardware error status
            if table.hardware_error_status() != 0 {
                led_r.set_low(); // Active low: LED ON
            } else {
                led_r.set_high(); // LED OFF
            }
        });

        // Publish back to SHARED_QUAD
        SHARED_QUAD.lock(|sq| {
            let mut sq = sq.borrow_mut();
            for i in 0..4 {
                sq.motors[i].current_angle = shaft_angles[i];
                sq.motors[i].current_velocity = shaft_vels[i];
                sq.motors[i].current_ma = measured_currents[i];
                sq.motors[i].status = statuses[i];
                sq.joint_angles[i] = assembly.joint_angle(i);
                sq.joint_velocities[i] = assembly.joint_velocity(i);
            }
        });


        // 7. Service RTT & USART1 Commander commands (every 10 ms = 100 Hz)
        if loop_count % 10 == 0 {
            // Read RTT DownChannel
            let count = down.read(&mut rtt_buf);
            for &b in &rtt_buf[..count] {
                if b == b'\r' || b == b'\n' {
                    if !rtt_line.is_empty() {
                        if let Ok(cmd_str) = core::str::from_utf8(&rtt_line) {
                            handle_unified_command(cmd_str.trim(), &mut commander4, &mut assembly);
                        }
                        rtt_line.clear();
                    }
                } else if rtt_line.push(b).is_err() {
                    rtt_line.clear();
                }
            }

            // Read USART1 non-blocking if present
            if let Some(ref mut _u) = uart {
                let regs = embassy_stm32::pac::USART1;
                while regs.isr().read().rxne() {
                    let b = (regs.rdr().read().dr() & 0xFF) as u8;
                    if b == b'\r' || b == b'\n' {
                        if !rtt_line.is_empty() {
                            if let Ok(cmd_str) = core::str::from_utf8(&rtt_line) {
                                handle_unified_command(cmd_str.trim(), &mut commander4, &mut assembly);
                            }
                            rtt_line.clear();
                        }
                    } else if rtt_line.push(b).is_err() {
                        rtt_line.clear();
                    }
                }
            }
        }

        // 8. Periodic telemetry & LED Status (2 Hz idle, 10 Hz active)
        let any_enabled = motors.iter().any(|m| m.enabled != 0);
        let log_interval = if any_enabled { 100 } else { 500 };
        if loop_count % log_interval == 0 {
            if TMAG_DETECTED.load(Ordering::Relaxed) {
                led_b.set_low();
                led_r.set_high();
            } else {
                led_r.toggle();
                led_b.set_high();
            }

            let sel = SHARED_QUAD.lock(|s| s.borrow().selected_motor) as usize;
            let topo_name = match assembly.topology() {
                TopologyPreset::Four1Dof => "4x1-DOF",
                TopologyPreset::Two2Dof => "2x2-DOF",
                TopologyPreset::One1DofOne2Dof => "1x1-DOF+1x2-DOF",
                TopologyPreset::Two1DofOne2Dof => "2x1-DOF+1x2-DOF",
                TopologyPreset::Custom => "Custom",
            };

            rprintln!(
                "[{}] Mot {}: tgt={:.2} | ang={:.2} ({:.1}°) | vel={:.1} | I={:.1} mA | en={}",
                topo_name,
                sel,
                motor_states[sel].target,
                shaft_angles[sel],
                shaft_angles[sel] * 180.0 / core::f32::consts::PI,
                shaft_vels[sel],
                measured_currents[sel],
                if motors[sel].enabled != 0 { 1 } else { 0 }
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Unified Command Handler (Motors A..D, Kinematics K, Diagnostics)
// ---------------------------------------------------------------------------
fn handle_unified_command(
    cmd: &str,
    commander: &mut Commander4Axis<EmbassyClock>,
    assembly: &mut ActuatorAssembly<4, 4>,
) {
    let trimmed = cmd.trim();
    if trimmed.is_empty() {
        return;
    }

    let first = trimmed.chars().next().unwrap_or(' ');
    match first {
        'A' | 'B' | 'C' | 'D' | 'a' | 'b' | 'c' | 'd' => {
            let mot_idx = match first.to_ascii_uppercase() {
                'A' => 0,
                'B' => 1,
                'C' => 2,
                'D' => 3,
                _ => 0,
            };
            let payload = trimmed[1..].trim_start();
            if payload.is_empty() {
                SHARED_QUAD.lock(|sq| sq.borrow_mut().selected_motor = mot_idx as u8);
                rprintln!("-> Selected Motor {} ('{}')", mot_idx, first.to_ascii_uppercase());
            } else if let Ok(val) = barefoc_commander::parse_f32(payload) {
                SHARED_QUAD.lock(|sq| {
                    let mut sq = sq.borrow_mut();
                    sq.motors[mot_idx].target = val;
                    sq.motors[mot_idx].enabled = true;
                });
                rprintln!("-> Motor {} target set to {:.3}", mot_idx, val);
            } else {
                commander.run(trimmed);
            }
        }
        'K' | 'k' => {
            let payload = trimmed[1..].trim_start();
            if payload.starts_with('M') || payload.starts_with('m') {
                let m_arg = payload[1..].trim_start();
                if let Ok(mode) = m_arg.parse::<u8>() {
                    let preset = TopologyPreset::from_u8(mode);
                    assembly.set_topology(preset);
                    SHARED_QUAD.lock(|sq| {
                        sq.borrow_mut().topology_mode = mode;
                    });
                    rprintln!("-> Kinematics topology mode set to {} ({:?})", mode, preset);
                } else {
                    rprintln!("-> Current topology mode: {}", assembly.topology_mode);
                }
            } else if payload.starts_with('T') || payload.starts_with('t') {
                let parts: heapless::Vec<&str, 4> = payload[1..].split_whitespace().collect();
                if parts.len() >= 2 {
                    if let (Ok(j), Ok(rad)) = (parts[0].parse::<usize>(), barefoc_commander::parse_f32(parts[1])) {
                        if j < 4 {
                            assembly.set_joint_target(j, rad);
                            SHARED_QUAD.lock(|sq| {
                                sq.borrow_mut().joint_targets[j] = rad;
                            });
                            rprintln!("-> Joint {} target set to {:.3} rad", j, rad);
                        }
                    }
                }
            } else {
                let _ = KinematicsCommander::handle(&mut commander.kinematics, payload);
            }
        }
        't' => {
            let parts: heapless::Vec<&str, 4> = trimmed.split_whitespace().collect();
            if parts.len() > 1 {
                if let Ok(val) = barefoc_commander::parse_f32(parts[1]) {
                    SHARED_QUAD.lock(|sq| {
                        let mut sq = sq.borrow_mut();
                        let sel = sq.selected_motor as usize;
                        sq.motors[sel].target = val;
                        sq.motors[sel].enabled = true;
                    });
                    rprintln!("-> Target set to {:.3} on selected motor", val);
                }
            }
        }
        'e' => {
            let parts: heapless::Vec<&str, 4> = trimmed.split_whitespace().collect();
            let en = if parts.len() > 1 {
                parts[1] == "1" || parts[1] == "on"
            } else {
                true
            };
            SHARED_QUAD.lock(|sq| {
                let mut sq = sq.borrow_mut();
                let sel = sq.selected_motor as usize;
                sq.motors[sel].enabled = en;
            });
            rprintln!("-> Selected motor enabled: {}", en);
        }
        's' => {
            let (sq, topo) = SHARED_QUAD.lock(|s| (*s.borrow(), assembly.topology()));
            rprintln!("=== FISHYSERVO 4-AXIS STATUS ===");
            rprintln!("  Topology: {:?} (mode={})", topo, sq.topology_mode);
            rprintln!("  Sensor Map: {:?}", sq.sensor_map);
            for i in 0..4 {
                let m = &sq.motors[i];
                rprintln!(
                    "  Motor {}: en={}, tgt={:.3}, ang={:.3} rad, vel={:.1} rad/s, I={:.1} mA",
                    i,
                    m.enabled,
                    m.target,
                    m.current_angle,
                    m.current_velocity,
                    m.current_ma
                );
            }
        }
        'h' | '?' => {
            rprintln!("Available Commands:");
            rprintln!("  A..D [val]     - Select motor or set target (e.g. 'A 1.57')");
            rprintln!("  KM <0..3>      - Set topology mode (0=4x1Dof, 1=2x2Dof, 2=1x1+1x2, 3=2x1+1x2)");
            rprintln!("  KT <j> <rad>   - Set joint target position");
            rprintln!("  KA <j>         - Query joint angle");
            rprintln!("  e 1 / 0        - Enable / Disable selected motor");
            rprintln!("  t <val>        - Set target on selected motor");
            rprintln!("  s              - Print 4-axis multi-topology status");
        }
        _ => {
            rprintln!("Unknown command: '{}'. Type 'h' for help.", trimmed);
        }
    }
}

// ---------------------------------------------------------------------------
// Handsfree Bootloader Entry
// ---------------------------------------------------------------------------
fn bootloader_write_hook(frame: &[u8]) {
    if frame.len() == 1 && frame[0] == 0xF0 {
        barefoc_stm32::bootloader::enter_bareboot(barefoc_stm32::bootloader::default_target());
    }
}

// Dedicated interrupt executor for I2C2 target task
static I2C_TARGET_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn TIM6() {
    unsafe { I2C_TARGET_EXECUTOR.on_interrupt() }
}

// ---------------------------------------------------------------------------
// MultiBus Topology Write Hook (0x78)
// ---------------------------------------------------------------------------
fn topology_mode_hook<const N: usize, B: RegisterBridge>(
    _ctx: &mut MultiMotorState<N, B>,
    _offset: u8,
    bytes: &[u8],
) -> HookResult {
    if !bytes.is_empty() {
        let mode = bytes[0];
        SHARED_QUAD.lock(|s| {
            s.borrow_mut().topology_mode = mode;
        });
        HookResult::Applied
    } else {
        HookResult::SelfClear
    }
}

// ---------------------------------------------------------------------------
// I2C2 Multi-Motor Multibus Server Task (Address 0x40)
// ---------------------------------------------------------------------------
#[embassy_executor::task]
async fn i2c_target_task(
    mut target: Stm32I2cTarget<'static, MultiMotorState<4, SharedServoState>, 384, 128>,
) {
    rprintln!("I2C2 Multi-Motor Multibus target server listening on address 0x40.");

    loop {
        // 1. Sync telemetry from SHARED_QUAD into target server
        let (selected, sq_motors) = SHARED_QUAD.lock(|s| {
            let s = s.borrow();
            (s.selected_motor, s.motors)
        });

        {
            let map = target.server_mut().map_mut();
            let _ = map.handler().set_selected(selected);
            for i in 0..4 {
                map.handler().motors[i] = sq_motors[i];
            }
            refresh_telemetry_multi(map);
        }

        // 2. Service bus transactions
        match target.service_once::<I2C2>() {
            Ok(ServiceOutcome::Idle) => {
                Timer::after_micros(200).await;
            }
            Ok(ServiceOutcome::WritePhase { .. }) => {
                let handler = target.server_mut().map_mut().handler();
                let sel = handler.selected();
                let motors = handler.motors;
                SHARED_QUAD.lock(|s| {
                    let mut s = s.borrow_mut();
                    s.selected_motor = sel;
                    for i in 0..4 {
                        s.motors[i].target = motors[i].target;
                        s.motors[i].enabled = motors[i].enabled;
                        s.motors[i].p_angle = motors[i].p_angle;
                        s.motors[i].pid_vel_p = motors[i].pid_vel_p;
                        s.motors[i].pid_vel_i = motors[i].pid_vel_i;
                        s.motors[i].pid_vel_d = motors[i].pid_vel_d;
                        s.motors[i].voltage_limit = motors[i].voltage_limit;
                        s.motors[i].velocity_limit = motors[i].velocity_limit;
                    }
                });
                embassy_futures::yield_now().await;
            }
            Ok(ServiceOutcome::WriteRejected { .. }) => {
                embassy_futures::yield_now().await;
            }
            Ok(ServiceOutcome::ReadPhase { .. }) => {
                embassy_futures::yield_now().await;
            }
            Err(_err) => {
                Timer::after_millis(1).await;
            }
        }
    }
}

fn set_i2c1_frequency(i2cclk: u32, frequency: u32) {
    let ratio = i2cclk / frequency;
    let presc = (ratio - 1) / 512;
    let presc_reg = core::cmp::min(presc, 15) as u8;
    let presc_val = (presc_reg + 1) as u32;
    let sclh = ((ratio / presc_val) - 2) / 2;
    let scll = sclh;
    let sdadel = core::cmp::max((i2cclk / 2_000_000 / presc_val) as u8, 2);
    let scldel = core::cmp::max((i2cclk / 500_000 / presc_val - 1) as u8, 4);

    embassy_stm32::pac::I2C1.cr1().modify(|w| w.set_pe(false));
    embassy_stm32::pac::I2C1.timingr().write(|w| {
        w.set_presc(presc_reg);
        w.set_scll(scll as u8);
        w.set_sclh(sclh as u8);
        w.set_sdadel(sdadel);
        w.set_scldel(scldel);
    });
    embassy_stm32::pac::I2C1.cr1().modify(|w| w.set_pe(true));
}

// ---------------------------------------------------------------------------
// Main System Initialization
// ---------------------------------------------------------------------------
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 2048,
                mode: rtt_target::ChannelMode::NoBlockTrim,
                name: "Terminal"
            }
        }
        down: {
            0: {
                size: 128,
                mode: rtt_target::ChannelMode::NoBlockTrim,
                name: "Terminal"
            }
        }
    };
    rtt_target::set_print_channel(channels.up.0);
    let rtt_down = channels.down.0;

    rprintln!("\r\n=======================================================");
    rprintln!("     FishyServo Quad H-Bridge Multi-Topology Servo     ");
    rprintln!("     Microcontroller: STM32H503KBU6 (Arm Cortex-M33)  ");
    rprintln!("     Bridges: A (PA15/PB5), B (PA8/PB15), C (PA6/PA7), D (PA9/PB0)");
    rprintln!("     Sensors: TMAG5273 on I2C1 (PB6/PB7)               ");
    rprintln!("     Host Bus: MultiBus on I2C2 (PB3/PB4 @ 0x40)      ");
    rprintln!("=======================================================");

    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = Some(embassy_stm32::rcc::HSIPrescaler::Div1);
    config.rcc.sys = embassy_stm32::rcc::Sysclk::Hsi;
    config.rcc.voltage_scale = embassy_stm32::rcc::VoltageScale::Scale3;
    let p = embassy_stm32::init(config);
    let boot_start = Instant::now();

    // DBGMCU: Keep CoreSight debug clocks running during sleep/stop
    embassy_stm32::pac::DBGMCU.cr().modify(|w| {
        w.set_stop(true);
        w.set_standby(true);
    });

    unsafe {
        let dbgmcu_cr = 0x4402_4004 as *mut u32;
        core::ptr::write_volatile(dbgmcu_cr, core::ptr::read_volatile(dbgmcu_cr) | 0x07);

        let dbgmcu_apb1lfzr = 0x4402_4008 as *mut u32;
        core::ptr::write_volatile(
            dbgmcu_apb1lfzr,
            core::ptr::read_volatile(dbgmcu_apb1lfzr) | (1 << 11) | (1 << 12),
        );
    }

    // -----------------------------------------------------------------------
    // USART1 Console (PA11 RX, PA12 TX @ 115200 baud)
    // -----------------------------------------------------------------------
    let mut usart_cfg = UartConfig::default();
    usart_cfg.baudrate = 115_200;
    let uart = Uart::new_blocking(p.USART1, p.PA12, p.PA11, usart_cfg).ok();

    // -----------------------------------------------------------------------
    // Non-Volatile Flash Persistence (Bank 2 Sectors 14 & 15: 16 KiB)
    // -----------------------------------------------------------------------
    let flash = embassy_stm32::flash::Flash::new_blocking(p.FLASH, embassy_stm32::flash::Config::default());
    let mut storage = H5FlashStorage::new(flash);
    let mut boot_table = ControlTable::new();
    if storage.load_config(&mut boot_table).is_err() {
        boot_table.set_error_flag(error_flags::FLASH_CORRUPT);
    }
    let node_id = boot_table.node_id();
    rprintln!("Loaded configuration from flash: NodeId = 0x{:02X}", node_id);
    SHARED_CONTROL_TABLE.lock(|ct| *ct.borrow_mut() = boot_table);
    GLOBAL_STORAGE.lock(|gs| *gs.borrow_mut() = Some(storage));

    // -----------------------------------------------------------------------
    // I2C2 Multi-Motor MultiBus Target (Address from persisted NodeId)
    // -----------------------------------------------------------------------
    let mut i2c2_cfg = I2cConfig::default();
    i2c2_cfg.sda_pullup = true;
    i2c2_cfg.scl_pullup = true;
    i2c2_cfg.frequency = Hertz::khz(100);
    let i2c2 = I2c::new_blocking(p.I2C2, p.PB3, p.PB4, i2c2_cfg);

    let initial_motors = [SharedServoState::new(); 4];
    let mut multibus_map = multi_motor_profile(initial_motors);

    // Register 0x78: TopologyMode RW hook for runtime topology switching
    let _ = multibus_map.add_register(RegEntry::rw_hooked(REG_TOPOLOGY_MODE, 1, topology_mode_hook));

    let server = RegServer::new(multibus_map);
    let mut target =
        Stm32I2cTarget::new(i2c2, node_id, server).expect("I2C2 Target binding failed");
    target.set_write_frame_hook(Some(bootloader_write_hook));

    interrupt::TIM6.set_priority(Priority::P8);
    let i2c_spawner = I2C_TARGET_EXECUTOR.start(interrupt::TIM6);
    i2c_spawner.spawn(i2c_target_task(target).expect("Failed to create i2c target task token"));

    Timer::after_millis(1).await;
    rprintln!("I2C2 target ready at 0x{:02X} ({} ms after init).", node_id, boot_start.elapsed().as_millis());


    // -----------------------------------------------------------------------
    // Status LEDs on PA0 (Blue) and PA1 (Red): Active LOW
    // -----------------------------------------------------------------------
    let mut led_b = Output::new(p.PA0, Level::Low, Speed::Low); // Blue ON while booting
    let led_r = Output::new(p.PA1, Level::High, Speed::Low); // Red OFF

    // -----------------------------------------------------------------------
    // Quad H-Bridge Motor PWM Timers (TIM1, TIM3 at 25 kHz)
    //
    // Channel mapping:
    // - Bridge A: PA15 (GPIO DIR) / PB5 (TIM1_CH3 PWM)
    // - Bridge B: PA8 (TIM1_CH1)  / PB15 (TIM3_CH4)
    // - Bridge C: PA6 (TIM3_CH1)  / PA7 (TIM3_CH2)
    // - Bridge D: PA9 (TIM1_CH2)  / PB0 (TIM3_CH3)
    // -----------------------------------------------------------------------
    let tim1_ch1_pin = PwmPin::new(p.PA8, OutputType::PushPull);
    let tim1_ch2_pin = PwmPin::new(p.PA9, OutputType::PushPull);
    let tim1_ch3_pin = PwmPin::new(p.PB5, OutputType::PushPull);
    let pwm1 = SimplePwm::new(
        p.TIM1,
        Some(tim1_ch1_pin),
        Some(tim1_ch2_pin),
        Some(tim1_ch3_pin),
        None,
        Hertz(25_000),
        CountingMode::EdgeAlignedUp,
    );

    let tim3_ch1_pin = PwmPin::new(p.PA6, OutputType::PushPull);
    let tim3_ch2_pin = PwmPin::new(p.PA7, OutputType::PushPull);
    let tim3_ch3_pin = PwmPin::new(p.PB0, OutputType::PushPull);
    let tim3_ch4_pin = PwmPin::new(p.PB15, OutputType::PushPull);
    let pwm3 = SimplePwm::new(
        p.TIM3,
        Some(tim3_ch1_pin),
        Some(tim3_ch2_pin),
        Some(tim3_ch3_pin),
        Some(tim3_ch4_pin),
        Hertz(25_000),
        CountingMode::EdgeAlignedUp,
    );

    let mut ch1 = pwm1.split();
    let mut ch3 = pwm3.split();

    ch1.ch1.enable();
    ch1.ch2.enable();
    ch1.ch3.enable();
    ch3.ch1.enable();
    ch3.ch2.enable();
    ch3.ch3.enable();
    ch3.ch4.enable();

    // Instantiate 4 H-Bridge Drivers
    let pin_pa15 = Output::new(p.PA15, Level::Low, Speed::VeryHigh);
    let mut driver_a = BridgeADriver::new(pin_pa15, ch1.ch3);
    driver_a.params_mut().voltage_power_supply = 5.0;
    driver_a.params_mut().voltage_limit = 5.0;
    driver_a.params_mut().pwm_frequency = 25_000;
    driver_a.init().expect("Driver A init failed");

    let mut driver_b = DCDriver2PWM::without_enable(ch1.ch1, ch3.ch4);
    driver_b.params_mut().voltage_power_supply = 5.0;
    driver_b.params_mut().voltage_limit = 5.0;
    driver_b.params_mut().pwm_frequency = 25_000;
    driver_b.init().expect("Driver B init failed");

    let mut driver_c = DCDriver2PWM::without_enable(ch3.ch1, ch3.ch2);
    driver_c.params_mut().voltage_power_supply = 5.0;
    driver_c.params_mut().voltage_limit = 5.0;
    driver_c.params_mut().pwm_frequency = 25_000;
    driver_c.init().expect("Driver C init failed");

    let mut driver_d = DCDriver2PWM::without_enable(ch1.ch2, ch3.ch3);
    driver_d.params_mut().voltage_power_supply = 5.0;
    driver_d.params_mut().voltage_limit = 5.0;
    driver_d.params_mut().pwm_frequency = 25_000;
    driver_d.init().expect("Driver D init failed");

    // Hold outputs in safe coast mode
    driver_a.set_pwm(0.0);
    driver_b.set_pwm(0.0);
    driver_c.set_pwm(0.0);
    driver_d.set_pwm(0.0);

    // -----------------------------------------------------------------------
    // ADC1 Configuration (ISENS_A..D on PA2..PA5)
    // -----------------------------------------------------------------------
    let adc1 = Adc::new_blocking(p.ADC1, embassy_stm32::adc::Config::default());
    GLOBAL_ADC.lock(|cell| {
        *cell.borrow_mut() = Some(adc1);
    });

    let adc_pin_a = Stm32AdcPin::new(AdcInputPin::ChA(p.PA2), SampleTime::Cycles65);
    let adc_pin_b = Stm32AdcPin::new(AdcInputPin::ChB(p.PA3), SampleTime::Cycles65);
    let adc_pin_c = Stm32AdcPin::new(AdcInputPin::ChC(p.PA4), SampleTime::Cycles65);
    let adc_pin_d = Stm32AdcPin::new(AdcInputPin::ChD(p.PA5), SampleTime::Cycles65);
    let vsens_pin = Stm32AdcPin::new(AdcInputPin::Vsens(p.PB1), SampleTime::Cycles65);

    let mut cs_a = CurrentMirrorSense::new(adc_pin_a, 200.0, 1000.0, 3.3, 12);
    let mut cs_b = CurrentMirrorSense::new(adc_pin_b, 200.0, 1000.0, 3.3, 12);
    let mut cs_c = CurrentMirrorSense::new(adc_pin_c, 200.0, 1000.0, 3.3, 12);
    let mut cs_d = CurrentMirrorSense::new(adc_pin_d, 200.0, 1000.0, 3.3, 12);

    cs_a.calibrate_offsets_rounds(&CLOCK, 200);
    cs_b.calibrate_offsets_rounds(&CLOCK, 200);
    cs_c.calibrate_offsets_rounds(&CLOCK, 200);
    cs_d.calibrate_offsets_rounds(&CLOCK, 200);

    // -----------------------------------------------------------------------
    // Sensor Bus I2C1 (PB6/PB7) & TMAG5273 Discovery
    // -----------------------------------------------------------------------
    let mut i2c1_cfg = I2cConfig::default();
    i2c1_cfg.sda_pullup = true;
    i2c1_cfg.scl_pullup = true;
    i2c1_cfg.frequency = Hertz::khz(50);
    i2c1_cfg.timeout = Duration::from_millis(10);
    let mut i2c1 = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, i2c1_cfg);

    rprintln!("=======================================================");
    rprintln!("     I2C1 TMAG5273 DISCOVERY (PB6/PB7 Pull-Ups)        ");
    rprintln!("=======================================================");

    let speeds_khz = [20u32, 50u32, 100u32];
    let candidate_addrs = [0x35u8, 0x22, 0x78, 0x44];
    let mut detected_tmag = None;

    for &khz in &speeds_khz {
        set_i2c1_frequency(64_000_000, khz * 1000);
        let mut _count = 0usize;
        for &addr in &candidate_addrs {
            let mut b = [0u8; 1];
            let ack = i2c1.blocking_read(addr, &mut b).is_ok();
            if ack {
                _count += 1;
                let mut mfg = [0u8; 2];
                if i2c1.blocking_write_read(addr, &[0x0E], &mut mfg).is_ok() {
                    let mfg_id = u16::from_le_bytes(mfg);
                    if mfg_id == 0x5449 {
                        let mut dev_id = [0u8; 1];
                        let is_v2 = if i2c1.blocking_write_read(addr, &[0x0D], &mut dev_id).is_ok() {
                            (dev_id[0] & 0x03) == 2
                        } else {
                            false
                        };
                        let variant = match (addr, is_v2) {
                            (0x22, false) => DeviceVariant::B1,
                            (0x22, true) => DeviceVariant::B2,
                            (0x78, false) => DeviceVariant::C1,
                            (0x78, true) => DeviceVariant::C2,
                            (0x44, false) => DeviceVariant::D1,
                            (0x44, true) => DeviceVariant::D2,
                            (_, true) => DeviceVariant::A2,
                            (_, false) => DeviceVariant::A1,
                        };
                        rprintln!("  [+] Found TI TMAG5273 ({:?}) at 0x{:02X}", variant, addr);
                        if detected_tmag.is_none() {
                            detected_tmag = Some((addr, variant));
                        }
                    }
                }
            }
        }
        if detected_tmag.is_some() {
            break;
        }
    }

    Timer::after_millis(1).await;
    set_i2c1_frequency(64_000_000, 50_000);

    let (sensor_a, has_sensor) = if let Some((addr, variant)) = detected_tmag {
        match Tmag5273::builder(i2c1, addr)
            .variant(variant)
            .operating_mode(OperatingMode::ContinuousMeasure)
            .channels(MagneticChannel::XYZ)
            .angle_enabled(AngleEnable::XY)
            .build()
        {
            Ok(dev) => {
                rprintln!("TMAG5273 Sensor initialized successfully on I2C1 (XY Angle enabled).");
                (N20Sensor::Tmag(Tmag5273Sensor::new(dev)), true)
            }
            Err(e) => {
                rprintln!("WARN: TMAG5273 init failed: {:?}. Fallback to dummy sensor.", e);
                (N20Sensor::Fallback(DummySensor::new()), false)
            }
        }
    } else {
        rprintln!("NOTICE: No TMAG5273 detected on I2C1. Fallback to dummy sensors.");
        (N20Sensor::Fallback(DummySensor::new()), false)
    };

    let sensor_b = N20Sensor::Fallback(DummySensor::new());
    let sensor_c = N20Sensor::Fallback(DummySensor::new());
    let sensor_d = N20Sensor::Fallback(DummySensor::new());

    // -----------------------------------------------------------------------
    // Instantiate 4 DCMotor Instances
    // -----------------------------------------------------------------------
    let mut motor_a = DCMotor::new(&CLOCK);
    motor_a.link_driver(QuadDCDriver::A(driver_a));
    motor_a.link_sensor(sensor_a);
    motor_a.link_current_sense(cs_a);
    motor_a.sensor_direction = Direction::CCW;
    motor_a.voltage_limit = 4.0;
    motor_a.velocity_limit = 50.0;
    motor_a.current_limit = 0.5;
    motor_a.controller = if has_sensor { MotionControlType::angle } else { MotionControlType::velocity_openloop };
    motor_a.torque_controller = TorqueControlType::voltage;
    motor_a.P_angle.set_p(15.0);
    motor_a.PID_velocity.set_p(1.2);
    motor_a.PID_velocity.set_i(4.0);
    motor_a.PID_velocity.set_d(0.0005);
    let _ = motor_a.init();

    let mut motor_b = DCMotor::new(&CLOCK);
    motor_b.link_driver(QuadDCDriver::B(driver_b));
    motor_b.link_sensor(sensor_b);
    motor_b.link_current_sense(cs_b);
    motor_b.sensor_direction = Direction::CCW;
    motor_b.voltage_limit = 4.0;
    motor_b.velocity_limit = 50.0;
    motor_b.current_limit = 0.5;
    motor_b.controller = MotionControlType::velocity_openloop;
    motor_b.torque_controller = TorqueControlType::voltage;
    motor_b.P_angle.set_p(15.0);
    motor_b.PID_velocity.set_p(1.2);
    motor_b.PID_velocity.set_i(4.0);
    motor_b.PID_velocity.set_d(0.0005);
    let _ = motor_b.init();

    let mut motor_c = DCMotor::new(&CLOCK);
    motor_c.link_driver(QuadDCDriver::C(driver_c));
    motor_c.link_sensor(sensor_c);
    motor_c.link_current_sense(cs_c);
    motor_c.sensor_direction = Direction::CCW;
    motor_c.voltage_limit = 4.0;
    motor_c.velocity_limit = 50.0;
    motor_c.current_limit = 0.5;
    motor_c.controller = MotionControlType::velocity_openloop;
    motor_c.torque_controller = TorqueControlType::voltage;
    motor_c.P_angle.set_p(15.0);
    motor_c.PID_velocity.set_p(1.2);
    motor_c.PID_velocity.set_i(4.0);
    motor_c.PID_velocity.set_d(0.0005);
    let _ = motor_c.init();

    let mut motor_d = DCMotor::new(&CLOCK);
    motor_d.link_driver(QuadDCDriver::D(driver_d));
    motor_d.link_sensor(sensor_d);
    motor_d.link_current_sense(cs_d);
    motor_d.sensor_direction = Direction::CCW;
    motor_d.voltage_limit = 4.0;
    motor_d.velocity_limit = 50.0;
    motor_d.current_limit = 0.5;
    motor_d.controller = MotionControlType::velocity_openloop;
    motor_d.torque_controller = TorqueControlType::voltage;
    motor_d.P_angle.set_p(15.0);
    motor_d.PID_velocity.set_p(1.2);
    motor_d.PID_velocity.set_i(4.0);
    motor_d.PID_velocity.set_d(0.0005);
    let _ = motor_d.init();

    led_b.set_high(); // Boot complete
    rprintln!("Boot complete: 4 motors initialized in safe coast mode ({} ms).", boot_start.elapsed().as_millis());

    let motors = [motor_a, motor_b, motor_c, motor_d];
    spawner.spawn(motor_task(motors, led_b, led_r, has_sensor, rtt_down, uart, vsens_pin).expect("Failed to create motor task token"));

    rprintln!("System running.");
}
