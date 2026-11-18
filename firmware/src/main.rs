#![no_std]
#![no_main]

use core::cell::RefCell;
use barefoc::common::filter::LowPassFilter;
use barefoc::common::time::Clock;
use barefoc::motor::enums::{MotionControlType, TorqueControlType};
use barefoc::traits::adc::AdcPin;
use barefoc::traits::sensor::{Direction, Sensor};
use barefoc_drivers::comms::registers::{RegisterError, RegisterId, RegisterValue};
use barefoc_drivers::comms::RegisterBridge;
use barefoc_drivers::dc::traits::DCDriver;
use barefoc_drivers::dc::{CurrentMirrorSense, DCDriver2PWM, DCMotor};
use barefoc_multibus::transport::RegServer;
use barefoc_stm32::bus_target_i2c::{ServiceOutcome, Stm32I2cTarget};
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{ADC1, I2C2, PA5, TIM1, TIM3};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel};
use embassy_stm32::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use panic_rtt_target as _;
use rtt_target::{rprintln, DownChannel};
use tmag_rs::barefoc::Tmag5273Sensor;
use tmag_rs::{AngleEnable, DeviceVariant, MagneticChannel, OperatingMode, Tmag5273};
use core::sync::atomic::{AtomicBool, Ordering};

static TMAG_DETECTED: AtomicBool = AtomicBool::new(false);

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
// ADC1 PA5 Pin adapter implementing barefoc::traits::adc::AdcPin
// ---------------------------------------------------------------------------
pub struct Stm32AdcPa5<'d> {
    adc: Adc<'d, ADC1, Blocking>,
    pin: Peri<'d, PA5>,
    sample_time: SampleTime,
}

impl<'d> Stm32AdcPa5<'d> {
    pub fn new(adc: Adc<'d, ADC1, Blocking>, pin: Peri<'d, PA5>, sample_time: SampleTime) -> Self {
        Self {
            adc,
            pin,
            sample_time,
        }
    }
}

impl<'d> AdcPin for Stm32AdcPa5<'d> {
    fn read(&mut self) -> u16 {
        let mut sum: u32 = 0;
        const SAMPLES: u32 = 8;
        for _ in 0..SAMPLES {
            sum += self.adc.blocking_read(&mut self.pin, self.sample_time) as u32;
        }
        (sum / SAMPLES) as u16
    }
}

// ---------------------------------------------------------------------------
// Shared Servo State and Register Bridge
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

// Global thread-safe shared state between motor_task and i2c_target_task
static SHARED_SERVO: Mutex<CriticalSectionRawMutex, RefCell<SharedServoState>> =
    Mutex::new(RefCell::new(SharedServoState::new()));

// ---------------------------------------------------------------------------
// Concrete Type Aliases & Unified Sensor Enum
// ---------------------------------------------------------------------------
#[allow(dead_code)]
pub enum N20Sensor<I2C: embedded_hal::i2c::I2c> {
    Tmag(Tmag5273Sensor<I2C>),
    Fallback(barefoc_drivers::dc::DummySensor<EmbassyClock>),
}

impl<I2C: embedded_hal::i2c::I2c> barefoc::traits::sensor::Sensor<EmbassyClock> for N20Sensor<I2C> {
    fn base(&self) -> &barefoc::traits::sensor::SensorBase {
        match self {
            Self::Tmag(s) => <Tmag5273Sensor<I2C> as barefoc::traits::sensor::Sensor<EmbassyClock>>::base(s),
            Self::Fallback(s) => <barefoc_drivers::dc::DummySensor<EmbassyClock> as barefoc::traits::sensor::Sensor<EmbassyClock>>::base(s),
        }
    }

    fn base_mut(&mut self) -> &mut barefoc::traits::sensor::SensorBase {
        match self {
            Self::Tmag(s) => <Tmag5273Sensor<I2C> as barefoc::traits::sensor::Sensor<EmbassyClock>>::base_mut(s),
            Self::Fallback(s) => <barefoc_drivers::dc::DummySensor<EmbassyClock> as barefoc::traits::sensor::Sensor<EmbassyClock>>::base_mut(s),
        }
    }

    fn get_sensor_angle(&self) -> f32 {
        match self {
            Self::Tmag(s) => <Tmag5273Sensor<I2C> as barefoc::traits::sensor::Sensor<EmbassyClock>>::get_sensor_angle(s),
            Self::Fallback(s) => <barefoc_drivers::dc::DummySensor<EmbassyClock> as barefoc::traits::sensor::Sensor<EmbassyClock>>::get_sensor_angle(s),
        }
    }
}

type MotorDriverType = DCDriver2PWM<
    SimplePwmChannel<'static, TIM1>,
    SimplePwmChannel<'static, TIM3>,
>;

type MotorSensorType = N20Sensor<I2c<'static, Blocking, Master>>;

type MotorCurrentSenseType = CurrentMirrorSense<Stm32AdcPa5<'static>, EmbassyClock>;

type DCMotorType = DCMotor<
    'static,
    MotorSensorType,
    MotorDriverType,
    MotorCurrentSenseType,
    EmbassyClock,
>;

// ---------------------------------------------------------------------------
// 1 kHz Motor Control Task
// ---------------------------------------------------------------------------
#[derive(PartialEq, Clone, Copy)]
enum HomingPhase {
    Idle,
    DriveFwd { step: u32, over_cnt: u32 },
    RelaxFwd { step: u32 },
    DriveRev { step: u32, over_cnt: u32 },
    Done,
}

#[embassy_executor::task]
async fn motor_task(
    mut motor: DCMotorType,
    mut led_b: Output<'static>,
    mut led_r: Output<'static>,
    has_sensor: bool,
    mut down: DownChannel,
) {
    TMAG_DETECTED.store(has_sensor, Ordering::Relaxed);
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut last_target = 0.0f32;
    let mut loop_count: u32 = 0;

    let mut homing = HomingPhase::Idle;
    let mut fwd_stop_angle = 0.0f32;
    let mut rev_stop_angle = 0.0f32;
    let mut fwd_stall_ma = 0.0f32;
    let mut rev_stall_ma = 0.0f32;
    let mut fwd_duration_ms: u32 = 0;
    let mut rev_duration_ms: u32 = 0;
    let mut peak_ma_fwd = 0.0f32;
    let mut peak_ma_rev = 0.0f32;
    let mut run_ma_fwd = 0.0f32;
    let mut run_ma_rev = 0.0f32;
    let mut last_current_ma = 0.0f32;
    let mut current_shaft_angle = 0.0f32;
    let mut lpf_current = LowPassFilter::new(0.015, 0.001, &CLOCK);

    let mut rtt_buf = [0u8; 64];
    let mut rtt_line: heapless::Vec<u8, 64> = heapless::Vec::new();

    rprintln!("Motor task started at 1 kHz with integrated RTT console.");

    loop {
        ticker.next().await;
        loop_count = loop_count.wrapping_add(1);

        // 1. Fetch updated target & tuning from shared state
        let (target, enabled, p_ang, vel_p, vel_i, vel_d, v_lim, vel_lim, mode, home_req, home_v, stall_thresh) =
            SHARED_SERVO.lock(|s| {
                let s = s.borrow();
                (
                    s.target,
                    s.enabled,
                    s.p_angle,
                    s.pid_vel_p,
                    s.pid_vel_i,
                    s.pid_vel_d,
                    s.voltage_limit,
                    s.velocity_limit,
                    s.controller_mode,
                    s.homing_requested,
                    s.homing_voltage,
                    s.stall_threshold_ma,
                )
            });

        // 2. Trigger homing if requested
        if home_req && homing == HomingPhase::Idle {
            homing = HomingPhase::DriveFwd { step: 0, over_cnt: 0 };
            peak_ma_fwd = 0.0;
            peak_ma_rev = 0.0;
            run_ma_fwd = 0.0;
            run_ma_rev = 0.0;
            SHARED_SERVO.lock(|s| {
                let mut s = s.borrow_mut();
                s.homing_requested = false;
                s.enabled = true;
            });
            motor.enable();
            motor.controller = MotionControlType::torque;
            rprintln!("Starting stall-detection endpoint homing (V={:.2}V, stall_thresh={:.1}mA)...", home_v, stall_thresh);
        }

        // 3. Apply dynamic parameter changes
        if (target - last_target).abs() > 1e-6 {
            motor.target = target;
            last_target = target;
        }

        if enabled && motor.enabled == 0 {
            motor.enable();
        } else if !enabled && motor.enabled != 0 && homing == HomingPhase::Idle {
            motor.disable();
        }

        motor.voltage_limit = v_lim;
        motor.velocity_limit = vel_lim;
        motor.P_angle.set_p(p_ang);
        motor.PID_velocity.set_p(vel_p);
        motor.PID_velocity.set_i(vel_i);
        motor.PID_velocity.set_d(vel_d);

        match mode {
            0 => motor.controller = MotionControlType::angle,
            1 => motor.controller = MotionControlType::velocity_openloop,
            2 => motor.controller = MotionControlType::torque,
            _ => {}
        }

        // 4. Homing state machine
        let effective_target = match homing {
            HomingPhase::Idle => target,
            HomingPhase::DriveFwd { ref mut step, ref mut over_cnt } => {
                *step += 1;
                motor.controller = MotionControlType::torque;
                // Timeout after 15 seconds to allow full stroke travel
                if *step > 15000 {
                    rprintln!("Homing FWD timeout (15s). Halting.");
                    homing = HomingPhase::Idle;
                    0.0
                } else {
                    if *step > 150 {
                        if *step < 450 {
                            if run_ma_fwd == 0.0 {
                                run_ma_fwd = last_current_ma;
                            } else {
                                run_ma_fwd += (last_current_ma - run_ma_fwd) * 0.05;
                            }
                        }
                        if last_current_ma > peak_ma_fwd {
                            peak_ma_fwd = last_current_ma;
                        }
                    }
                    let is_stall = (last_current_ma > stall_thresh)
                        || (*step > 350 && run_ma_fwd > 5.0 && last_current_ma > run_ma_fwd + 40.0 && last_current_ma > 60.0);

                    if *step > 150 && is_stall {
                        *over_cnt += 1;
                        if *over_cnt >= 20 {
                            fwd_stop_angle = if let Some(s) = motor.sensor() { s.get_angle() } else { current_shaft_angle };
                            fwd_stall_ma = last_current_ma;
                            fwd_duration_ms = *step;
                            rprintln!(
                                "-> Forward hard stop detected! I={:.1} mA (peak={:.1} mA, run={:.1} mA) at {} ms (raw={:.3} rad)",
                                last_current_ma, peak_ma_fwd, run_ma_fwd, *step, fwd_stop_angle
                            );
                            homing = HomingPhase::RelaxFwd { step: 0 };
                        }
                    } else {
                        *over_cnt = over_cnt.saturating_sub(1);
                    }
                    home_v
                }
            }
            HomingPhase::RelaxFwd { ref mut step } => {
                *step += 1;
                motor.controller = MotionControlType::torque;
                if *step >= 250 {
                    homing = HomingPhase::DriveRev { step: 0, over_cnt: 0 };
                }
                0.0
            }
            HomingPhase::DriveRev { ref mut step, ref mut over_cnt } => {
                *step += 1;
                motor.controller = MotionControlType::torque;
                if *step > 15000 {
                    rprintln!("Homing REV timeout (15s). Halting.");
                    homing = HomingPhase::Idle;
                    0.0
                } else {
                    if *step > 150 {
                        if *step < 450 {
                            if run_ma_rev == 0.0 {
                                run_ma_rev = last_current_ma;
                            } else {
                                run_ma_rev += (last_current_ma - run_ma_rev) * 0.05;
                            }
                        }
                        if last_current_ma > peak_ma_rev {
                            peak_ma_rev = last_current_ma;
                        }
                    }
                    let is_stall = (last_current_ma > stall_thresh)
                        || (*step > 350 && run_ma_rev > 5.0 && last_current_ma > run_ma_rev + 40.0 && last_current_ma > 60.0);

                    if *step > 150 && is_stall {
                        *over_cnt += 1;
                        if *over_cnt >= 20 {
                            rev_stop_angle = if let Some(s) = motor.sensor() { s.get_angle() } else { current_shaft_angle };
                            rev_stall_ma = last_current_ma;
                            rev_duration_ms = *step;
                            rprintln!(
                                "-> Reverse hard stop detected! I={:.1} mA (peak={:.1} mA, run={:.1} mA) at {} ms (raw={:.3} rad)",
                                last_current_ma, peak_ma_rev, run_ma_rev, *step, rev_stop_angle
                            );
                            homing = HomingPhase::Done;
                        }
                    } else {
                        *over_cnt = over_cnt.saturating_sub(1);
                    }
                    -home_v
                }
            }
            HomingPhase::Done => {
                let center_raw = (fwd_stop_angle + rev_stop_angle) * 0.5;
                motor.sensor_offset = -center_raw;
                motor.controller = MotionControlType::angle;
                rprintln!("=======================================================");
                rprintln!("     OPEN-LOOP STALL HOMING + TMAG ANGLE RANGE REPORT  ");
                rprintln!("=======================================================");
                let fwd_deg = fwd_stop_angle * 180.0 / core::f32::consts::PI;
                let rev_deg = rev_stop_angle * 180.0 / core::f32::consts::PI;
                let stroke = (fwd_stop_angle - rev_stop_angle).abs();
                let stroke_deg = stroke * 180.0 / core::f32::consts::PI;
                rprintln!("  Forward Stop (raw):  {:.3} rad ({:.1}°), I_stall={:.1} mA, travel={} ms",
                    fwd_stop_angle, fwd_deg, fwd_stall_ma, fwd_duration_ms);
                rprintln!("  Reverse Stop (raw):  {:.3} rad ({:.1}°), I_stall={:.1} mA, travel={} ms",
                    rev_stop_angle, rev_deg, rev_stall_ma, rev_duration_ms);
                rprintln!("  Auto-Calibrated Center: {:.3} rad (zero reference)", center_raw);
                rprintln!("  Total Stroke Range:  {:.3} rad ({:.1}°)", stroke, stroke_deg);
                rprintln!("  Switching to Closed-Loop [ANG] holding at 0.0 rad (center)...");
                rprintln!("=======================================================");
                homing = HomingPhase::Idle;
                SHARED_SERVO.lock(|s| {
                    let mut s = s.borrow_mut();
                    s.controller_mode = 0; // Angle mode
                    s.target = 0.0;
                    s.enabled = true; // active closed-loop holding
                });
                motor.enable();
                0.0
            }
        };

        // 5. Step closed-loop servo control loop
        motor.move_motor(effective_target);

        // 6. Publish real-time telemetry back to shared state
        let shaft_angle = motor.shaft_angle();
        let shaft_velocity = motor.shaft_velocity();
        current_shaft_angle = shaft_angle;
        let current_ma_raw = motor
            .current_sense_mut()
            .map(|cs| cs.get_current_magnitude() * 1000.0)
            .unwrap_or(0.0);
        let current_ma = lpf_current.update(current_ma_raw, &CLOCK);
        last_current_ma = current_ma;
        let status = motor.motor_status as u8;

        // Emergency overcurrent trip (> 600 mA)
        if current_ma > 600.0 && motor.enabled != 0 {
            rprintln!("EMERGENCY OVERCURRENT TRIP! I={:.1} mA > 600 mA. Bridge disabled.", current_ma);
            motor.disable();
            SHARED_SERVO.lock(|s| {
                let mut s = s.borrow_mut();
                s.enabled = false;
                s.target = 0.0;
            });
            homing = HomingPhase::Idle;
        }

        SHARED_SERVO.lock(|s| {
            let mut s = s.borrow_mut();
            s.current_angle = shaft_angle;
            s.current_velocity = shaft_velocity;
            s.current_ma = current_ma;
            s.status = status;
        });

        // 7. Service RTT interactive console commands (every 10 ms = 100 Hz)
        if loop_count % 10 == 0 {
            let count = down.read(&mut rtt_buf);
            for &b in &rtt_buf[..count] {
                if b == b'\r' || b == b'\n' {
                    if !rtt_line.is_empty() {
                        if let Ok(cmd_str) = core::str::from_utf8(&rtt_line) {
                            handle_rtt_command(cmd_str.trim());
                        }
                        rtt_line.clear();
                    }
                } else if rtt_line.push(b).is_err() {
                    rtt_line.clear();
                }
            }
        }

        // 8. Periodic RTT Telemetry & LED Status (2 Hz idle, 10 Hz moving/homing)
        let log_interval = if homing != HomingPhase::Idle || motor.enabled != 0 { 100 } else { 500 };
        if loop_count % log_interval == 0 {
            if TMAG_DETECTED.load(Ordering::Relaxed) {
                // TMAG sensor detected: stop blinking (steady Blue ON, Red OFF)
                led_b.set_low();
                led_r.set_high();
            } else {
                // TMAG sensor missing: blink Red (open-loop fallback warning)
                led_r.toggle();
                led_b.set_high();
            }

            let deg = shaft_angle * 180.0 / core::f32::consts::PI;
            let tgt_deg = effective_target * 180.0 / core::f32::consts::PI;
            let mode_str = if homing != HomingPhase::Idle {
                "HOMING"
            } else {
                match motor.controller {
                    MotionControlType::angle => "ANG",
                    MotionControlType::velocity_openloop => "VEL_OL",
                    MotionControlType::velocity => "VEL",
                    MotionControlType::torque => "VOLT",
                    _ => "OTHER",
                }
            };
            rprintln!(
                "[{}] tgt={:.2} ({:.1}°) | ang={:.2} ({:.1}°) | vel={:.1} rad/s | I={:.1} mA | en={}",
                mode_str,
                effective_target,
                tgt_deg,
                shaft_angle,
                deg,
                shaft_velocity,
                current_ma,
                if motor.enabled != 0 { 1 } else { 0 }
            );
        }
    }
}

fn handle_rtt_command(cmd: &str) {
    let parts: heapless::Vec<&str, 4> = cmd.split_whitespace().collect();
    if parts.is_empty() {
        return;
    }
    match parts[0] {
        "t" => {
            if parts.len() > 1 {
                if let Ok(val) = parts[1].parse::<f32>() {
                    let clamped = val.clamp(-0.80, 0.80);
                    SHARED_SERVO.lock(|s| {
                        let mut s = s.borrow_mut();
                        s.target = clamped;
                        s.controller_mode = 0; // ensure closed-loop angle mode
                        s.enabled = true;
                    });
                    let deg = clamped * 180.0 / core::f32::consts::PI;
                    rprintln!("-> Target angle set to {:.3} rad ({:.1}°) [ANG mode, enabled]", clamped, deg);
                }
            }
        }
        "d" => {
            if parts.len() > 1 {
                if let Ok(val) = parts[1].parse::<f32>() {
                    let clamped_deg = val.clamp(-46.0, 46.0);
                    let rad = clamped_deg * core::f32::consts::PI / 180.0;
                    SHARED_SERVO.lock(|s| {
                        let mut s = s.borrow_mut();
                        s.target = rad;
                        s.controller_mode = 0; // ensure closed-loop angle mode
                        s.enabled = true;
                    });
                    rprintln!("-> Target angle set to {:.1}° ({:.3} rad) [ANG mode, enabled]", clamped_deg, rad);
                }
            }
        }
        "e" => {
            if parts.len() > 1 {
                let en = parts[1] == "1" || parts[1] == "on";
                SHARED_SERVO.lock(|s| s.borrow_mut().enabled = en);
                rprintln!("-> Motor enabled: {}", en);
            } else {
                let en = SHARED_SERVO.lock(|s| {
                    let mut s = s.borrow_mut();
                    s.enabled = !s.enabled;
                    s.enabled
                });
                rprintln!("-> Motor enabled toggled to: {}", en);
            }
        }
        "p" => {
            if parts.len() > 1 {
                if let Ok(val) = parts[1].parse::<f32>() {
                    SHARED_SERVO.lock(|s| s.borrow_mut().p_angle = val);
                    rprintln!("-> P_angle gain set to {:.2}", val);
                }
            }
        }
        "v" => {
            if parts.len() > 1 {
                if let Ok(val) = parts[1].parse::<f32>() {
                    SHARED_SERVO.lock(|s| s.borrow_mut().pid_vel_p = val);
                    rprintln!("-> PID_velocity P gain set to {:.2}", val);
                }
            }
        }
        "u" => {
            if parts.len() > 1 {
                if let Ok(val) = parts[1].parse::<f32>() {
                    SHARED_SERVO.lock(|s| {
                        let mut s = s.borrow_mut();
                        s.target = val;
                        s.controller_mode = 2; // voltage mode
                        s.enabled = true;
                    });
                    rprintln!("-> Voltage mode target set to {:.2} V (motor enabled)", val);
                }
            }
        }
        "m" => {
            if parts.len() > 1 {
                let mode_code = match parts[1] {
                    "a" | "angle" => 0,
                    "v" | "vel" | "velocity" => 1,
                    "u" | "volt" | "voltage" => 2,
                    _ => 255,
                };
                if mode_code != 255 {
                    SHARED_SERVO.lock(|s| s.borrow_mut().controller_mode = mode_code);
                    rprintln!("-> Control mode changed to {}", parts[1]);
                } else {
                    rprintln!("Unknown mode. Use 'angle', 'vel', or 'volt'");
                }
            }
        }
        "s" => {
            let (target, angle, vel, cur, en, p, vp, vi, mode) = SHARED_SERVO.lock(|s| {
                let s = s.borrow();
                (
                    s.target,
                    s.current_angle,
                    s.current_velocity,
                    s.current_ma,
                    s.enabled,
                    s.p_angle,
                    s.pid_vel_p,
                    s.pid_vel_i,
                    s.controller_mode,
                )
            });
            let deg = angle * 180.0 / core::f32::consts::PI;
            let tgt_deg = target * 180.0 / core::f32::consts::PI;
            let mode_str = match mode {
                0 => "Angle (closed-loop)",
                1 => "Velocity (open-loop)",
                2 => "Voltage (open-loop)",
                _ => "Unknown",
            };
            rprintln!("=== SERVO STATUS ===");
            rprintln!("  Mode:           {}", mode_str);
            rprintln!("  Enabled:        {}", en);
            rprintln!("  Target:         {:.3} ({:.1}° / {:.2} V)", target, tgt_deg, target);
            rprintln!("  Current Angle:  {:.3} rad ({:.1}°)", angle, deg);
            rprintln!("  Velocity:       {:.2} rad/s", vel);
            rprintln!("  Current:        {:.1} mA", cur);
            rprintln!("  Gains:          P_ang={:.2}, Vel_P={:.2}, Vel_I={:.2}", p, vp, vi);
        }
        "cal" | "home" => {
            let mut v = 4.0f32;
            let mut thresh = 100.0f32;
            if parts.len() > 1 {
                if let Ok(val) = parts[1].parse::<f32>() {
                    v = val.clamp(0.5, 5.0);
                }
            }
            if parts.len() > 2 {
                if let Ok(val) = parts[2].parse::<f32>() {
                    thresh = val.clamp(20.0, 600.0);
                }
            }
            SHARED_SERVO.lock(|s| {
                let mut s = s.borrow_mut();
                s.homing_voltage = v;
                s.stall_threshold_ma = thresh;
                s.homing_requested = true;
            });
            rprintln!("-> Triggered endpoint stall homing (V={:.2}V, threshold={:.1}mA)", v, thresh);
        }
        "h" | "?" => {
            rprintln!("Available RTT Commands:");
            rprintln!("  home [V] [mA] - Auto-detect hard-stop endpoints via current sense (e.g. 'home 1.5 120')");
            rprintln!("  t <rad>       - Set target angle in radians (e.g. 't 1.57')");
            rprintln!("  d <deg>       - Set target angle in degrees (e.g. 'd 90')");
            rprintln!("  u <volts>     - Drive motor in voltage mode (e.g. 'u 2.5')");
            rprintln!("  m <mode>      - Change mode: 'angle', 'vel', 'volt'");
            rprintln!("  e 1 / 0       - Enable (1) or Disable (0) motor bridge");
            rprintln!("  p <val>       - Set position P_angle gain");
            rprintln!("  v <val>       - Set velocity loop P gain");
            rprintln!("  s             - Print full status");
        }
        _ => {
            rprintln!("Unknown command: '{}'. Type 'h' for help.", parts[0]);
        }
    }
}

// ---------------------------------------------------------------------------
// Handsfree bootloader entry (raw single-byte write [0xF0] to I2C address 0x40)
// ---------------------------------------------------------------------------
// This hook runs in whatever context calls Stm32I2cTarget::service_once. The
// I2C2 target task is spawned on the high-priority InterruptExecutor below, so
// this executes in interrupt context and preempts a starved motor task.
fn bootloader_write_hook(frame: &[u8]) {
    if frame.len() == 1 && frame[0] == 0xF0 {
        barefoc_stm32::request_bootloader();
    }
}

// Dedicated interrupt executor for the I2C2 target task. TIM6 is a basic timer
// that is unused by the application (TIM1/TIM3 drive PWM, TIM2 is the Embassy
// time driver); only its NVIC vector is needed to pend the executor, the
// peripheral itself is never configured or enabled.
static I2C_TARGET_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn TIM6() {
    unsafe { I2C_TARGET_EXECUTOR.on_interrupt() }
}

// ---------------------------------------------------------------------------
// I2C2 Multibus Server Task (Address 0x40)
// ---------------------------------------------------------------------------
#[embassy_executor::task]
async fn i2c_target_task(mut target: Stm32I2cTarget<'static, SharedServoState, 256, 128>) {
    rprintln!("I2C2 Multibus target server listening on address 0x40.");

    loop {
        // Refresh telemetry before servicing host transactions
        let (angle, vel, cur, stat) = SHARED_SERVO.lock(|s| {
            let s = s.borrow();
            (s.current_angle, s.current_velocity, s.current_ma, s.status)
        });

        {
            let map = target.server_mut().map_mut();
            map.handler().current_angle = angle;
            map.handler().current_velocity = vel;
            map.handler().current_ma = cur;
            map.handler().status = stat;
            barefoc_multibus::refresh_telemetry(map);
        }

        // Service bus events if pending
        match target.service_once::<I2C2>() {
            Ok(ServiceOutcome::Idle) => {
                // Bus is idle, yield briefly to avoid hogging executor
                Timer::after_micros(200).await;
            }
            Ok(ServiceOutcome::WritePhase { .. }) => {
                // Host wrote to registers - propagate updated state into SHARED_SERVO
                let handler = *target.server_mut().map_mut().handler();
                SHARED_SERVO.lock(|s| {
                    let mut s = s.borrow_mut();
                    s.target = handler.target;
                    s.enabled = handler.enabled;
                    s.p_angle = handler.p_angle;
                    s.pid_vel_p = handler.pid_vel_p;
                    s.pid_vel_i = handler.pid_vel_i;
                    s.pid_vel_d = handler.pid_vel_d;
                    s.voltage_limit = handler.voltage_limit;
                    s.velocity_limit = handler.velocity_limit;
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
    rprintln!("     N20 Closed-Loop Servo with BareFOC & Multibus    ");
    rprintln!("     Microcontroller: STM32H503KBU6 (Arm Cortex-M33)  ");
    rprintln!("     Bridge D: TIM1_CH2 (PA9) / TIM3_CH3 (PB0)        ");
    rprintln!("     Sensor: TMAG5273 on I2C1 (PB6/PB7)               ");
    rprintln!("     Host Bus: Multibus on I2C2 (PB3/PB4 @ 0x40)      ");
    rprintln!("=======================================================");

    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = Some(embassy_stm32::rcc::HSIPrescaler::Div1);
    config.rcc.sys = embassy_stm32::rcc::Sysclk::Hsi;
    config.rcc.voltage_scale = embassy_stm32::rcc::VoltageScale::Scale3;
    let p = embassy_stm32::init(config);
    let boot_start = Instant::now();

    // -----------------------------------------------------------------------
    // DBGMCU Configuration: Keep CoreSight debug clocks running during low-power
    // sleep (WFE/WFI), stop, and standby modes.
    // On STM32H5, WFE/WFI in the Embassy executor idle loop gates debug clocks
    // by default, disabling SWD immediately on boot unless un-gated here.
    // Also freeze watchdog timers (IWDG, WWDG) when CPU is halted by debugger.
    // -----------------------------------------------------------------------
    embassy_stm32::pac::DBGMCU.cr().modify(|w| {
        w.set_stop(true);
        w.set_standby(true);
    });

    // SAFETY: DBGMCU_CR (0x4402_4004) and DBGMCU_APB1LFZR (0x4402_4008) are valid
    // hardware memory-mapped registers for the STM32H503 microcontroller.
    // Writing bits 0..2 (0x07) sets DBG_SLEEP, DBG_STOP, DBG_STANDBY.
    // Writing bits 11, 12 in APB1LFZR freezes WWDG and IWDG during debug halt.
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
    // Task 3.3: Configure I2C2 Target at address 0x40 running barefoc-multibus
    // This is done BEFORE sensor discovery so the host bus is available while
    // the I2C1 scan is still running. The target hardware ACKs its address as
    // soon as the peripheral is configured, and the target task is spawned now
    // on the high-priority interrupt executor so it is ready before the
    // blocking sensor discovery below begins.
    // -----------------------------------------------------------------------
    let mut i2c2_cfg = I2cConfig::default();
    i2c2_cfg.sda_pullup = true;
    i2c2_cfg.scl_pullup = true;
    i2c2_cfg.frequency = Hertz::khz(100);
    let i2c2 = I2c::new_blocking(p.I2C2, p.PB3, p.PB4, i2c2_cfg);

    let initial_state = SharedServoState::new();
    let multibus_map = barefoc_multibus::simplefoc_profile(initial_state);
    let server = RegServer::new(multibus_map);
    let mut target =
        Stm32I2cTarget::new(i2c2, 0x40, server).expect("I2C2 Target binding failed");
    // Raw single-byte write [0xF0] to address 0x40 requests the BareBoot
    // bootloader. Installed as a raw-frame hook so it is observed before the
    // register server latches 0xF0 as a pointer, and dispatched from the
    // interrupt-context task below so it cannot be starved by the motor loop.
    target.set_write_frame_hook(Some(bootloader_write_hook));

    // Run the I2C2 target on a high-priority interrupt executor so bus events
    // (including the bootloader trigger) are serviced even while the 1 kHz
    // motor task monopolises the cooperative thread executor. TIM6's NVIC
    // vector is only used to pend this executor; set its priority before start.
    interrupt::TIM6.set_priority(Priority::P8);
    let i2c_spawner = I2C_TARGET_EXECUTOR.start(interrupt::TIM6);
    i2c_spawner.spawn(i2c_target_task(target).expect("Failed to create i2c target task token"));

    // Yield once so the target task enters its service loop before the
    // blocking I2C1 discovery below monopolises the executor.
    Timer::after_millis(1).await;
    rprintln!(
        "I2C2 target ready at 0x40 ({} ms after init).",
        boot_start.elapsed().as_millis()
    );

    // -----------------------------------------------------------------------
    // Task 2.1: Safe boot Coast mode on Bridges A, B, C (Outputs LOW)
    // Note: SWD pins PA13 (SWDIO) and PA14 (SWCLK) are strictly preserved
    // and never initialized or disturbed by GPIO configuration.
    // PA15 is JTDI, which is safely available as GPIO output in SWD mode.
    // -----------------------------------------------------------------------
    let _in_a1 = Output::new(p.PA15, Level::Low, Speed::Low);
    let _in_a2 = Output::new(p.PB5, Level::Low, Speed::Low);
    let _in_b1 = Output::new(p.PA8, Level::Low, Speed::Low);
    let _in_b2 = Output::new(p.PB15, Level::Low, Speed::Low);
    let _in_c1 = Output::new(p.PA6, Level::Low, Speed::Low);
    let _in_c2 = Output::new(p.PA7, Level::Low, Speed::Low);

    // Status LEDs on PA0 (Blue) and PA1 (Red): Active LOW
    let mut led_b = Output::new(p.PA0, Level::Low, Speed::Low); // Blue ON to signal booting
    let led_r = Output::new(p.PA1, Level::High, Speed::Low); // Red OFF

    // -----------------------------------------------------------------------
    // Task 2.2: Channel D PWM on TIM1_CH2 (PA9) and TIM3_CH3 (PB0) at 25 kHz
    // -----------------------------------------------------------------------
    let ch2_pin = PwmPin::new(p.PA9, OutputType::PushPull);
    let pwm1 = SimplePwm::new(
        p.TIM1,
        None,
        Some(ch2_pin),
        None,
        None,
        Hertz(25_000),
        CountingMode::EdgeAlignedUp,
    );

    let ch3_pin = PwmPin::new(p.PB0, OutputType::PushPull);
    let pwm3 = SimplePwm::new(
        p.TIM3,
        None,
        None,
        Some(ch3_pin),
        None,
        Hertz(25_000),
        CountingMode::EdgeAlignedUp,
    );

    let mut channels1 = pwm1.split();
    let mut channels3 = pwm3.split();
    channels1.ch2.enable();
    channels3.ch3.enable();

    let mut driver = DCDriver2PWM::without_enable(channels1.ch2, channels3.ch3);
    driver.params_mut().voltage_power_supply = 5.0;
    driver.params_mut().voltage_limit = 5.0;
    driver.params_mut().pwm_frequency = 25_000;
    driver.init().expect("DCDriver2PWM init failed");

    // -----------------------------------------------------------------------
    // Task 2.3: Sensor Bus I2C1 (PB6/PB7) with internal pull-ups
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

    // Only the TMAG5273 factory addresses are probed: 0x35 (A1/A2), 0x22
    // (B1/B2), 0x78 (C1/C2) and 0x44 (D1/D2). The previous 0x03..=0x77 sweep
    // at three speeds spent hundreds of milliseconds in blocking timeouts and
    // starved the I2C2 target task. The speed fallback order (20, 50, 100 kHz)
    // and the variant detection semantics are unchanged.
    let speeds_khz = [20u32, 50u32, 100u32];
    let candidate_addrs = [0x35u8, 0x22, 0x78, 0x44];
    let mut detected_tmag = None;

    for &khz in &speeds_khz {
        set_i2c1_frequency(64_000_000, khz * 1000);
        rprintln!("--- Probing TMAG5273 candidates on I2C1 @ {} kHz ---", khz);
        let mut count = 0usize;
        for &addr in &candidate_addrs {
            let mut b = [0u8; 1];
            // Read-only probe: a blocking write to an address that NACKs waits
            // for the configured timeout, while a read NACKs immediately
            // (hardware AUTOEND STOP). This keeps discovery short.
            let ack = i2c1.blocking_read(addr, &mut b).is_ok();
            if ack {
                count += 1;
                rprintln!("  [+] Device ACK at 7-bit 0x{:02X} (8-bit: 0x{:02X})", addr, addr << 1);

                let mut mfg = [0u8; 2];
                if i2c1.blocking_write_read(addr, &[0x0E], &mut mfg).is_ok() {
                    let mfg_id = u16::from_le_bytes(mfg);
                    rprintln!("      Reg 0x0E-0x0F (Mfg ID): 0x{:04X} ([0x{:02X}, 0x{:02X}])", mfg_id, mfg[0], mfg[1]);
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
                        rprintln!("      -> Identified TI TMAG5273 ({:?}) at 0x{:02X}", variant, addr);
                        if detected_tmag.is_none() {
                            detected_tmag = Some((addr, variant));
                        }
                    }
                }
                let mut who_am_i = [0u8; 1];
                if i2c1.blocking_write_read(addr, &[0x75], &mut who_am_i).is_ok() {
                    rprintln!("      Reg 0x75 (WHO_AM_I):    0x{:02X}", who_am_i[0]);
                }
                let mut id0 = [0u8; 1];
                if i2c1.blocking_write_read(addr, &[0x00], &mut id0).is_ok() {
                    rprintln!("      Reg 0x00 (CHIP_ID):     0x{:02X}", id0[0]);
                }
                let mut reg_dump = [0u8; 16];
                if i2c1.blocking_write_read(addr, &[0x00], &mut reg_dump).is_ok() {
                    rprintln!("      Regs 0x00..0x0F:        {:02X?}", reg_dump);
                }
            }
        }
        if count == 0 {
            rprintln!("  [-] No TMAG5273 candidates responded on I2C1 at {} kHz.", khz);
        }
        if detected_tmag.is_some() {
            // Higher speeds are only a fallback for sensors that fail at the
            // lower speed; stop at the first speed that identifies a device.
            break;
        }
    }

    // Let the I2C2 target task service any host transaction that arrived while
    // discovery was blocking the executor.
    Timer::after_millis(1).await;

    set_i2c1_frequency(64_000_000, 50_000);
    rprintln!("=======================================================\n");

    let (sensor, has_sensor) = if let Some((addr, variant)) = detected_tmag {
        match Tmag5273::builder(i2c1, addr)
            .variant(variant)
            .operating_mode(OperatingMode::ContinuousMeasure)
            .channels(MagneticChannel::XYZ)
            .angle_enabled(AngleEnable::XY)
            .build()
        {
            Ok(dev) => {
                rprintln!("TMAG5273 Sensor initialized successfully on I2C1 (Continuous Measure, XY Angle enabled).");
                (N20Sensor::Tmag(Tmag5273Sensor::new(dev)), true)
            }
            Err(e) => {
                rprintln!("WARN: TMAG5273 init failed: {:?}. Fallback to dummy sensor.", e);
                (N20Sensor::Fallback(barefoc_drivers::dc::DummySensor::new()), false)
            }
        }
    } else {
        rprintln!("NOTICE: No TMAG5273 detected on I2C1. Fallback to dummy sensor.");
        (N20Sensor::Fallback(barefoc_drivers::dc::DummySensor::new()), false)
    };

    TMAG_DETECTED.store(has_sensor, Ordering::Relaxed);
    if has_sensor {
        rprintln!("TMAG5273 active! Real-time angle sensing enabled. LED blinking stopped (Solid Blue ON).");
    }

    // -----------------------------------------------------------------------
    // Task 2.4: ADC1 on PA5 (ISENS_D) with CurrentMirrorSense
    // -----------------------------------------------------------------------
    let adc1 = Adc::new_blocking(p.ADC1, embassy_stm32::adc::Config::default());
    let adc_pa5 = Stm32AdcPa5::new(adc1, p.PA5, SampleTime::Cycles65);
    let mut current_sense = CurrentMirrorSense::new(adc_pa5, 200.0, 1000.0, 3.3, 12);
    current_sense.calibrate_offsets_rounds(&CLOCK, 500);
    rprintln!(
        "CurrentMirrorSense calibrated: offset count = {}",
        current_sense.offset_count()
    );

    // -----------------------------------------------------------------------
    // Task 3.1: Initialize DCMotor with TMAG5273 Sensor
    // -----------------------------------------------------------------------
    let mut motor = DCMotor::new(&CLOCK);
    motor.link_driver(driver);
    motor.link_sensor(sensor);
    motor.link_current_sense(current_sense);

    motor.sensor_direction = Direction::CCW;
    motor.sensor_offset = -4.095; // default center from open-loop stroke calibration
    motor.controller = if has_sensor {
        MotionControlType::angle
    } else {
        MotionControlType::velocity_openloop
    };
    motor.torque_controller = TorqueControlType::voltage;

    motor.voltage_limit = 4.0; // 4.0 V max for micro N20 motor
    motor.velocity_limit = 5.0; // 5.0 rad/s (~286 deg/s) max
    motor.current_limit = 0.5; // 500 mA limit

    // Position servo PID loop tuning
    motor.P_angle.set_p(15.0);
    motor.P_angle.set_limit(5.0);

    // Velocity sub-loop PID tuning
    motor.PID_velocity.set_p(1.2);
    motor.PID_velocity.set_i(4.0);
    motor.PID_velocity.set_d(0.0005);
    motor.PID_velocity.set_limit(4.0);
    motor.PID_velocity.set_output_ramp(500.0);
    motor.LPF_velocity.set_tf(0.015);

    SHARED_SERVO.lock(|s| {
        let mut s = s.borrow_mut();
        s.controller_mode = if has_sensor { 0 } else { 1 };
        s.target = 0.0;
        s.enabled = false;
        s.p_angle = 15.0;
        s.pid_vel_p = 1.2;
        s.pid_vel_i = 4.0;
        s.pid_vel_d = 0.0005;
        s.voltage_limit = 4.0;
        s.velocity_limit = 5.0;
    });

    if has_sensor {
        rprintln!("DCMotor initialized in CLOSED-LOOP ANGLE mode (Direction::CCW, center 0.0 rad).");
    } else {
        rprintln!("NOTICE: No TMAG sensor - falling back to velocity_openloop mode.");
    }

    let init_res = motor.init();
    rprintln!("DCMotor init returned: {}", init_res);

    // Signal boot completion: Blue LED OFF
    led_b.set_high();
    rprintln!(
        "Boot complete: sensor={}, host bus 0x40 ({} ms after init).",
        has_sensor,
        boot_start.elapsed().as_millis()
    );

    // -----------------------------------------------------------------------
    // Spawn Tasks
    // -----------------------------------------------------------------------
    spawner.spawn(motor_task(motor, led_b, led_r, has_sensor, rtt_down).expect("Failed to create motor task token"));

    rprintln!("System running.");
}
