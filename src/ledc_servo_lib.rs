//! This is a small lib for controlling servo using LEDC.

#![allow(unused)]

use esp_idf_svc::hal::gpio::OutputPin;
use esp_idf_svc::hal::ledc;
use esp_idf_svc::hal::ledc::{LedcChannel, LedcTimer};
use esp_idf_svc::hal::peripheral::Peripheral;
use esp_idf_svc::hal::prelude::{FromValueType, Hertz};
use esp_idf_svc::sys::EspError;
use std::marker::PhantomData;
use std::ops::Range;
use std::time::Duration;

#[derive(Debug, Clone)]
pub struct ServoConfig {
    /// Max angle that servo can't be turned, mostly 180, 360.
    pub max_angle: f64,
    /// What frequency expect servo (ex. 50Hz for SG90).
    pub frequency: Hertz,
    /// What pulse width servo supports (ex. 500-2400ns for SG90).
    pub pulse_width_ns: Range<u32>,
    /// PWM resolution in bits.
    pub resolution: ledc::Resolution,
    /// ESP32 supports High Speed Mode.
    /// ESP32S2, ESP32S3, ESP32C2 and ESP32C3 supports Low Speed Mode.
    pub speed_mode: ledc::SpeedMode,
}

impl ServoConfig {
    /// Config for [SG90](https://components101.com/motors/servo-motor-basics-pinout-datasheet).
    pub fn sg90(speed_mode: ledc::SpeedMode) -> Self {
        ServoConfig {
            max_angle: 180.0,
            frequency: 50.Hz(),
            pulse_width_ns: 500..2400,
            speed_mode,
            resolution: ledc::Resolution::Bits12,
        }
    }

    /// Config for [SG90S](https://components101.com/motors/mg90s-metal-gear-servo-motor).
    fn sg90s(speed_mode: ledc::SpeedMode) -> Self {
        Self::sg90(speed_mode)
    }
}

pub struct Servo<'d> {
    pub ledc_driver: ledc::LedcDriver<'d>,
    config: ServoConfig,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> Servo<'d> {
    pub fn new<T: LedcTimer, C: LedcChannel, P: OutputPin>(
        config: ServoConfig,
        timer: impl Peripheral<P = T> + 'd,
        channel: impl Peripheral<P = C> + 'd,
        pin: impl Peripheral<P = P> + 'd,
    ) -> Result<Servo<'d>, EspError> {
        let timer_config = ledc::config::TimerConfig::default()
            .resolution(config.resolution)
            .speed_mode(config.speed_mode)
            .frequency(config.frequency);

        let timer_driver = ledc::LedcTimerDriver::new(timer, &timer_config)?;

        let ledc_driver = ledc::LedcDriver::new(channel, timer_driver, pin)?;

        Ok(Servo {
            ledc_driver,
            config,
            _p: PhantomData,
        })
    }

    pub fn get_angle(&self) -> f64 {
        let max_duty = self.ledc_driver.get_max_duty();
        let current_duty = self.ledc_driver.get_duty();
        calculate_angle(&self.config, current_duty, max_duty)
    }

    pub fn set_angle(&mut self, angle: f64) -> Result<(), EspError> {
        let max_duty = self.ledc_driver.get_max_duty();
        let angle = angle.min(self.config.max_angle).max(0.0);
        let duty = calculate_duty(
            &self.config,
            f64::min(angle, self.config.max_angle),
            max_duty,
        );
        self.ledc_driver.set_duty(duty);
        self.ledc_driver.enable()
    }
}

const NANOS_IS_SEC: f64 = 1_000_000.0;

/// Transforms 'angle' to 'duty' in respect that given servo pulse range.
fn calculate_duty(config: &ServoConfig, angle: f64, max_duty: u32) -> u32 {
    let pulse_ns = angle / config.max_angle
        * (config.pulse_width_ns.end - config.pulse_width_ns.start) as f64
        + (config.pulse_width_ns.start as f64);

    let duty = pulse_ns * max_duty as f64 * config.frequency.0 as f64 / NANOS_IS_SEC;
    duty as u32
}

/// Transforms 'duty' to 'angle' in respect that given servo pulse range.
fn calculate_angle(config: &ServoConfig, duty: u32, max_duty: u32) -> f64 {
    let pulse_ns = (duty as f64 * NANOS_IS_SEC / config.frequency.0 as f64 / max_duty as f64);

    (pulse_ns - config.pulse_width_ns.start as f64)
        / (config.pulse_width_ns.end - config.pulse_width_ns.start) as f64
        * config.max_angle
}

#[cfg(test)]
pub mod tests {
    use crate::ledc_servo_lib::{calculate_duty, Servo, ServoConfig};
    use esp_idf_svc::hal::ledc::SpeedMode;

    #[test]
    fn calculate_duty_test() {
        let config = ServoConfig::sg90(SpeedMode::LowSpeed);
        assert_eq!(calculate_duty(&config, 0.0, 1023), 25);
        // todo tests
    }
}
