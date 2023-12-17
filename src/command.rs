//! Describes a command to servo or stepper motor to move.

use std::ops::Range;

#[derive(Debug)]
pub enum Cmd {
    Stop,
    Forward {
        /// Delay between steps in milliseconds.
        delay_ms: u32,
    },
    Backward {
        /// Delay between steps in milliseconds.
        delay_ms: u32,
    },
}

impl Cmd {
    pub fn from_joystick_value(joystick_value: u32, config: &CmdMappingConfig) -> Self {
        let res = match joystick_value {
            val if config.joystick_center_range.contains(&val) => Cmd::Stop,
            val if val <= config.joystick_center_range.start => {
                let delay_ms = config.value_to_delay(val as f64, false);
                Cmd::Forward { delay_ms }
            }
            val if val > config.joystick_center_range.end => {
                let delay_ms = config.value_to_delay(val as f64, true);
                Cmd::Backward { delay_ms }
            }
            _ => panic!("joystick value is out of range: {}", joystick_value),
        };
        log::debug!("cmd =  {res:?} for {joystick_value}");
        res
    }
}

pub struct CmdMappingConfig {
    // Min value of joystick.
    joystick_min_value: u32,
    /// Interval of joystick values that considered as center.
    joystick_center_range: Range<u32>,
    /// Max value of joystick.
    joystick_max_value: u32,

    /// Min possible delay, for fastest motion.
    pub min_delay_ms: u32,
    /// Max possible delay, for slowest motion.
    pub max_delay_ms: u32,

    /// Coefficient delay to joystick value
    pub ratio: f64,
}

impl CmdMappingConfig {
    pub fn new(delta: u32, joystick_max: u32, min_delay_ms: u32, max_delay_ms: u32) -> Self {
        let center = joystick_max / 2;
        let joystick_center_range = center - delta..center + delta;
        Self {
            joystick_min_value: 0,
            joystick_center_range,
            joystick_max_value: joystick_max,
            min_delay_ms,
            max_delay_ms,
            ratio: max_delay_ms as f64 / joystick_max as f64,
        }
    }
}

impl Default for CmdMappingConfig {
    fn default() -> Self {
        Self::new(15, 2800, 1, 20)
    }
}

impl CmdMappingConfig {
    /// Converts joystick value to the delay in milliseconds.
    /// The more the joystick is deflected, the faster the motor moves.
    fn value_to_delay(&self, joystick_value: f64, invert: bool) -> u32 {
        if invert {
            (self.max_delay_ms - (joystick_value * self.ratio) as u32).max(self.min_delay_ms)
        } else {
            ((joystick_value * self.ratio) as u32).max(self.min_delay_ms)
        }
    }
}
