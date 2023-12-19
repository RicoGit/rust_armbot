use esp_idf_svc::hal::delay::Delay;
use std::ops::Range;

use crate::gamepad::{Gamepad, Position};
use crate::ledc_servo::Servo;

pub struct ArmBot<'d, G> {
    config: ArmBotConfig,
    delay: Delay,

    // pub base: Motor,
    shoulder_servo: Servo<'d>,
    elbow_servo: Servo<'d>,
    gripper_servo: Servo<'d>,

    gamepad: G,
    elbow_angle: f64,
    shoulder_angle: f64,
    gripper_angle: f64,

    /// Last step time.
    last_ts: u32,
}

impl<'d, G: Gamepad> ArmBot<'d, G> {
    pub fn new(
        config: ArmBotConfig,
        gamepad: G,
        shoulder_servo: Servo<'d>,
        elbow_servo: Servo<'d>,
        gripper_servo: Servo<'d>,
    ) -> eyre::Result<Self> {
        Ok(Self {
            config,
            delay: Delay::new(10_000),

            shoulder_servo,
            elbow_servo,
            gripper_servo,

            gamepad,
            elbow_angle: 0.0,
            shoulder_angle: 0.0,
            gripper_angle: 0.0,
            last_ts: 0,
        })
    }

    /// Makes the arm bot do a cycle of its movement.
    pub fn do_step(&mut self) -> eyre::Result<()> {
        // add min delay and max delay
        let _state = self.gamepad.read_state(&self.config.delay_range_ms)?;

        // todo finish
        self.delay.delay_ms(2000);

        Ok(())
    }

    pub fn make_step(
        cmd: Position,
        servo: &mut Servo<'d>,
        its_time: impl Fn(u32) -> bool,
    ) -> eyre::Result<()> {
        match cmd {
            Position::Center => {
                // do noting
            }
            Position::Low(delay_ms) => {
                if its_time(delay_ms) {
                    let _ = servo.dir(true);
                    servo.step()?;
                }
            }
            Position::High(delay_ms) => {
                if its_time(delay_ms) {
                    let _ = servo.dir(false);
                    servo.step()?;
                }
            }
        }

        Ok(())
    }
}

pub struct ArmBotConfig {
    /// Desirable range of the shoulder angle.
    pub shoulder_angle_range: Range<usize>,
    /// Desirable range of the elbow angle.
    pub elbow_angle_range: Range<usize>,
    /// Desirable range of the gripper angle.
    pub gripper_angle_range: Range<usize>,

    /// Min possible delay, for fastest motion.
    /// Max possible delay, for slowest motion.
    pub delay_range_ms: Range<u32>,
}

impl Default for ArmBotConfig {
    fn default() -> Self {
        Self {
            shoulder_angle_range: 30..150,
            elbow_angle_range: 30..150,
            gripper_angle_range: 20..70,
            delay_range_ms: 1..20,
        }
    }
}
