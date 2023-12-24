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
        })
    }

    /// Makes the arm bot do a cycle of its movement.
    pub fn do_step(&mut self) -> eyre::Result<()> {
        let state = self.gamepad.read_state(&self.config.step_size)?;
        if state.is_center() {
            // noting to do
            return Ok(());
        }

        // todo make step of base_rotator
        Self::make_step(&state.shoulder, &mut self.shoulder_servo)?;
        Self::make_step(&state.elbow, &mut self.elbow_servo)?;
        Self::make_step(&state.gripper, &mut self.gripper_servo)?;

        Ok(())
    }

    pub fn make_step(cmd: &Position, servo: &mut Servo<'d>) -> eyre::Result<()> {
        match cmd {
            Position::Center => {
                // do noting
            }
            Position::Low(step) => {
                let _ = servo.dir(true);
                servo.step(*step)?;
            }
            Position::High(step) => {
                let _ = servo.dir(false);
                servo.step(*step)?;
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

    /// Min possible step, for slowest motion.
    /// Max possible step, for fastest motion.
    pub step_size: Range<u32>,
}

impl Default for ArmBotConfig {
    fn default() -> Self {
        Self {
            shoulder_angle_range: 30..150,
            elbow_angle_range: 30..150,
            gripper_angle_range: 20..70,
            step_size: 1..10,
        }
    }
}
