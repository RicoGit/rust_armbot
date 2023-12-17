use crate::command::{Cmd, CmdMappingConfig};
use crate::gamepad::{Gamepad, State};
use esp_idf_svc::hal::delay::Delay;
use std::ops::Range;

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
        let state = self.gamepad.read_state()?;
        if state.is_none() {
            return Ok(());
        }

        let new_state = state.unwrap();

        let now = self.get_now_ms();
        let shoulder_cmd =
            Cmd::from_joystick_value(new_state.shoulder_angle, &self.config.cmd_mapping_config);
        Self::make_step(shoulder_cmd, &mut self.shoulder_servo, |delay_ms| {
            self.last_ts + delay_ms > now
        });

        self.delay.delay_us(1);
        Ok(())
    }

    pub fn make_step(cmd: Cmd, servo: &mut Servo<'d>, its_time: impl Fn(u32) -> bool) {
        match cmd {
            Cmd::Stop => {
                // do noting
            }
            Cmd::Forward { delay_ms } => {
                if its_time(delay_ms) {
                    servo.dir(true);
                    servo.step();
                }
            }
            Cmd::Backward { delay_ms } => {
                if its_time(delay_ms) {
                    servo.dir(false);
                    servo.step();
                }
            }
        }
    }

    fn get_now_ms(&self) -> u32 {
        self.last_ts + self.config.cmd_mapping_config.min_delay_ms
    }
}

pub struct ArmBotConfig {
    /// Desirable range of the shoulder angle.
    pub shoulder_angle_range: Range<usize>,
    /// Desirable range of the elbow angle.
    pub elbow_angle_range: Range<usize>,
    /// Desirable range of the gripper angle.
    pub gripper_angle_range: Range<usize>,
    /// Mapping describes joystick and.
    pub cmd_mapping_config: CmdMappingConfig,
}

impl Default for ArmBotConfig {
    fn default() -> Self {
        Self {
            shoulder_angle_range: 30..150,
            elbow_angle_range: 30..150,
            gripper_angle_range: 20..70,
            cmd_mapping_config: Default::default(),
        }
    }
}
