use std::ops::Range;

use esp_idf_svc::hal::adc::config::Config;
use esp_idf_svc::hal::adc::{attenuation, Adc, AdcChannelDriver, AdcDriver};
use esp_idf_svc::hal::gpio::ADCPin;
use esp_idf_svc::hal::peripheral::Peripheral;
use log::{debug, info};

use crate::util;

pub struct GamepadConfig {
    // Min value of joystick.
    joystick_min_value: u32,
    /// Max value of joystick.
    joystick_max_value: u32,

    /// Offset from center position.
    center_offset: u32,

    /// If set to true, then real center position will be read from the joystick at the start.
    use_real_center: bool,
}

impl GamepadConfig {
    pub fn new(center_offset: u32, joystick_max_value: u32) -> Self {
        Self {
            joystick_min_value: 10,
            center_offset,
            joystick_max_value,
            use_real_center: true,
        }
    }

    fn center_range(&self, val: u32) -> Range<u32> {
        val - self.center_offset..val + self.center_offset
    }
}

impl Default for GamepadConfig {
    fn default() -> Self {
        Self::new(50, 2757)
    }
}

pub trait Gamepad {
    /// Returns raw values of joystick.
    fn read_raw_state(&mut self) -> eyre::Result<RawState>;

    /// Returns state of joystick mapped to the specified output range.
    fn read_state(&mut self, output: &Range<u32>) -> eyre::Result<State>;
}

#[derive(Debug, Clone, Default)]
pub struct RawState {
    pub base_rotator: u32,
    pub shoulder: u32,
    pub elbow: u32,
    pub gripper: u32,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct State {
    pub base_rotator: Position,
    pub shoulder: Position,
    pub elbow: Position,
    pub gripper: Position,
}

impl State {
    pub fn is_center(&self) -> bool {
        self.base_rotator == Position::Center
            && self.shoulder == Position::Center
            && self.elbow == Position::Center
            && self.gripper == Position::Center
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub enum Position {
    Low(u32),
    #[default]
    Center,
    High(u32),
}

impl Position {
    fn new(
        val: u32,
        config: &GamepadConfig,
        center_range: &Range<u32>,
        output: &Range<u32>,
    ) -> Self {
        if center_range.contains(&val) {
            Position::Center
        } else if val < center_range.start {
            let val = util::map(
                val,
                config.joystick_min_value,
                center_range.start,
                output.start,
                output.end,
                true,
            );
            Position::Low(val)
        } else {
            let val = util::map(
                val,
                center_range.end,
                config.joystick_max_value,
                output.start,
                output.end,
                false,
            );
            Position::High(val)
        }
    }
}

pub struct GamepadImpl<'d, ADC: Adc, P0: ADCPin, P1: ADCPin, P2: ADCPin, P3: ADCPin> {
    config: GamepadConfig,
    adc: AdcDriver<'d, ADC>,

    base_rotator: AdcChannelDriver<'d, { attenuation::DB_11 }, P0>,
    shoulder: AdcChannelDriver<'d, { attenuation::DB_11 }, P1>,
    elbow: AdcChannelDriver<'d, { attenuation::DB_11 }, P2>,
    gripper: AdcChannelDriver<'d, { attenuation::DB_11 }, P3>,

    base_rotator_center: Range<u32>,
    shoulder_center: Range<u32>,
    elbow_center: Range<u32>,
    gripper_center: Range<u32>,
}

impl<'d, ADC: Adc, P0: ADCPin, P1: ADCPin, P2: ADCPin, P3: ADCPin>
    GamepadImpl<'d, ADC, P0, P1, P2, P3>
where
    ADC: Adc,
    P0: ADCPin<Adc = ADC>,
    P1: ADCPin<Adc = ADC>,
    P2: ADCPin<Adc = ADC>,
    P3: ADCPin<Adc = ADC>,
{
    pub fn new(
        config: GamepadConfig,
        adc: impl Peripheral<P = ADC> + 'd,
        base_rotator_pin: P0,
        shoulder_pin: P1,
        elbow_pin: P2,
        gripper_pin: P3,
    ) -> eyre::Result<Self> {
        let adc = AdcDriver::new(adc, &Config::new())?;

        let base_rotator: AdcChannelDriver<{ attenuation::DB_11 }, _> =
            AdcChannelDriver::new(base_rotator_pin)?;

        let shoulder: AdcChannelDriver<{ attenuation::DB_11 }, _> =
            AdcChannelDriver::new(shoulder_pin)?;

        let elbow: AdcChannelDriver<{ attenuation::DB_11 }, _> = AdcChannelDriver::new(elbow_pin)?;

        let gripper: AdcChannelDriver<{ attenuation::DB_11 }, _> =
            AdcChannelDriver::new(gripper_pin)?;

        let default_center_range = config.center_range(config.joystick_max_value / 2);
        let mut gamepad = Self {
            config,
            adc,
            base_rotator,
            shoulder,
            elbow,
            gripper,
            base_rotator_center: default_center_range.clone(),
            shoulder_center: default_center_range.clone(),
            elbow_center: default_center_range.clone(),
            gripper_center: default_center_range.clone(),
        };

        if gamepad.config.use_real_center {
            // read and store center position
            let real_positions = gamepad.read_raw_state()?;
            gamepad.base_rotator_center = gamepad.config.center_range(real_positions.base_rotator);
            gamepad.shoulder_center = gamepad.config.center_range(real_positions.shoulder);
            gamepad.elbow_center = gamepad.config.center_range(real_positions.elbow);
            gamepad.gripper_center = gamepad.config.center_range(real_positions.gripper);
        }
        info!("base_rotator center={:?}", gamepad.base_rotator_center);
        info!("shoulder center={:?}", gamepad.shoulder_center);
        info!("elbow center={:?}", gamepad.elbow_center);
        info!("gripper center={:?}", gamepad.gripper_center);

        Ok(gamepad)
    }
}

impl<'d, ADC, P0, P1, P2, P3> Gamepad for GamepadImpl<'d, ADC, P0, P1, P2, P3>
where
    ADC: Adc,
    P0: ADCPin<Adc = ADC>,
    P1: ADCPin<Adc = ADC>,
    P2: ADCPin<Adc = ADC>,
    P3: ADCPin<Adc = ADC>,
{
    fn read_raw_state(&mut self) -> eyre::Result<RawState> {
        let base_rotator_angle = self.adc.read(&mut self.base_rotator)? as u32;
        let shoulder_angle = self.adc.read(&mut self.shoulder)? as u32;
        let elbow_angle = self.adc.read(&mut self.elbow)? as u32;
        let gripper_angle = self.adc.read(&mut self.gripper)? as u32;

        fn normalize_value(val: u32, config: &GamepadConfig) -> u32 {
            val.min(config.joystick_max_value)
                .max(config.joystick_min_value)
        }

        let state = RawState {
            base_rotator: normalize_value(base_rotator_angle, &self.config),
            shoulder: normalize_value(shoulder_angle, &self.config),
            elbow: normalize_value(elbow_angle, &self.config),
            gripper: normalize_value(gripper_angle, &self.config),
        };
        debug!("raw state = {:?}", state);
        Ok(state)
    }

    fn read_state(&mut self, output: &Range<u32>) -> eyre::Result<State> {
        let state = self.read_raw_state()?;
        let state = State {
            base_rotator: Position::new(
                state.base_rotator,
                &self.config,
                &self.base_rotator_center,
                output,
            ),
            shoulder: Position::new(state.shoulder, &self.config, &self.shoulder_center, output),
            elbow: Position::new(state.elbow, &self.config, &self.elbow_center, output),
            gripper: Position::new(state.gripper, &self.config, &self.gripper_center, output),
        };
        debug!("state = {:?}", state);
        Ok(state)
    }
}
