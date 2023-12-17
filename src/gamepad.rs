use esp_idf_svc::hal::adc::config::Config;
use esp_idf_svc::hal::adc::{attenuation, Adc, AdcChannelDriver, AdcDriver, ADC1};
use esp_idf_svc::hal::gpio::ADCPin;
use esp_idf_svc::hal::peripheral::Peripheral;
use std::ops::Range;

pub trait Gamepad {
    /// Returns true if the state has changed.
    fn read_state(&mut self) -> eyre::Result<Option<State>>;
}

impl<'d, ADC: Adc, P0: ADCPin, P1: ADCPin, P2: ADCPin, P3: ADCPin> Gamepad
    for GamepadImpl<'d, ADC, P0, P1, P2, P3>
{
    fn read_state(&mut self) -> eyre::Result<Option<State>> {
        // todo impl
        Ok(Default::default())
    }
}

#[derive(Debug, Clone, Default)]
pub struct State {
    pub base_rotator_angle: usize,
    pub elbow_angle: usize,
    pub shoulder_angle: usize,
    pub gripper_angle: usize,
}

pub struct GamepadImpl<'d, ADC: Adc, P0: ADCPin, P1: ADCPin, P2: ADCPin, P3: ADCPin> {
    adc: AdcDriver<'d, ADC>,
    base_rotator: AdcChannelDriver<'d, { attenuation::DB_11 }, P0>,
    shoulder: AdcChannelDriver<'d, { attenuation::DB_11 }, P1>,
    elbow: AdcChannelDriver<'d, { attenuation::DB_11 }, P2>,
    gripper: AdcChannelDriver<'d, { attenuation::DB_11 }, P3>,

    pub state: State,
}

impl<'d, ADC: Adc, P0: ADCPin, P1: ADCPin, P2: ADCPin, P3: ADCPin>
    GamepadImpl<'d, ADC, P0, P1, P2, P3>
{
    pub fn new(
        adc: impl Peripheral<P = ADC> + 'd,
        base_rotator_pin: P0,
        shoulder_pin: P1,
        elbow_pin: P2,
        gripper_pin: P3,
    ) -> eyre::Result<Self> {
        let adc = AdcDriver::new(adc, &Config::new().calibration(true))?;

        let base_rotator: AdcChannelDriver<{ attenuation::DB_11 }, _> =
            AdcChannelDriver::new(base_rotator_pin)?;

        let shoulder: AdcChannelDriver<{ attenuation::DB_11 }, _> =
            AdcChannelDriver::new(shoulder_pin)?;

        let elbow: AdcChannelDriver<{ attenuation::DB_11 }, _> = AdcChannelDriver::new(elbow_pin)?;

        let gripper: AdcChannelDriver<{ attenuation::DB_11 }, _> =
            AdcChannelDriver::new(gripper_pin)?;

        Ok(Self {
            adc,
            base_rotator,
            shoulder,
            elbow,
            gripper,

            state: Default::default(),
        })
    }
}
