use esp_idf_svc::hal::ledc;
use esp_idf_svc::hal::peripherals::Peripherals;
use std::thread;
use std::time::Duration;

use crate::armbot::{ArmBot, ArmBotConfig};
use crate::gamepad::{GamepadConfig, GamepadImpl};
use crate::ledc_servo::{Servo, ServoConfig};

mod armbot;
mod gamepad;
mod ledc_servo;
mod util;

fn main() -> eyre::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let gamepad = GamepadImpl::new(
        GamepadConfig {
            center_offset: 100,
            .. GamepadConfig::default()
        },
        peripherals.adc1,
        peripherals.pins.gpio0,
        peripherals.pins.gpio1,
        peripherals.pins.gpio2,
        peripherals.pins.gpio3,
    )
    .expect("gamepad init failed");

    let servo_cfg = ServoConfig::sg90(ledc::SpeedMode::LowSpeed);

    let shoulder_servo = Servo::new(
        "shoulder",
        servo_cfg.clone(),
        peripherals.ledc.timer0,
        peripherals.ledc.channel0,
        peripherals.pins.gpio5,
    )
    .expect("shoulder init failed");

    let elbow_servo = Servo::new(
        "elbow",
        servo_cfg.clone(),
        peripherals.ledc.timer1,
        peripherals.ledc.channel1,
        peripherals.pins.gpio6,
    )
    .expect("elbow init failed");

    let gripper_servo = Servo::new(
        "gripper",
        servo_cfg,
        peripherals.ledc.timer2,
        peripherals.ledc.channel2,
        peripherals.pins.gpio7,
    )
    .expect("gripper init failed");

    let mut bot = ArmBot::new(
        ArmBotConfig::default(),
        gamepad,
        shoulder_servo,
        elbow_servo,
        gripper_servo,
    )
    .expect("ArmBot init failed");

    log::info!("Arm bot initialized");

    loop {
        // do step blocks until the step is done
        bot.do_step().unwrap();
        thread::sleep(Duration::from_millis(10)); // todo remove
    }
}
