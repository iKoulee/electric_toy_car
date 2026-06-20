#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    time::Duration,
};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    esp_println::println!("Controller Board initialized!");

    // TODO: Initialize I2C for mini-joystick
    // TODO: Initialize communication (e.g. ESP-NOW) to connect with the vehicle

    loop {
        // Main control loop processing input
        esp_println::println!("Reading joystick state...");
        delay.delay(Duration::from_millis(500));
    }
}
