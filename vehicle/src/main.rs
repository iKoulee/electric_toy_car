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

    esp_println::println!("Main Vehicle Board initialized!");

    // TODO: Initialize PWM for H-Bridge motor control
    // TODO: Initialize communication (e.g. ESP-NOW) to receive instructions

    loop {
        // Main drive loop listening to commands and handling fail-safes
        esp_println::println!("Checking connection state and updating motors...");
        delay.delay(Duration::from_millis(500));
    }
}
