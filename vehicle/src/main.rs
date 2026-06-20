#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    rmt::Rmt,
    time::Duration,
    time::Rate,
};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();
    let mut led_toggle = false;

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("Failed to initialize RMT");
    let mut led = common_led::new_ws2812::<_, _, { common_led::LED_BUFFER_SIZE }>(
        rmt.channel0,
        peripherals.GPIO8,
    )
    .expect("Failed to initialize WS2812B LED");

    if let Err(error) = common_led::set_rgb(&mut led, 16, 16, 0) {
        esp_println::println!("Failed to set vehicle boot LED color: {:?}", error);
    }

    esp_println::println!("Main Vehicle Board initialized!");

    // TODO: Initialize PWM for H-Bridge motor control
    // TODO: Initialize communication (e.g. ESP-NOW) to receive instructions

    loop {
        // Main drive loop listening to commands and handling fail-safes
        esp_println::println!("Checking connection state and updating motors...");

        let color = if led_toggle { (0, 0, 16) } else { (16, 0, 0) };
        led_toggle = !led_toggle;

        if let Err(error) = common_led::set_rgb(&mut led, color.0, color.1, color.2) {
            esp_println::println!("Failed to update vehicle LED color: {:?}", error);
        }

        delay.delay(Duration::from_millis(500));
    }
}
