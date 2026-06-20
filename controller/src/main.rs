#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    i2c::master::{
        AcknowledgeCheckFailedReason,
        Config as I2cConfig,
        Error as I2cError,
        I2c,
    },
    rmt::Rmt,
    time::Duration,
    time::Rate,
};

esp_bootloader_esp_idf::esp_app_desc!();

const I2C_SCAN_START_ADDR: u8 = 0x08;
const I2C_SCAN_END_ADDR: u8 = 0x77;

fn device_responded(i2c: &mut I2c<'_, esp_hal::Blocking>, address: u8) -> Result<bool, I2cError> {
    let mut probe = [0u8; 1];

    match i2c.read(address, &mut probe) {
        Ok(()) => Ok(true),
        Err(I2cError::AcknowledgeCheckFailed(AcknowledgeCheckFailedReason::Address)) => Ok(false),
        Err(I2cError::AcknowledgeCheckFailed(AcknowledgeCheckFailedReason::Unknown)) => Ok(false),
        Err(I2cError::AcknowledgeCheckFailed(AcknowledgeCheckFailedReason::Data)) => Ok(true),
        Err(error) => Err(error),
    }
}

fn scan_i2c_bus(i2c: &mut I2c<'_, esp_hal::Blocking>) {
    esp_println::println!(
        "Scanning I2C bus (0x{:02X}..=0x{:02X})...",
        I2C_SCAN_START_ADDR,
        I2C_SCAN_END_ADDR
    );

    let mut found_any = false;

    for address in I2C_SCAN_START_ADDR..=I2C_SCAN_END_ADDR {
        match device_responded(i2c, address) {
            Ok(true) => {
                found_any = true;
                esp_println::println!("I2C device found at 0x{:02X}", address);
            }
            Ok(false) => {}
            Err(error) => {
                esp_println::println!(
                    "I2C probe error at 0x{:02X}: {}",
                    address,
                    error
                );
            }
        }
    }

    if !found_any {
        esp_println::println!("No I2C devices found on bus.");
    }
}

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();
    let mut led_toggle = false;

    esp_println::println!("Controller Board initialized!");

    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(100));
    let mut i2c = I2c::new(peripherals.I2C0, i2c_config)
        .expect("Failed to initialize I2C0")
        .with_sda(peripherals.GPIO6)
        .with_scl(peripherals.GPIO7);

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("Failed to initialize RMT");
    let mut led = common_led::new_ws2812::<_, _, { common_led::LED_BUFFER_SIZE }>(
        rmt.channel0,
        peripherals.GPIO8,
    )
    .expect("Failed to initialize WS2812B LED");

    if let Err(error) = common_led::set_rgb(&mut led, 0, 16, 0) {
        esp_println::println!("Failed to set controller boot LED color: {:?}", error);
    }

    scan_i2c_bus(&mut i2c);

    // TODO: Initialize communication (e.g. ESP-NOW) to connect with the vehicle

    loop {
        // Main control loop processing input
        esp_println::println!("Reading joystick state...");

        let color = if led_toggle { (0, 0, 16) } else { (16, 0, 0) };
        led_toggle = !led_toggle;

        if let Err(error) = common_led::set_rgb(&mut led, color.0, color.1, color.2) {
            esp_println::println!("Failed to update controller LED color: {:?}", error);
        }

        delay.delay(Duration::from_millis(500));
    }
}
