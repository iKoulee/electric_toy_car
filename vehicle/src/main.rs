#![no_std]
#![no_main]

use common_comms::{
    LinkState,
    LinkWatchdog,
    LINK_TIMEOUT_MS,
};
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    rmt::Rmt,
    time::Duration,
    time::Rate,
};

esp_bootloader_esp_idf::esp_app_desc!();

const VEHICLE_LOOP_INTERVAL_MS: u64 = 50;

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

    let watchdog = LinkWatchdog::new(LINK_TIMEOUT_MS);
    let mut elapsed_ms: u64 = 0;
    let mut last_state = LinkState::AwaitingFirstPacket;

    loop {
        // TODO: Poll ESP-NOW receive queue and call watchdog.record_valid_packet(elapsed_ms)
        //       whenever a fresh, checksum-valid control packet arrives.
        let state = watchdog.state(elapsed_ms);

        if state != last_state {
            match state {
                LinkState::AwaitingFirstPacket => {
                    esp_println::println!("Vehicle link state: waiting for first control packet");
                }
                LinkState::Alive => {
                    esp_println::println!("Vehicle link state: alive");
                }
                LinkState::TimedOut => {
                    esp_println::println!("Vehicle link state: timed out, entering fail-safe stop");
                    // TODO: Immediately command H-bridge stop state here.
                }
            }
            last_state = state;
        }

        let color = match state {
            LinkState::AwaitingFirstPacket => (16, 12, 0),
            LinkState::Alive => (0, 16, 0),
            LinkState::TimedOut => {
                let blink = led_toggle;
                led_toggle = !led_toggle;
                if blink {
                    (16, 0, 0)
                } else {
                    (2, 0, 0)
                }
            }
        };

        if let Err(error) = common_led::set_rgb(&mut led, color.0, color.1, color.2) {
            esp_println::println!("Failed to update vehicle LED color: {:?}", error);
        }

        delay.delay(Duration::from_millis(VEHICLE_LOOP_INTERVAL_MS));
        elapsed_ms = elapsed_ms.saturating_add(VEHICLE_LOOP_INTERVAL_MS);
    }
}
