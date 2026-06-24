#![no_std]
#![no_main]

mod esp_now_transport;

use common_comms::{
    LinkState,
    LinkWatchdog,
    LINK_TIMEOUT_MS,
    CONTROL_PACKET_LEN,
};
use common_comms::espnow::{LinkError, VehicleLink};
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    interrupt::software::SoftwareInterruptControl,
    rmt::Rmt,
    time::Duration,
    time::Rate,
    timer::timg::TimerGroup,
};

esp_bootloader_esp_idf::esp_app_desc!();

const VEHICLE_LOOP_INTERVAL_MS: u64 = 50;

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let delay = Delay::new();
    let mut led_toggle = false;

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("Failed to initialize RMT");
    let mut led_buf = common_led::ws2812_buffer!();
    let mut led = common_led::new_ws2812(rmt.channel0, peripherals.GPIO8, &mut led_buf);

    if let Err(error) = common_led::set_rgb(&mut led, 16, 16, 0) {
        esp_println::println!("Failed to set vehicle boot LED color: {:?}", error);
    }

    esp_println::println!("Main Vehicle Board initialized!");

    // TODO(next): Initialize PWM for H-Bridge motor control and expose a stop command API.

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let esp_radio_ctrl = esp_radio::init().expect("Radio init failed");
    let (mut wifi_ctrl, interfaces) =
        esp_radio::wifi::new(&esp_radio_ctrl, peripherals.WIFI, Default::default())
            .expect("ESP-NOW/WiFi init failed");
    wifi_ctrl
        .set_mode(esp_radio::wifi::WifiMode::Sta)
        .expect("WiFi set mode failed");
    wifi_ctrl.start().expect("WiFi start failed");
    let esp_now = interfaces.esp_now;

    let transport = esp_now_transport::Esp32C6EspNow::new(esp_now);
    let mut vehicle_link = VehicleLink::new(transport);
    let mut rx_buf = [0u8; CONTROL_PACKET_LEN];

    let mut watchdog = LinkWatchdog::new(LINK_TIMEOUT_MS);
    let mut elapsed_ms: u64 = 0;
    let mut last_state = LinkState::AwaitingFirstPacket;

    loop {
        match vehicle_link.try_receive_control(&mut rx_buf) {
            Ok(Some(received)) => {
                watchdog.record_valid_packet(elapsed_ms);
                // TODO(motor): apply received.packet.x / .y / .buttons to H-bridge PWM
                let _ = received;
            }
            Ok(None) => {}
            Err(LinkError::StaleSequence) => {}
            Err(e) => {
                esp_println::println!("VehicleLink recv error: {:?}", e);
            }
        }

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
                    // TODO(next): Immediately command H-bridge stop state here.
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
