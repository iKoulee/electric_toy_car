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
use common_comms::protocol::ControlPacket;
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    interrupt::software::SoftwareInterruptControl,
    rmt::Rmt,
    time::Rate,
    timer::timg::TimerGroup,
};

esp_bootloader_esp_idf::esp_app_desc!();

const VEHICLE_LOOP_INTERVAL_MS: u64 = 50;

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::heap_allocator!(size: 64 * 1024);

    // Mirror controller init order exactly: radio/WiFi BEFORE any peripheral (RMT/LED) setup.
    esp_println::println!("init[1]: SoftwareInterruptControl + TimerGroup");
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_println::println!("init[2]: esp_rtos::start");
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);
    esp_println::println!("init[3]: esp_radio::init");
    let esp_radio_ctrl = esp_radio::init().expect("Radio init failed");
    esp_println::println!("init[4]: esp_radio::wifi::new");
    let (mut wifi_ctrl, interfaces) =
        esp_radio::wifi::new(&esp_radio_ctrl, peripherals.WIFI, Default::default())
            .expect("ESP-NOW/WiFi init failed");
    esp_println::println!("init[5]: wifi set_mode Sta + start");
    wifi_ctrl
        .set_mode(esp_radio::wifi::WifiMode::Sta)
        .expect("WiFi set mode failed");
    wifi_ctrl.start().expect("WiFi start failed");
    esp_println::println!("init[6]: esp_now set_channel 1");
    let esp_now = interfaces.esp_now;
    esp_now.set_channel(1).expect("Failed to set ESP-NOW channel");

    // LED init after WiFi, same as controller.
    let mut led_toggle = false;
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("Failed to initialize RMT");
    let mut led_buf = common_led::ws2812_buffer!();
    let mut led = common_led::new_ws2812(rmt.channel0, peripherals.GPIO8, &mut led_buf);
    if let Err(error) = common_led::set_rgb(&mut led, 16, 16, 0) {
        esp_println::println!("Failed to set vehicle boot LED color: {:?}", error);
    }

    esp_println::println!("Main Vehicle Board initialized! ESP-NOW on channel 1");

    let transport = esp_now_transport::Esp32C6EspNow::new(esp_now);
    let mut vehicle_link = VehicleLink::new(transport);
    let mut rx_buf = [0u8; CONTROL_PACKET_LEN];

    let mut watchdog = LinkWatchdog::new(LINK_TIMEOUT_MS);
    let mut elapsed_ms: u64 = 0;
    let mut last_state = LinkState::AwaitingFirstPacket;
    let mut last_button: u8 = 0;
    let mut tick: u64 = 0;

    loop {
        tick = tick.wrapping_add(1);

        match vehicle_link.try_receive_control(&mut rx_buf) {
            Ok(Some(received)) => {
                watchdog.record_valid_packet(elapsed_ms);
                let (seq, px, py, pbtn) = (
                    received.packet.sequence,
                    received.packet.x,
                    received.packet.y,
                    received.packet.buttons,
                );
                esp_println::println!(
                    "[tick {}] ESP-NOW received seq={} x={} y={} btn={:#04x} rssi={:?}",
                    tick, seq, px, py, pbtn, received.meta.rssi_dbm
                );
                let btns = received.packet.buttons;
                let tracked = ControlPacket::BUTTON_A | ControlPacket::BUTTON_B
                    | ControlPacket::BUTTON_C | ControlPacket::BUTTON_D;
                if btns & tracked != 0 {
                    last_button = btns & tracked;
                }
            }
            Ok(None) => {
                if tick % 20 == 0 {
                    esp_println::println!("[tick {}] ESP-NOW: nothing received yet", tick);
                }
            }
            Err(LinkError::StaleSequence) => {
                esp_println::println!("[tick {}] ESP-NOW: stale/replayed sequence dropped", tick);
            }
            Err(e) => {
                esp_println::println!("[tick {}] VehicleLink recv error: {:?}", tick, e);
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
                    for _ in 0..2 {
                        let _ = common_led::set_rgb(&mut led, 0, 0, 16);
                        Timer::after_millis(200).await;
                        let _ = common_led::set_rgb(&mut led, 0, 0, 0);
                        Timer::after_millis(200).await;
                    }
                }
                LinkState::TimedOut => {
                    esp_println::println!("Vehicle link state: timed out, entering fail-safe stop");
                    vehicle_link.reset_sequence();
                    // TODO(next): Immediately command H-bridge stop state here.
                }
            }
            last_state = state;
        }

        let color = match state {
            LinkState::AwaitingFirstPacket => (16, 12, 0),
            LinkState::Alive => {
                if last_button & ControlPacket::BUTTON_A != 0 {
                    (16, 12, 0)
                } else if last_button & ControlPacket::BUTTON_B != 0 {
                    (16, 16, 16)
                } else if last_button & ControlPacket::BUTTON_C != 0 {
                    (16, 0, 0)
                } else if last_button & ControlPacket::BUTTON_D != 0 {
                    (0, 0, 16)
                } else {
                    (0, 16, 0)
                }
            }
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

        Timer::after_millis(VEHICLE_LOOP_INTERVAL_MS).await;
        elapsed_ms = elapsed_ms.saturating_add(VEHICLE_LOOP_INTERVAL_MS);
    }
}
