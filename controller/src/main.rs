#![no_std]
#![no_main]

mod board;
mod config;
mod esp_now_transport;
mod joystick;
mod pairing;
mod usb_link;

use common_comms::espnow::{EspNowLink, Inbound, BROADCAST_ADDRESS};
use common_comms::frame::MAX_ENCODED_FRAME;
use common_comms::protocol::{ControlPacket, CONTROL_TX_INTERVAL_MS};
use common_host_proto::{
    decode_host_payload, encode_board_payload, BoardKind, BoardToHost, HostToBoard, RelayPayload,
    RELAY_PAYLOAD_MAX,
};
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration as EmbassyDuration, TimeoutError, Timer};
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    rmt::Rmt,
    time::Rate,
    timer::timg::TimerGroup,
    usb_serial_jtag::UsbSerialJtag,
};

use config::control::{MAX_RX_DRAIN_PER_TICK, READ_FAILURES_BEFORE_NEUTRAL_KEEPALIVE};
use config::diag::{
    ERROR_LOG_PERIOD, PRINT_ON_CHANGE_ONLY, RUN_SCAN, RUN_STARTUP_PROBES, SAMPLE_LOGS_ENABLED,
    TX_LOGS_ENABLED,
};
use config::i2c::FREQUENCY_KHZ;
use config::joystick::DEFAULT_ADDRESS;
use config::timing::{LOOP_TICK_MS, READ_TIMEOUT_MS, STATUS_LOG_INTERVAL_MS};
use joystick::{
    decode_joystick_state, encode_buttons, neutral_joystick_state, print_joystick_status,
    print_runtime_state, read_joystick_runtime_frame_async, resolve_active_joystick_address,
    run_i2c_joystick_diagnostics, run_joystick_dynamic_probe, scan_i2c_bus, I2cScanSummary,
};

esp_bootloader_esp_idf::esp_app_desc!();

/// The controller's concrete ESP-NOW link type.
type ControllerEspLink<'r> = EspNowLink<esp_now_transport::Esp32C6EspNow<'r>>;
/// The controller's pairing store (flash lives for the whole program).
type ControllerPairingStore = pairing::PairingStore<'static>;

/// Apply a host-protocol command, whether it arrived over USB (`CMDS`) or over
/// the ESP-NOW tunnel (`TunnelCmd`). Both paths are handled identically.
fn apply_local_cmd(
    cmd: HostToBoard,
    led_override: &mut Option<[u8; 3]>,
    stream_to_peer: &mut bool,
    link: &mut ControllerEspLink<'_>,
    store: &mut Option<ControllerPairingStore>,
) {
    match cmd {
        HostToBoard::SetLed(color) => {
            *led_override = color;
            let _ = usb_link::EVENTS.try_send(BoardToHost::LedAck);
        }
        HostToBoard::EnableRemoteTelemetry { on } => *stream_to_peer = on,
        HostToBoard::Repair => {
            if let Some(st) = store.as_mut() {
                st.clear();
            }
            link.reset_pairing();
        }
        HostToBoard::ForPeer(payload) => {
            let _ = link.send_tunnel_cmd(&payload);
        }
        _ => {}
    }
}

/// Emit telemetry to the USB host, and — when remote-telemetry streaming is on
/// and a peer is paired — also mirror it over the tunnel to the gateway.
fn emit_telemetry(link: &mut ControllerEspLink<'_>, stream_to_peer: bool, event: BoardToHost) {
    if stream_to_peer && link.is_paired() {
        let mut buf = [0u8; RELAY_PAYLOAD_MAX];
        if let Ok(n) = encode_board_payload(&event, &mut buf) {
            let _ = link.send_tunnel_evt(&buf[..n]);
        }
    }
    let _ = usb_link::EVENTS.try_send(event);
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::heap_allocator!(size: 64 * 1024);

    // Pairing persistence + re-pair gesture: hold BOOT (GPIO9) low during reset
    // to clear the stored pairing and force a fresh handshake; otherwise load the
    // paired vehicle MAC so control comes up in unicast immediately.
    let pins = board::controller_pins!(peripherals);
    let boot_btn = Input::new(pins.boot, InputConfig::default().with_pull(Pull::Up));
    let repair_requested = boot_btn.is_low();
    let mut pairing_store: Option<ControllerPairingStore> =
        pairing::PairingStore::new(peripherals.FLASH);
    if repair_requested {
        if let Some(store) = pairing_store.as_mut() {
            store.clear();
        }
    }
    let stored_peer = pairing_store.as_mut().and_then(|s| s.load());

    let usb = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    spawner
        .spawn(usb_link::task(usb))
        .expect("usb_link task spawn failed");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);

    let esp_radio_ctrl = esp_radio::init().expect("Radio init failed");
    let (mut wifi_ctrl, interfaces) =
        esp_radio::wifi::new(&esp_radio_ctrl, peripherals.WIFI, Default::default())
            .expect("ESP-NOW/WiFi init failed");
    wifi_ctrl
        .set_mode(esp_radio::wifi::WifiMode::Sta)
        .expect("WiFi set mode failed");
    wifi_ctrl.start().expect("WiFi start failed");
    let esp_now = interfaces.esp_now;
    esp_now
        .set_channel(1)
        .expect("Failed to set ESP-NOW channel");

    let delay = Delay::new();

    esp_println::println!(
        "Controller Board initialized! ESP-NOW on channel 1, broadcast target {:02X?}",
        BROADCAST_ADDRESS
    );

    esp_println::println!(
        "Configuring I2C0 at {} kHz on GPIO6(SDA)/GPIO7(SCL)",
        FREQUENCY_KHZ
    );
    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(FREQUENCY_KHZ));
    let mut i2c = I2c::new(peripherals.I2C0, i2c_config)
        .expect("Failed to initialize I2C0")
        .with_sda(pins.sda)
        .with_scl(pins.scl);

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("Failed to initialize RMT");
    let mut led_buf = common_led::ws2812_buffer!();
    let mut led = common_led::new_ws2812(rmt.channel0, pins.led, &mut led_buf);
    let mut led_toggle = false;

    if let Err(error) = common_led::set_rgb(&mut led, 0, 16, 0) {
        esp_println::println!("Failed to set controller boot LED color: {:?}", error);
    }

    let scan_summary = if RUN_SCAN {
        let summary = scan_i2c_bus(&mut i2c);
        if let Some(found) = summary.first_found {
            esp_println::println!(
                "I2C scan summary: {} device(s), first at 0x{:02X}.",
                summary.found_count,
                found
            );
        }
        summary
    } else {
        esp_println::println!(
            "Skipping full I2C scan; using validated joystick candidate probing."
        );
        I2cScanSummary {
            found_count: 0,
            first_found: None,
        }
    };

    if RUN_STARTUP_PROBES {
        run_i2c_joystick_diagnostics(&mut i2c, scan_summary.first_found);
    }

    let active_joystick_address = resolve_active_joystick_address(&mut i2c, scan_summary);

    if RUN_STARTUP_PROBES {
        if let Some(address) = active_joystick_address {
            run_joystick_dynamic_probe(&mut i2c, &delay, address);
        }
    }

    if let Some(address) = active_joystick_address {
        esp_println::println!("Joystick active address resolved to 0x{:02X}.", address);
    } else {
        esp_println::println!(
            "Warning: could not resolve a working joystick address; last known default is 0x{:02X}.",
            DEFAULT_ADDRESS
        );
    }

    let mut i2c = i2c.into_async();

    let transport = esp_now_transport::Esp32C6EspNow::new(esp_now);
    let mut link = EspNowLink::new(transport);
    if let Some(mac) = stored_peer {
        let _ = link.learn_peer(mac);
    }

    let mut tx_seq: u16 = 0;
    let mut last_sampled_state = None;
    let mut last_transmitted_state = None;
    // Start at interval so an immediate keepalive fires on the first tick.
    let mut ticks_since_tx: u64 = CONTROL_TX_INTERVAL_MS;
    let mut consecutive_read_failures: u8 = 0;
    let mut ticks_since_status_log: u64 = STATUS_LOG_INTERVAL_MS;
    // LED override from USB host: None = automatic state-driven color.
    let mut led_override: Option<[u8; 3]> = None;
    // Whether this board mirrors its telemetry to the peer over the tunnel.
    let mut stream_to_peer = false;
    let mut rx_buf = [0u8; MAX_ENCODED_FRAME];

    loop {
        // --- ESP-NOW receive drain (pairing acks + tunnel; controller ignores
        //     inbound control frames) ---
        for _ in 0..MAX_RX_DRAIN_PER_TICK {
            match link.try_receive(&mut rx_buf) {
                Ok(Some(Inbound::PairAck { peer })) => {
                    // The vehicle acknowledged pairing: learn + persist its MAC
                    // and switch control transmission to unicast. Ignore once
                    // paired so a foreign board cannot hijack the pairing.
                    if !link.is_paired() && link.learn_peer(peer).is_ok() {
                        if let Some(st) = pairing_store.as_mut() {
                            st.save(peer);
                        }
                    }
                }
                Ok(Some(Inbound::TunnelCmd { bytes, peer })) => {
                    if link.paired_peer() != Some(peer) {
                        continue;
                    }
                    let mut tmp = [0u8; RELAY_PAYLOAD_MAX];
                    let n = bytes.len().min(tmp.len());
                    tmp[..n].copy_from_slice(&bytes[..n]);
                    if let Ok(cmd) = decode_host_payload(&tmp[..n]) {
                        apply_local_cmd(
                            cmd,
                            &mut led_override,
                            &mut stream_to_peer,
                            &mut link,
                            &mut pairing_store,
                        );
                    }
                }
                Ok(Some(Inbound::TunnelEvt { bytes, peer })) => {
                    // Acting as gateway: forward the paired peer's telemetry to USB.
                    if link.paired_peer() != Some(peer) {
                        continue;
                    }
                    if let Ok(payload) = RelayPayload::from_slice(bytes) {
                        let _ = usb_link::EVENTS.try_send(BoardToHost::FromPeer {
                            source: BoardKind::Vehicle,
                            payload,
                        });
                    }
                }
                Ok(Some(Inbound::Control(_))) => {}
                Ok(None) => break,
                Err(_) => {}
            }
        }

        // Cooperative control scheduling: fast sampling + event-driven TX + periodic keepalive.
        let mut tx_reason: Option<&str> = None;
        let mut tx_state = last_transmitted_state.unwrap_or_else(neutral_joystick_state);

        match with_timeout(
            EmbassyDuration::from_millis(READ_TIMEOUT_MS),
            read_joystick_runtime_frame_async(&mut i2c, active_joystick_address),
        )
        .await
        {
            Ok(Ok((address, frame))) => {
                let state = decode_joystick_state(&frame);
                let joystick_active = state.x != 128 || state.y != 128;
                let changed = last_sampled_state != Some(state) || joystick_active;
                consecutive_read_failures = 0;

                if changed {
                    tx_reason = Some("change");
                    tx_state = state;
                    let button_mask = encode_buttons(&state.buttons);
                    emit_telemetry(
                        &mut link,
                        stream_to_peer,
                        BoardToHost::JoystickState {
                            x: state.x,
                            y: state.y,
                            buttons: button_mask,
                        },
                    );
                }

                if SAMPLE_LOGS_ENABLED && (!PRINT_ON_CHANGE_ONLY || changed) {
                    let button_mask = encode_buttons(&state.buttons);
                    let preview = ControlPacket::new(tx_seq, state.x, state.y, button_mask);
                    print_runtime_state(tx_seq as u32, address, &state, &preview, "sample");
                }

                last_sampled_state = Some(state);

                if changed && ticks_since_status_log >= STATUS_LOG_INTERVAL_MS {
                    print_joystick_status(&state, consecutive_read_failures);
                    ticks_since_status_log = 0;
                }
            }
            Ok(Err(error)) => {
                consecutive_read_failures = consecutive_read_failures.saturating_add(1);
                if consecutive_read_failures == 1
                    || consecutive_read_failures.is_multiple_of(ERROR_LOG_PERIOD)
                {
                    esp_println::println!(
                        "Joystick read failed at 0x{:02X}: {} (consecutive={})",
                        active_joystick_address.unwrap_or(DEFAULT_ADDRESS),
                        error,
                        consecutive_read_failures
                    );
                }
            }
            Err(TimeoutError) => {
                consecutive_read_failures = consecutive_read_failures.saturating_add(1);
                if consecutive_read_failures == 1
                    || consecutive_read_failures.is_multiple_of(ERROR_LOG_PERIOD)
                {
                    esp_println::println!(
                        "Joystick read timed out at 0x{:02X} after {} ms (consecutive={})",
                        active_joystick_address.unwrap_or(DEFAULT_ADDRESS),
                        READ_TIMEOUT_MS,
                        consecutive_read_failures
                    );
                }
            }
        }

        if tx_reason.is_none() && ticks_since_tx >= CONTROL_TX_INTERVAL_MS {
            if consecutive_read_failures >= READ_FAILURES_BEFORE_NEUTRAL_KEEPALIVE {
                tx_reason = Some("keepalive-neutral");
                tx_state = neutral_joystick_state();
            } else {
                tx_reason = Some("keepalive");
                tx_state = last_sampled_state.unwrap_or_else(neutral_joystick_state);
            }
        }

        if let Some(reason) = tx_reason {
            tx_seq = tx_seq.wrapping_add(1);
            let button_mask = encode_buttons(&tx_state.buttons);
            let packet = ControlPacket::new(tx_seq, tx_state.x, tx_state.y, button_mask);

            match link.send_control(packet) {
                Ok(()) => {
                    if TX_LOGS_ENABLED {
                        esp_println::println!(
                            "ESP-NOW sent #{} ({}) x={} y={} btn={:#04x}",
                            tx_seq,
                            reason,
                            tx_state.x,
                            tx_state.y,
                            button_mask
                        );
                    }
                }
                Err(e) => {
                    esp_println::println!("ESP-NOW send error: {:?}", e);
                }
            }

            if TX_LOGS_ENABLED {
                print_runtime_state(
                    tx_seq as u32,
                    active_joystick_address.unwrap_or(DEFAULT_ADDRESS),
                    &tx_state,
                    &packet,
                    reason,
                );
            }

            last_transmitted_state = Some(tx_state);
            ticks_since_tx = 0;
        }

        // Process USB host commands.
        while let Ok(cmd) = usb_link::CMDS.try_receive() {
            apply_local_cmd(
                cmd,
                &mut led_override,
                &mut stream_to_peer,
                &mut link,
                &mut pairing_store,
            );
        }

        let color = if let Some([r, g, b]) = led_override {
            (r, g, b)
        } else if led_toggle {
            (0, 0, 16)
        } else {
            (16, 0, 0)
        };
        led_toggle = !led_toggle;

        if let Err(error) = common_led::set_rgb(&mut led, color.0, color.1, color.2) {
            esp_println::println!("Failed to update controller LED color: {:?}", error);
        }

        Timer::after_millis(LOOP_TICK_MS).await;
        ticks_since_tx = ticks_since_tx.saturating_add(LOOP_TICK_MS);
        ticks_since_status_log = ticks_since_status_log.saturating_add(LOOP_TICK_MS);
    }
}
