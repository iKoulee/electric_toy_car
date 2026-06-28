#![no_std]
#![no_main]

mod esp_now_transport;
mod joystick;
mod usb_link;

use common_comms::espnow::ControllerLink;
use common_comms::protocol::{CONTROL_TX_INTERVAL_MS, ControlPacket};
use common_host_proto::{BoardToHost, HostToBoard};
use embassy_executor::Spawner;
use embassy_time::{Duration as EmbassyDuration, TimeoutError, Timer, with_timeout};
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    rmt::Rmt,
    timer::timg::TimerGroup,
    time::Rate,
    usb_serial_jtag::UsbSerialJtag,
};
use esp_radio::esp_now::BROADCAST_ADDRESS;

use joystick::{
    I2cScanSummary,
    JOYSTICK_DEFAULT_ADDRESS,
    I2C_FREQUENCY_KHZ,
    RUN_I2C_SCAN,
    RUN_STARTUP_PROBES,
    decode_joystick_state,
    encode_buttons,
    neutral_joystick_state,
    print_joystick_status,
    print_runtime_state,
    read_joystick_runtime_frame_async,
    resolve_active_joystick_address,
    run_i2c_joystick_diagnostics,
    run_joystick_dynamic_probe,
    scan_i2c_bus,
};

esp_bootloader_esp_idf::esp_app_desc!();

const CONTROL_LOOP_TICK_MS: u64 = 10;
// At 10kHz, a write_read of 38-byte frame can exceed 30ms; keep timeout above transfer worst-case.
const JOYSTICK_READ_TIMEOUT_MS: u64 = 80;
const READ_FAILURES_BEFORE_NEUTRAL_KEEPALIVE: u8 = 3;
const JOYSTICK_ERROR_LOG_PERIOD: u8 = 10;
const JOYSTICK_STATUS_LOG_INTERVAL_MS: u64 = 250;
const JOYSTICK_SAMPLE_LOGS_ENABLED: bool = false;
const JOYSTICK_PRINT_ON_CHANGE_ONLY: bool = true;
const CONTROL_TX_LOGS_ENABLED: bool = false;

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let usb = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    spawner.spawn(usb_link::task(usb)).expect("usb_link task spawn failed");

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
    esp_now.set_channel(1).expect("Failed to set ESP-NOW channel");

    let delay = Delay::new();

    esp_println::println!(
        "Controller Board initialized! ESP-NOW on channel 1, broadcast target {:02X?}",
        BROADCAST_ADDRESS
    );

    esp_println::println!(
        "Configuring I2C0 at {} kHz on GPIO6(SDA)/GPIO7(SCL)",
        I2C_FREQUENCY_KHZ
    );
    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(I2C_FREQUENCY_KHZ));
    let mut i2c = I2c::new(peripherals.I2C0, i2c_config)
        .expect("Failed to initialize I2C0")
        .with_sda(peripherals.GPIO6)
        .with_scl(peripherals.GPIO7);

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("Failed to initialize RMT");
    let mut led_buf = common_led::ws2812_buffer!();
    let mut led = common_led::new_ws2812(rmt.channel0, peripherals.GPIO8, &mut led_buf);
    let mut led_toggle = false;

    if let Err(error) = common_led::set_rgb(&mut led, 0, 16, 0) {
        esp_println::println!("Failed to set controller boot LED color: {:?}", error);
    }

    let scan_summary = if RUN_I2C_SCAN {
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
        esp_println::println!("Skipping full I2C scan; using validated joystick candidate probing.");
        I2cScanSummary { found_count: 0, first_found: None }
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
            JOYSTICK_DEFAULT_ADDRESS
        );
    }

    let mut i2c = i2c.into_async();

    let transport = esp_now_transport::Esp32C6EspNow::new(esp_now);
    let mut link = ControllerLink::new(transport, BROADCAST_ADDRESS);

    let mut tx_seq: u16 = 0;
    let mut last_sampled_state = None;
    let mut last_transmitted_state = None;
    // Start at interval so an immediate keepalive fires on the first tick.
    let mut ticks_since_tx: u64 = CONTROL_TX_INTERVAL_MS;
    let mut consecutive_read_failures: u8 = 0;
    let mut ticks_since_status_log: u64 = JOYSTICK_STATUS_LOG_INTERVAL_MS;
    // LED override from USB host: None = automatic state-driven color.
    let mut led_override: Option<[u8; 3]> = None;

    loop {
        // Cooperative control scheduling: fast sampling + event-driven TX + periodic keepalive.
        let mut tx_reason: Option<&str> = None;
        let mut tx_state = last_transmitted_state.unwrap_or_else(neutral_joystick_state);

        match with_timeout(
            EmbassyDuration::from_millis(JOYSTICK_READ_TIMEOUT_MS),
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
                    let _ = usb_link::EVENTS.try_send(BoardToHost::JoystickState {
                        x: state.x,
                        y: state.y,
                        buttons: button_mask,
                    });
                }

                if JOYSTICK_SAMPLE_LOGS_ENABLED && (!JOYSTICK_PRINT_ON_CHANGE_ONLY || changed) {
                    let button_mask = encode_buttons(&state.buttons);
                    let preview = ControlPacket::new(tx_seq, state.x, state.y, button_mask);
                    print_runtime_state(tx_seq as u32, address, &state, &preview, "sample");
                }

                last_sampled_state = Some(state);

                if changed && ticks_since_status_log >= JOYSTICK_STATUS_LOG_INTERVAL_MS {
                    print_joystick_status(&state, consecutive_read_failures);
                    ticks_since_status_log = 0;
                }
            }
            Ok(Err(error)) => {
                consecutive_read_failures = consecutive_read_failures.saturating_add(1);
                if consecutive_read_failures == 1
                    || consecutive_read_failures.is_multiple_of(JOYSTICK_ERROR_LOG_PERIOD)
                {
                    esp_println::println!(
                        "Joystick read failed at 0x{:02X}: {} (consecutive={})",
                        active_joystick_address.unwrap_or(JOYSTICK_DEFAULT_ADDRESS),
                        error,
                        consecutive_read_failures
                    );
                }
            }
            Err(TimeoutError) => {
                consecutive_read_failures = consecutive_read_failures.saturating_add(1);
                if consecutive_read_failures == 1
                    || consecutive_read_failures.is_multiple_of(JOYSTICK_ERROR_LOG_PERIOD)
                {
                    esp_println::println!(
                        "Joystick read timed out at 0x{:02X} after {} ms (consecutive={})",
                        active_joystick_address.unwrap_or(JOYSTICK_DEFAULT_ADDRESS),
                        JOYSTICK_READ_TIMEOUT_MS,
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
                    esp_println::println!(
                        "ESP-NOW sent #{} ({}) x={} y={} btn={:#04x}",
                        tx_seq, reason, tx_state.x, tx_state.y, button_mask
                    );
                }
                Err(e) => {
                    esp_println::println!("ESP-NOW send error: {:?}", e);
                }
            }

            if CONTROL_TX_LOGS_ENABLED {
                print_runtime_state(
                    tx_seq as u32,
                    active_joystick_address.unwrap_or(JOYSTICK_DEFAULT_ADDRESS),
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
            match cmd {
                HostToBoard::SetLed(color) => {
                    led_override = color;
                    let _ = usb_link::EVENTS.try_send(BoardToHost::LedAck);
                }
                _ => {}
            }
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

        Timer::after_millis(CONTROL_LOOP_TICK_MS).await;
        ticks_since_tx = ticks_since_tx.saturating_add(CONTROL_LOOP_TICK_MS);
        ticks_since_status_log = ticks_since_status_log.saturating_add(CONTROL_LOOP_TICK_MS);
    }
}
