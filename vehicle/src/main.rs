#![no_std]
#![no_main]

mod drive;
mod esp_now_transport;
mod ibt2;
mod pairing;
mod usb_link;

use ibt2::HBridge;

use common_comms::espnow::{EspNowLink, Inbound, BROADCAST_ADDRESS};
use common_comms::frame::MAX_ENCODED_FRAME;
use common_comms::protocol::ControlPacket;
use common_comms::{LinkState, LinkWatchdog, LINK_TIMEOUT_MS};
use common_host_proto::{
    decode_host_payload, encode_board_payload, BoardKind, BoardToHost, HostToBoard, LinkStateKind,
    RelayPayload, RELAY_PAYLOAD_MAX,
};
use common_led::{LED_BUFFER_SIZE, LedPulseCode, Ws2812Led};
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    interrupt::software::SoftwareInterruptControl,
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    rmt::Rmt,
    time::Rate,
    timer::timg::TimerGroup,
    usb_serial_jtag::UsbSerialJtag,
};

esp_bootloader_esp_idf::esp_app_desc!();

const VEHICLE_LOOP_INTERVAL_MS: u64 = 50;
/// Max ESP-NOW frames drained per tick, so a burst of control + tunnel frames
/// does not starve either path (the radio RX queue is 10 deep).
const MAX_RX_DRAIN_PER_TICK: usize = 8;

/// The vehicle's concrete ESP-NOW link type.
type VehicleEspLink<'r> = EspNowLink<esp_now_transport::Esp32C6EspNow<'r>>;
/// The vehicle's pairing store (flash lives for the whole program).
type VehiclePairingStore = pairing::PairingStore<'static>;

// ── Pure-logic loop state ─────────────────────────────────────────────────────

struct VehicleRunState {
    elapsed_ms: u64,
    last_state: LinkState,
    last_button: u8,
    tick: u64,
    led_toggle: bool,
    led_override: Option<[u8; 3]>,
    /// When set, this board mirrors its telemetry to the peer over the tunnel
    /// (it is acting as the remote board for a USB gateway).
    stream_to_peer: bool,
    /// USB/tunnel manual motor override. While `Some`, the motor holds this duty
    /// every tick — control packets and the link-timeout brake cannot stomp it.
    /// Cleared automatically when the physical operator reclaims control by
    /// moving the joystick out of its dead zone or pressing brake.
    manual_pwm: Option<i8>,
    /// Last duty commanded to the motor this tick, for `CurrentSenseRaw` telemetry.
    last_duty: i16,
}

impl VehicleRunState {
    fn new() -> Self {
        Self {
            elapsed_ms: 0,
            last_state: LinkState::AwaitingFirstPacket,
            last_button: 0,
            tick: 0,
            led_toggle: false,
            led_override: None,
            stream_to_peer: false,
            manual_pwm: None,
            last_duty: 0,
        }
    }
}

// ── Shared command / telemetry helpers ────────────────────────────────────────

/// Apply a host-protocol command, whether it arrived over USB (`CMDS`) or over
/// the ESP-NOW tunnel (`TunnelCmd`). Both paths are handled identically so a
/// USB-gateway relay behaves exactly like a direct USB connection.
fn apply_local_cmd(
    cmd: HostToBoard,
    motor: &mut impl HBridge,
    s: &mut VehicleRunState,
    link: &mut VehicleEspLink<'_>,
    store: &mut Option<VehiclePairingStore>,
) {
    match cmd {
        HostToBoard::SetLed(color) => {
            s.led_override = color;
            let _ = usb_link::EVENTS.try_send(BoardToHost::LedAck);
        }
        HostToBoard::SetMotorEnable { r_en, l_en } => {
            motor.set_r_en(r_en);
            motor.set_l_en(l_en);
        }
        HostToBoard::SetMotorPwm { duty } => {
            // Latch the manual override so it holds across control packets and
            // link-state transitions, then apply it and report it immediately.
            s.manual_pwm = Some(duty);
            motor.set_pwm(duty);
            s.last_duty = duty as i16;
            emit_telemetry(link, s, BoardToHost::MotorState { duty: duty as i16 });
        }
        HostToBoard::EnableRemoteTelemetry { on } => s.stream_to_peer = on,
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
fn emit_telemetry(link: &mut VehicleEspLink<'_>, s: &VehicleRunState, event: BoardToHost) {
    if s.stream_to_peer && link.is_paired() {
        let mut buf = [0u8; RELAY_PAYLOAD_MAX];
        if let Ok(n) = encode_board_payload(&event, &mut buf) {
            let _ = link.send_tunnel_evt(&buf[..n]);
        }
    }
    let _ = usb_link::EVENTS.try_send(event);
}

// ── Composition root ──────────────────────────────────────────────────────────

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let p = esp_hal::init(esp_hal::Config::default());

    esp_alloc::heap_allocator!(size: 64 * 1024);

    // ── Pairing persistence + re-pair gesture ─────────────────────────────────
    //
    // Hold BOOT (GPIO9) low during reset to clear the stored pairing and force a
    // fresh handshake. Otherwise load the paired peer MAC so the link comes up
    // in unicast immediately.
    let boot_btn = Input::new(p.GPIO9, InputConfig::default().with_pull(Pull::Up));
    let repair_requested = boot_btn.is_low();
    let mut pairing_store: Option<VehiclePairingStore> = pairing::PairingStore::new(p.FLASH);
    if repair_requested {
        if let Some(store) = pairing_store.as_mut() {
            store.clear();
        }
    }
    let stored_peer = pairing_store.as_mut().and_then(|s| s.load());

    // Embassy runtime must start before any radio/WiFi/ESP-NOW initialisation.
    let sw_int = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    let timg0 = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let usb = UsbSerialJtag::new(p.USB_DEVICE).into_async();
    spawner
        .spawn(usb_link::task(usb))
        .expect("usb_link task spawn failed");

    // ── Lifetime anchors ──────────────────────────────────────────────────────
    //
    // These variables must outlive all hardware objects that borrow from them:
    //
    // • esp_radio_ctrl — EspNow and wifi_ctrl borrow from it; kept alive so the
    //   wireless stack remains active for the program lifetime.
    // • wifi_ctrl      — holds the WiFi driver handle; must not drop.
    // • ledc / pwm_timer — LEDC channels store &'d dyn TimerIFace; dropping
    //   either would invalidate all motor PWM output.
    // • led_buf        — SmartLedsAdapter stores &'d mut [PulseCode; N].

    let esp_radio_ctrl = esp_radio::init().expect("Radio init failed");
    let (mut wifi_ctrl, interfaces) =
        esp_radio::wifi::new(&esp_radio_ctrl, p.WIFI, Default::default())
            .expect("ESP-NOW/WiFi init failed");
    wifi_ctrl
        .set_mode(esp_radio::wifi::WifiMode::Sta)
        .expect("WiFi set mode failed");
    wifi_ctrl.start().expect("WiFi start failed");
    let esp_now = interfaces.esp_now;

    let mut ledc = Ledc::new(p.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut pwm_timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    pwm_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(9765),
        })
        .expect("LEDC timer config failed");

    let mut rpwm = ledc.channel::<LowSpeed>(channel::Number::Channel0, p.GPIO2);
    rpwm.configure(channel::config::Config {
        timer: &pwm_timer,
        duty_pct: 0,
        drive_mode: esp_hal::gpio::DriveMode::PushPull,
    })
    .expect("RPWM channel config failed");

    let mut lpwm = ledc.channel::<LowSpeed>(channel::Number::Channel1, p.GPIO3);
    lpwm.configure(channel::config::Config {
        timer: &pwm_timer,
        duty_pct: 0,
        drive_mode: esp_hal::gpio::DriveMode::PushPull,
    })
    .expect("LPWM channel config failed");

    let r_en = Output::new(p.GPIO4, Level::High, OutputConfig::default());
    let l_en = Output::new(p.GPIO5, Level::High, OutputConfig::default());
    // One IBT-2 driver owns motor control + current sense: ADC1 on GPIO0 (R_IS) and
    // GPIO1 (L_IS).
    let mut driver = ibt2::Ibt2::new(rpwm, lpwm, r_en, l_en, p.ADC1, p.GPIO0, p.GPIO1);

    // Capture the idle current-sense baseline before driving the motor.
    driver.calibrate_offset().await;

    let mut led_buf = common_led::ws2812_buffer!();

    // ── Transport + LED setup ─────────────────────────────────────────────────

    let (link, led, rx_buf) = setup(esp_now, p.RMT, p.GPIO8, &mut led_buf).await;

    // ── Main loop ─────────────────────────────────────────────────────────────

    run(link, driver, led, rx_buf, pairing_store, stored_peer).await
}

// ── Hardware setup (runtime · transport · LED) ────────────────────────────────
//
// Wraps ESP-NOW in the vehicle link and initialises the WS2812B LED. All
// radio/WiFi objects are anchored in main() because EspNow borrows from
// esp_radio_ctrl for its lifetime.

async fn setup<'radio, 'led>(
    esp_now: esp_radio::esp_now::EspNow<'radio>,
    rmt: esp_hal::peripherals::RMT<'static>,
    gpio8: esp_hal::peripherals::GPIO8<'static>,
    led_buf: &'led mut [LedPulseCode; LED_BUFFER_SIZE],
) -> (
    VehicleEspLink<'radio>,
    Ws2812Led<'led, { LED_BUFFER_SIZE }>,
    [u8; MAX_ENCODED_FRAME],
) {
    esp_now
        .set_channel(1)
        .expect("Failed to set ESP-NOW channel");
    let transport = esp_now_transport::Esp32C6EspNow::new(esp_now);
    let link = EspNowLink::new(transport);
    let rx_buf = [0u8; MAX_ENCODED_FRAME];

    let rmt = Rmt::new(rmt, Rate::from_mhz(80)).expect("Failed to initialize RMT");
    let mut led = common_led::new_ws2812(rmt.channel0, gpio8, led_buf);
    let _ = common_led::set_rgb(&mut led, 16, 16, 0); // yellow boot colour

    (link, led, rx_buf)
}

// ── Main control loop ─────────────────────────────────────────────────────────

async fn run<'radio, 'led, D: HBridge>(
    mut link: VehicleEspLink<'radio>,
    mut motor: D,
    mut led: Ws2812Led<'led, { LED_BUFFER_SIZE }>,
    mut rx_buf: [u8; MAX_ENCODED_FRAME],
    mut store: Option<VehiclePairingStore>,
    stored_peer: Option<[u8; 6]>,
) -> ! {
    let mut watchdog = LinkWatchdog::new(LINK_TIMEOUT_MS);
    let mut s = VehicleRunState::new();

    // Come up already paired when a peer MAC survived in flash.
    if let Some(mac) = stored_peer {
        let _ = link.learn_peer(mac);
    }

    loop {
        s.tick = s.tick.wrapping_add(1);

        // --- ESP-NOW receive drain (control + tunnel + pairing) ---
        for _ in 0..MAX_RX_DRAIN_PER_TICK {
            match link.try_receive(&mut rx_buf) {
                Ok(Some(Inbound::Control(received))) => {
                    // Once paired, ignore control from any other device so a
                    // second nearby car/controller cannot drive this vehicle.
                    if let Some(peer) = link.paired_peer() {
                        if received.meta.peer_mac != peer {
                            continue;
                        }
                    }
                    watchdog.record_valid_packet(s.elapsed_ms);

                    // Pairing bootstrap: learn + persist the controller MAC.
                    if !link.is_paired() && link.learn_peer(received.meta.peer_mac).is_ok() {
                        if let Some(st) = store.as_mut() {
                            st.save(received.meta.peer_mac);
                        }
                    }
                    // A broadcast control frame means the controller has not
                    // learned us yet; acknowledge so it can switch to unicast.
                    if received.meta.dst_mac == BROADCAST_ADDRESS {
                        let _ = link.send_pair_ack();
                    }

                    // A physical operator (brake button, or joystick pushed out
                    // of its dead zone) always reclaims control from a USB manual
                    // override — safety takes precedence over remote control.
                    let brake = received.packet.buttons & ControlPacket::BUTTON_C != 0;
                    if brake || drive::y_to_duty(received.packet.y) != 0 {
                        s.manual_pwm = None;
                    }
                    let duty: i16 = if brake {
                        motor.brake();
                        0
                    } else if let Some(pwm) = s.manual_pwm {
                        // USB manual override holds while the joystick is centred.
                        pwm as i16
                    } else {
                        let duty = drive::y_to_duty(received.packet.y);
                        motor.set_pwm(duty as i8);
                        duty
                    };
                    s.last_duty = duty;
                    emit_telemetry(
                        &mut link,
                        &s,
                        BoardToHost::ReceivedPacket {
                            x: received.packet.x,
                            y: received.packet.y,
                            buttons: received.packet.buttons,
                        },
                    );
                    emit_telemetry(&mut link, &s, BoardToHost::MotorState { duty });

                    let tracked = ControlPacket::BUTTON_A
                        | ControlPacket::BUTTON_B
                        | ControlPacket::BUTTON_C
                        | ControlPacket::BUTTON_D;
                    if received.packet.buttons & tracked != 0 {
                        s.last_button = received.packet.buttons & tracked;
                    }
                }
                Ok(Some(Inbound::TunnelCmd { bytes, peer })) => {
                    // Only accept tunnelled commands from the paired peer.
                    if link.paired_peer() != Some(peer) {
                        continue;
                    }
                    // Copy out to release the rx buffer borrow before acting.
                    let mut tmp = [0u8; RELAY_PAYLOAD_MAX];
                    let n = bytes.len().min(tmp.len());
                    tmp[..n].copy_from_slice(&bytes[..n]);
                    if let Ok(cmd) = decode_host_payload(&tmp[..n]) {
                        apply_local_cmd(cmd, &mut motor, &mut s, &mut link, &mut store);
                    }
                }
                Ok(Some(Inbound::TunnelEvt { bytes, peer })) => {
                    // Acting as gateway: forward the paired peer's telemetry to USB.
                    if link.paired_peer() != Some(peer) {
                        continue;
                    }
                    if let Ok(payload) = RelayPayload::from_slice(bytes) {
                        let _ = usb_link::EVENTS.try_send(BoardToHost::FromPeer {
                            source: BoardKind::Controller,
                            payload,
                        });
                    }
                }
                Ok(Some(Inbound::PairAck { .. })) => {
                    // The vehicle initiates pairing; a PairAck here is unexpected.
                }
                Ok(None) => break,
                // Stale/parse errors: skip this frame and keep draining.
                Err(_) => {}
            }
        }

        // --- Link state machine ---
        let link_state = watchdog.state(s.elapsed_ms);
        if link_state != s.last_state {
            let kind = match link_state {
                LinkState::AwaitingFirstPacket => LinkStateKind::AwaitingFirstPacket,
                LinkState::Alive => {
                    motor.enable();
                    for _ in 0..2 {
                        let _ = common_led::set_rgb(&mut led, 0, 0, 16);
                        Timer::after_millis(200).await;
                        let _ = common_led::set_rgb(&mut led, 0, 0, 0);
                        Timer::after_millis(200).await;
                    }
                    LinkStateKind::Alive
                }
                LinkState::TimedOut => {
                    link.reset_sequence();
                    // brake() keeps H-bridges enabled so USB SetMotorPwm commands
                    // take effect immediately while timed-out. A latched manual
                    // override (`s.manual_pwm`) is re-asserted after this each tick,
                    // so a computer keeps direct motor control via USB cable.
                    motor.brake();
                    s.last_duty = 0;
                    LinkStateKind::TimedOut
                }
            };
            emit_telemetry(&mut link, &s, BoardToHost::EspNowLinkState(kind));
            s.last_state = link_state;
        }

        // --- USB command drain ---
        while let Ok(cmd) = usb_link::CMDS.try_receive() {
            apply_local_cmd(cmd, &mut motor, &mut s, &mut link, &mut store);
        }

        // --- Hold manual PWM ---
        // Re-assert the USB manual override every tick so control-packet handling
        // and link-state transitions (notably the timeout brake) cannot leave it
        // at zero. Runs regardless of link state, matching the documented intent
        // that a USB host can drive the motor even while the RF link is down.
        if let Some(pwm) = s.manual_pwm {
            motor.set_pwm(pwm);
            s.last_duty = pwm as i16;
        }

        // --- Current sense ---
        // Sample the IBT-2 IS pins every 4th tick (~200 ms) so the reading does
        // not flood the 50 ms loop or the depth-8 USB event channel. Reuses the
        // telemetry path, so it also mirrors over the tunnel when streaming.
        if s.tick % 4 == 0 {
            let c = motor.read_current().await;
            emit_telemetry(
                &mut link,
                &s,
                BoardToHost::CurrentSense {
                    r_ma: c.r_ma,
                    l_ma: c.l_ma,
                },
            );
            emit_telemetry(
                &mut link,
                &s,
                BoardToHost::CurrentSenseRaw {
                    r_mv: c.r_mv,
                    l_mv: c.l_mv,
                    duty: s.last_duty,
                },
            );
        }

        // --- LED colour ---
        let color = if let Some([r, g, b]) = s.led_override {
            (r, g, b)
        } else {
            match link_state {
                LinkState::AwaitingFirstPacket => (16, 12, 0),
                LinkState::Alive => {
                    if s.last_button & ControlPacket::BUTTON_A != 0 {
                        (16, 12, 0)
                    } else if s.last_button & ControlPacket::BUTTON_B != 0 {
                        (16, 16, 16)
                    } else if s.last_button & ControlPacket::BUTTON_C != 0 {
                        (16, 0, 0)
                    } else if s.last_button & ControlPacket::BUTTON_D != 0 {
                        (0, 0, 16)
                    } else {
                        (0, 16, 0)
                    }
                }
                LinkState::TimedOut => {
                    let blink = s.led_toggle;
                    s.led_toggle = !s.led_toggle;
                    if blink { (16, 0, 0) } else { (2, 0, 0) }
                }
            }
        };
        let _ = common_led::set_rgb(&mut led, color.0, color.1, color.2);

        Timer::after_millis(VEHICLE_LOOP_INTERVAL_MS).await;
        s.elapsed_ms = s.elapsed_ms.saturating_add(VEHICLE_LOOP_INTERVAL_MS);
    }
}
