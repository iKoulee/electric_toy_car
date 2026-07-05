#![no_std]
#![no_main]

mod esp_now_transport;
mod motor;
mod usb_link;

use common_comms::espnow::{LinkError, VehicleLink};
use common_comms::protocol::ControlPacket;
use common_comms::{LinkState, LinkWatchdog, CONTROL_PACKET_LEN, LINK_TIMEOUT_MS};
use common_host_proto::{BoardToHost, HostToBoard, LinkStateKind};
use common_led::{LED_BUFFER_SIZE, LedPulseCode, Ws2812Led};
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
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

// ── Pure-logic loop state ─────────────────────────────────────────────────────

struct VehicleRunState {
    elapsed_ms: u64,
    last_state: LinkState,
    last_button: u8,
    tick: u64,
    led_toggle: bool,
    led_override: Option<[u8; 3]>,
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
        }
    }
}

// ── Composition root ──────────────────────────────────────────────────────────

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let p = esp_hal::init(esp_hal::Config::default());

    esp_alloc::heap_allocator!(size: 64 * 1024);

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
    let motor = motor::Ibt2Motor::new(rpwm, lpwm, r_en, l_en);

    let mut led_buf = common_led::ws2812_buffer!();

    // ── Transport + LED setup ─────────────────────────────────────────────────

    let (link, led, rx_buf) = setup(esp_now, p.RMT, p.GPIO8, &mut led_buf).await;

    // ── Main loop ─────────────────────────────────────────────────────────────

    run(link, motor, led, rx_buf).await
}

// ── Hardware setup (runtime · transport · LED) ────────────────────────────────
//
// Configures the Embassy runtime, wraps ESP-NOW in the vehicle transport, and
// initialises the WS2812B LED.  All radio/WiFi objects are anchored in main()
// because EspNow borrows from esp_radio_ctrl for its lifetime.

async fn setup<'radio, 'led>(
    esp_now: esp_radio::esp_now::EspNow<'radio>,
    rmt: esp_hal::peripherals::RMT<'static>,
    gpio8: esp_hal::peripherals::GPIO8<'static>,
    led_buf: &'led mut [LedPulseCode; LED_BUFFER_SIZE],
) -> (
    VehicleLink<esp_now_transport::Esp32C6EspNow<'radio>>,
    Ws2812Led<'led, { LED_BUFFER_SIZE }>,
    [u8; CONTROL_PACKET_LEN],
) {
    esp_now
        .set_channel(1)
        .expect("Failed to set ESP-NOW channel");
    let transport = esp_now_transport::Esp32C6EspNow::new(esp_now);
    let link = VehicleLink::new(transport);
    let rx_buf = [0u8; CONTROL_PACKET_LEN];

    let rmt = Rmt::new(rmt, Rate::from_mhz(80)).expect("Failed to initialize RMT");
    let mut led = common_led::new_ws2812(rmt.channel0, gpio8, led_buf);
    let _ = common_led::set_rgb(&mut led, 16, 16, 0); // yellow boot colour

    (link, led, rx_buf)
}

// ── Main control loop ─────────────────────────────────────────────────────────

async fn run<'radio, 'd, 'led>(
    mut link: VehicleLink<esp_now_transport::Esp32C6EspNow<'radio>>,
    mut motor: motor::Ibt2Motor<'d>,
    mut led: Ws2812Led<'led, { LED_BUFFER_SIZE }>,
    mut rx_buf: [u8; CONTROL_PACKET_LEN],
) -> ! {
    let mut watchdog = LinkWatchdog::new(LINK_TIMEOUT_MS);
    let mut s = VehicleRunState::new();

    loop {
        s.tick = s.tick.wrapping_add(1);

        // --- ESP-NOW receive + motor actuation ---
        match link.try_receive_control(&mut rx_buf) {
            Ok(Some(received)) => {
                watchdog.record_valid_packet(s.elapsed_ms);

                let duty: i16 = if received.packet.buttons & ControlPacket::BUTTON_C != 0 {
                    motor.brake();
                    0
                } else {
                    motor.set_drive(received.packet.y);
                    motor::y_to_duty(received.packet.y)
                };
                let _ = usb_link::EVENTS.try_send(BoardToHost::ReceivedPacket {
                    x: received.packet.x,
                    y: received.packet.y,
                    buttons: received.packet.buttons,
                });
                let _ = usb_link::EVENTS.try_send(BoardToHost::MotorState { duty });

                let tracked = ControlPacket::BUTTON_A
                    | ControlPacket::BUTTON_B
                    | ControlPacket::BUTTON_C
                    | ControlPacket::BUTTON_D;
                if received.packet.buttons & tracked != 0 {
                    s.last_button = received.packet.buttons & tracked;
                }
            }
            Ok(None) => {}
            Err(LinkError::StaleSequence) => {}
            Err(_) => {}
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
                    // take effect immediately while timed-out.  This is intentional
                    // — a computer may take over direct motor control via USB cable.
                    motor.brake();
                    LinkStateKind::TimedOut
                }
            };
            let _ = usb_link::EVENTS.try_send(BoardToHost::EspNowLinkState(kind));
            s.last_state = link_state;
        }

        // --- USB command drain ---
        while let Ok(cmd) = usb_link::CMDS.try_receive() {
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
                    motor.set_pwm(duty);
                }
                _ => {}
            }
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
