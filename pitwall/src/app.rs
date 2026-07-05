//! Application state: the telemetry model both boards feed into, the scrolling
//! event log, and the command-input buffer. `ingest` demultiplexes each decoded
//! `BoardToHost` message into per-board state and history for the TUI to render.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use common_host_proto::{decode_board_payload, BoardKind, BoardToHost, LinkStateKind};
use ratatui::style::Color;

/// Samples kept per sparkline (≈ last N telemetry updates).
const HIST_LEN: usize = 120;
/// Maximum retained event-log lines.
const LOG_LEN: usize = 500;
/// A board panel is considered "stale" if nothing arrived within this window.
pub const STALE_AFTER: Duration = Duration::from_millis(1500);

/// Joystick button bit masks — mirrors `common_comms/src/protocol.rs` BUTTON_*.
/// Duplicated (rather than depending on the embedded crate) to keep this a plain
/// host build.
pub const BUTTONS: [(&str, u8); 5] = [
    ("JOY", 1 << 0),
    ("C", 1 << 1),
    ("A", 1 << 2),
    ("B", 1 << 3),
    ("D", 1 << 4),
];

/// Rolling telemetry for a single board.
#[derive(Default)]
pub struct BoardTelemetry {
    pub link: Option<LinkStateKind>,
    pub joy_x: Option<u8>,
    pub joy_y: Option<u8>,
    pub buttons: u8,
    pub motor_duty: Option<i16>,
    pub last_seen: Option<Instant>,
    pub hist_x: VecDeque<u64>,
    pub hist_y: VecDeque<u64>,
    /// Motor duty shifted into 0..=200 (duty + 100) so a sparkline can show it.
    pub hist_duty: VecDeque<u64>,
}

impl BoardTelemetry {
    fn push_hist(buf: &mut VecDeque<u64>, v: u64) {
        if buf.len() >= HIST_LEN {
            buf.pop_front();
        }
        buf.push_back(v);
    }

    pub fn is_stale(&self, now: Instant) -> bool {
        self.last_seen
            .is_none_or(|t| now.duration_since(t) > STALE_AFTER)
    }
}

pub struct AppState {
    pub controller: BoardTelemetry,
    pub vehicle: BoardTelemetry,
    /// Which board is directly on USB (learned from `Pong`).
    pub gateway: Option<BoardKind>,
    pub port_name: String,
    pub connected: bool,
    pub error_count: u32,
    pub log: VecDeque<(String, Color)>,
    pub input: String,
    pub should_quit: bool,
    pub show_help: bool,
}

/// Help text shown in the F1 popup.
pub const HELP: &[&str] = &[
    "Commands — type, then press Enter:",
    "",
    "  ping                    ping the connected board",
    "  led R G B  /  led off   LED override / restore auto",
    "  motor_en R_EN L_EN      vehicle: enable pins (on/off)",
    "  motor_pwm -100..100     vehicle: set motor PWM",
    "  remote_tele on|off      stream this board's telemetry",
    "  repair                  forget pairing, re-handshake",
    "  peer <cmd>              run <cmd> on the paired peer",
    "  help / h / ?            show this help",
    "  quit / q / Esc          exit",
    "",
    "Tip: the Vehicle panel stays empty until the vehicle",
    "streams telemetry over the tunnel. Enable it with:",
    "    peer remote_tele on",
    "",
    "F1 or any key: close this help",
];

impl AppState {
    pub fn new(port_name: String) -> Self {
        Self {
            controller: BoardTelemetry::default(),
            vehicle: BoardTelemetry::default(),
            gateway: None,
            port_name,
            connected: false,
            error_count: 0,
            log: VecDeque::new(),
            input: String::new(),
            should_quit: false,
            show_help: false,
        }
    }

    pub fn board_mut(&mut self, kind: BoardKind) -> &mut BoardTelemetry {
        match kind {
            BoardKind::Controller => &mut self.controller,
            BoardKind::Vehicle => &mut self.vehicle,
        }
    }

    pub fn push_log(&mut self, line: String, color: Color) {
        if self.log.len() >= LOG_LEN {
            self.log.pop_front();
        }
        self.log.push_back((line, color));
    }

    /// Consume one decoded board→host message: append to the event log and
    /// update the corresponding board's live telemetry + history.
    pub fn ingest(&mut self, msg: &BoardToHost) {
        let (line, color) = board_msg_line(msg);
        self.push_log(line, color);

        match msg {
            BoardToHost::Pong { board, .. } => {
                self.gateway = Some(*board);
                self.connected = true;
            }
            BoardToHost::Error(_) => self.error_count += 1,
            BoardToHost::FromPeer { source, payload } => {
                if let Ok(inner) = decode_board_payload(payload) {
                    self.apply_telemetry(*source, &inner);
                }
            }
            // Directly-connected telemetry is attributed to the gateway board.
            other => {
                if let Some(kind) = self.gateway {
                    self.apply_telemetry(kind, other);
                }
            }
        }
    }

    fn apply_telemetry(&mut self, kind: BoardKind, msg: &BoardToHost) {
        let now = Instant::now();
        let b = self.board_mut(kind);
        b.last_seen = Some(now);
        match msg {
            BoardToHost::JoystickState { x, y, buttons }
            | BoardToHost::ReceivedPacket { x, y, buttons } => {
                b.joy_x = Some(*x);
                b.joy_y = Some(*y);
                b.buttons = *buttons;
                BoardTelemetry::push_hist(&mut b.hist_x, *x as u64);
                BoardTelemetry::push_hist(&mut b.hist_y, *y as u64);
            }
            BoardToHost::MotorState { duty } => {
                b.motor_duty = Some(*duty);
                let shifted = (*duty as i64 + 100).clamp(0, 200) as u64;
                BoardTelemetry::push_hist(&mut b.hist_duty, shifted);
            }
            BoardToHost::EspNowLinkState(state) => b.link = Some(*state),
            _ => {}
        }
    }
}

pub fn board_kind_str(board: &BoardKind) -> &'static str {
    match board {
        BoardKind::Controller => "Controller",
        BoardKind::Vehicle => "Vehicle",
    }
}

pub fn link_str(state: &LinkStateKind) -> &'static str {
    match state {
        LinkStateKind::AwaitingFirstPacket => "AwaitingFirstPacket",
        LinkStateKind::Alive => "Alive",
        LinkStateKind::TimedOut => "TimedOut",
    }
}

/// Colour a link state for panel headers.
pub fn link_color(state: Option<LinkStateKind>) -> Color {
    match state {
        Some(LinkStateKind::Alive) => Color::Green,
        Some(LinkStateKind::AwaitingFirstPacket) => Color::Yellow,
        Some(LinkStateKind::TimedOut) => Color::Red,
        None => Color::DarkGray,
    }
}

/// Render a board message as a (text, colour) pair for the event log — parity
/// with the original line-oriented output, including recursive `FromPeer`.
pub fn board_msg_line(msg: &BoardToHost) -> (String, Color) {
    match msg {
        BoardToHost::Pong { version, board } => (
            format!("[PONG] version={version} board={}", board_kind_str(board)),
            Color::Green,
        ),
        BoardToHost::JoystickState { x, y, buttons } => (
            format!("[JOYSTICK] x={x} y={y} buttons={buttons:#04x}"),
            Color::Cyan,
        ),
        BoardToHost::EspNowLinkState(state) => {
            (format!("[LINK] {}", link_str(state)), Color::Yellow)
        }
        BoardToHost::ReceivedPacket { x, y, buttons } => (
            format!("[PACKET] x={x} y={y} buttons={buttons:#04x}"),
            Color::Cyan,
        ),
        BoardToHost::MotorState { duty } => (format!("[MOTOR] duty={duty}"), Color::Blue),
        BoardToHost::LedAck => ("[LED_ACK]".to_string(), Color::Green),
        BoardToHost::Error(e) => (format!("[ERROR] {e:?}"), Color::Red),
        BoardToHost::FromPeer { source, payload } => {
            let src = board_kind_str(source);
            match decode_board_payload(payload) {
                Ok(inner) => {
                    let (line, color) = board_msg_line(&inner);
                    (format!("[{src}] {line}"), color)
                }
                Err(_) => (format!("[{src}] <undecodable telemetry>"), Color::DarkGray),
            }
        }
        _ => (format!("[UNKNOWN] {msg:?}"), Color::DarkGray),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common_host_proto::{encode_board_payload, RelayPayload, RELAY_PAYLOAD_MAX};

    fn relay(msg: &BoardToHost) -> RelayPayload {
        let mut buf = [0u8; RELAY_PAYLOAD_MAX];
        let n = encode_board_payload(msg, &mut buf).unwrap();
        RelayPayload::from_slice(&buf[..n]).unwrap()
    }

    #[test]
    fn direct_telemetry_attributed_to_gateway() {
        let mut app = AppState::new("test".into());
        app.ingest(&BoardToHost::Pong { version: 1, board: BoardKind::Vehicle });
        app.ingest(&BoardToHost::MotorState { duty: 42 });

        assert_eq!(app.vehicle.motor_duty, Some(42));
        // duty 42 → shifted 142 in the sparkline history.
        assert_eq!(app.vehicle.hist_duty.back().copied(), Some(142));
        assert!(app.controller.motor_duty.is_none());
    }

    #[test]
    fn from_peer_routes_to_source_board() {
        let mut app = AppState::new("test".into());
        app.ingest(&BoardToHost::Pong { version: 1, board: BoardKind::Vehicle });

        let payload = relay(&BoardToHost::JoystickState { x: 200, y: 60, buttons: 0b0000_0101 });
        app.ingest(&BoardToHost::FromPeer { source: BoardKind::Controller, payload });

        assert_eq!(app.controller.joy_x, Some(200));
        assert_eq!(app.controller.joy_y, Some(60));
        assert_eq!(app.controller.buttons, 0b0000_0101);
        assert_eq!(app.controller.hist_x.back().copied(), Some(200));
    }

    #[test]
    fn errors_are_counted() {
        let mut app = AppState::new("test".into());
        app.ingest(&BoardToHost::Error(common_host_proto::HostError::Busy));
        assert_eq!(app.error_count, 1);
    }
}
