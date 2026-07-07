//! Parsing of the interactive text commands into `HostToBoard` messages.
//!
//! Moved verbatim from the original single-file tool; the command grammar is
//! unchanged so muscle memory (`motor_pwm 40`, `peer led 0 255 0`, …) still works.

use common_host_proto::{
    encode_host_payload, HostToBoard, RelayPayload, PROTOCOL_VERSION, RELAY_PAYLOAD_MAX,
};

fn parse_bool(s: &str) -> Option<bool> {
    match s {
        "1" | "true" | "on" => Some(true),
        "0" | "false" | "off" => Some(false),
        _ => None,
    }
}

/// Parse one interactive command line into a `HostToBoard` message.
/// Returns `None` for an unrecognised or malformed command.
pub fn parse_command(input: &str) -> Option<HostToBoard> {
    let parts: Vec<&str> = input.split_whitespace().collect();
    match parts.as_slice() {
        ["ping"] => Some(HostToBoard::Ping { version: PROTOCOL_VERSION }),
        ["led", "off"] => Some(HostToBoard::SetLed(None)),
        ["led", r, g, b] => {
            let r = r.parse::<u8>().ok()?;
            let g = g.parse::<u8>().ok()?;
            let b = b.parse::<u8>().ok()?;
            Some(HostToBoard::SetLed(Some([r, g, b])))
        }
        ["motor_en", r, l] => {
            let r_en = parse_bool(r)?;
            let l_en = parse_bool(l)?;
            Some(HostToBoard::SetMotorEnable { r_en, l_en })
        }
        ["motor_pwm", d] => {
            let duty = d.parse::<i8>().ok()?.clamp(-100, 100);
            Some(HostToBoard::SetMotorPwm { duty })
        }
        ["remote_tele", s] => Some(HostToBoard::EnableRemoteTelemetry { on: parse_bool(s)? }),
        ["repair"] => Some(HostToBoard::Repair),
        // Relay any other command to the peer board via the gateway: parse the
        // remainder recursively and wrap its raw encoding in ForPeer.
        ["peer", rest @ ..] if !rest.is_empty() => {
            let inner = parse_command(&rest.join(" "))?;
            let mut buf = [0u8; RELAY_PAYLOAD_MAX];
            let n = encode_host_payload(&inner, &mut buf).ok()?;
            let payload = RelayPayload::from_slice(&buf[..n]).ok()?;
            Some(HostToBoard::ForPeer(payload))
        }
        _ => None,
    }
}
