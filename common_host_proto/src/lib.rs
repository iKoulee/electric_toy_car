//! Bidirectional USB host protocol shared by controller and vehicle boards.
//!
//! Wire format: each message is COBS-encoded postcard, terminated by 0x00.
//! Variant ordinals are the wire discriminants — append-only, never reorder.
//!
//! Handshake: host sends Ping; board replies Pong identifying itself.
#![no_std]

use serde::{Deserialize, Serialize};

pub const PROTOCOL_VERSION: u8 = 1;
/// Maximum encoded frame size including COBS overhead and 0x00 terminator.
pub const MAX_FRAME_BYTES: usize = 64;

/// Maximum length of a relayed (tunnelled) payload carried inside
/// [`HostToBoard::ForPeer`] / [`BoardToHost::FromPeer`]. Sized well above the
/// current message set while keeping a wrapped frame within [`MAX_FRAME_BYTES`].
pub const RELAY_PAYLOAD_MAX: usize = 48;

/// A relayed host-protocol message serialized as opaque bytes.
///
/// The bytes are a **non-COBS** postcard encoding of a [`HostToBoard`]
/// (in `ForPeer`) or [`BoardToHost`] (in `FromPeer`) — COBS framing is only for
/// the USB byte stream, so the tunnel carries the raw payload. Build/consume it
/// with [`encode_host_payload`]/[`decode_host_payload`] and the `_board_`
/// variants.
pub type RelayPayload = heapless::Vec<u8, RELAY_PAYLOAD_MAX>;

/// Messages from host → board.
/// Boards return `Error::NotApplicable` for variants they don't support.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub enum HostToBoard {
    Ping { version: u8 },
    /// Override the onboard LED. `None` restores automatic state-driven color.
    SetLed(Option<[u8; 3]>),
    /// Vehicle only: directly set IBT-2 R_EN and L_EN enable pins.
    SetMotorEnable { r_en: bool, l_en: bool },
    /// Vehicle only: directly set IBT-2 PWM duty (-100–100).
    /// Positive → RPWM active, LPWM=0. Negative → LPWM active, RPWM=0. Zero with
    /// the enables high is an electrodynamic brake, not a coast.
    SetMotorPwm { duty: i8 },
    /// Relay an opaque host→board message to the paired peer board over the
    /// ESP-NOW tunnel. The gateway forwards the bytes verbatim; the remote board
    /// decodes them as a [`HostToBoard`] and executes it locally.
    ForPeer(RelayPayload),
    /// Enable or disable streaming this board's telemetry to the peer over the
    /// tunnel. Off by default to save airtime and controller battery; typically
    /// sent to the remote board wrapped in [`HostToBoard::ForPeer`].
    EnableRemoteTelemetry { on: bool },
    /// Forget the stored pairing and re-run the pairing handshake. Wrap in
    /// [`HostToBoard::ForPeer`] to re-pair the remote board.
    Repair,
    /// Vehicle only: when `on`, a latched manual PWM override ([`SetMotorPwm`]) is
    /// fed through the same slew-rate limiter (ramp) as joystick drive instead of
    /// applied instantly. Off by default so the host sees the exact duty it commands.
    ///
    /// [`SetMotorPwm`]: HostToBoard::SetMotorPwm
    SetManualPwmRamp { on: bool },
}

/// Messages from board → host.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub enum BoardToHost {
    Pong { version: u8, board: BoardKind },
    /// Controller only: current joystick reading.
    JoystickState { x: u8, y: u8, buttons: u8 },
    /// Both boards: ESP-NOW link state change.
    EspNowLinkState(LinkStateKind),
    /// Vehicle only: last received control packet.
    ReceivedPacket { x: u8, y: u8, buttons: u8 },
    /// Vehicle only: current motor drive level (positive = forward, negative = reverse).
    MotorState { duty: i16 },
    LedAck,
    Error(HostError),
    /// Telemetry relayed from the peer board through the gateway. `payload` is an
    /// opaque non-COBS postcard [`BoardToHost`]; decode with
    /// [`decode_board_payload`]. `source` identifies which board produced it.
    FromPeer {
        source: BoardKind,
        payload: RelayPayload,
    },
    /// Vehicle only: IBT-2 current-sense readings in milliamps (offset-subtracted,
    /// scaled). `r_ma` = right/`R_IS` channel, `l_ma` = left/`L_IS` channel.
    /// NOTE: the channel↔drive-direction mapping and the mA scale are **not yet
    /// verified** — see the calibration procedure in `vehicle/src/ibt2.rs`.
    CurrentSense { r_ma: u16, l_ma: u16 },
    /// Vehicle only: raw IBT-2 current-sense diagnostics for calibration — the
    /// averaged IS voltages in millivolts (`r_mv`, `l_mv`, before offset subtraction)
    /// alongside the commanded motor `duty` at sample time. Pair with a multimeter to
    /// derive the offset/scale and confirm the channel↔direction mapping.
    CurrentSenseRaw { r_mv: u16, l_mv: u16, duty: i16 },
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum BoardKind {
    Controller,
    Vehicle,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum LinkStateKind {
    AwaitingFirstPacket,
    Alive,
    TimedOut,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum HostError {
    NotApplicable,
    FrameTooLarge,
    ParseError,
    /// Command channel full; caller should retry.
    Busy,
}

/// Encode a board→host message into `buf` using COBS framing.
/// Returns the number of bytes written (including the trailing 0x00).
pub fn encode_board(msg: &BoardToHost, buf: &mut [u8]) -> Result<usize, postcard::Error> {
    Ok(postcard::to_slice_cobs(msg, buf)?.len())
}

/// Decode a host→board COBS frame (must include the trailing 0x00).
/// Decodes in-place; the slice is mutated during COBS removal.
pub fn decode_host(frame: &mut [u8]) -> Result<HostToBoard, postcard::Error> {
    postcard::from_bytes_cobs(frame)
}

/// Encode a host→board message into `buf` using COBS framing.
/// Returns the number of bytes written (including the trailing 0x00).
pub fn encode_host(msg: &HostToBoard, buf: &mut [u8]) -> Result<usize, postcard::Error> {
    Ok(postcard::to_slice_cobs(msg, buf)?.len())
}

/// Decode a board→host COBS frame (must include the trailing 0x00).
/// Decodes in-place; the slice is mutated during COBS removal.
pub fn decode_board(frame: &mut [u8]) -> Result<BoardToHost, postcard::Error> {
    postcard::from_bytes_cobs(frame)
}

// ── Tunnel payload codecs (non-COBS) ────────────────────────────────────────
//
// The ESP-NOW tunnel carries the raw postcard bytes of a relayed message; COBS
// framing belongs only to the USB byte stream. These encode/decode the opaque
// payload stored in `ForPeer` / `FromPeer`.

/// Encode a host→board message as a raw (non-COBS) tunnel payload.
pub fn encode_host_payload(msg: &HostToBoard, buf: &mut [u8]) -> Result<usize, postcard::Error> {
    Ok(postcard::to_slice(msg, buf)?.len())
}

/// Decode a raw (non-COBS) host→board tunnel payload.
pub fn decode_host_payload(bytes: &[u8]) -> Result<HostToBoard, postcard::Error> {
    postcard::from_bytes(bytes)
}

/// Encode a board→host message as a raw (non-COBS) tunnel payload.
pub fn encode_board_payload(msg: &BoardToHost, buf: &mut [u8]) -> Result<usize, postcard::Error> {
    Ok(postcard::to_slice(msg, buf)?.len())
}

/// Decode a raw (non-COBS) board→host tunnel payload.
pub fn decode_board_payload(bytes: &[u8]) -> Result<BoardToHost, postcard::Error> {
    postcard::from_bytes(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn for_peer_wraps_and_fits_usb_frame() {
        // Inner command the PC wants the remote board to run.
        let inner = HostToBoard::SetMotorPwm { duty: 42 };
        let mut inner_buf = [0u8; RELAY_PAYLOAD_MAX];
        let n = encode_host_payload(&inner, &mut inner_buf).unwrap();
        let payload = RelayPayload::from_slice(&inner_buf[..n]).unwrap();

        // Wrap and COBS-encode for USB; must fit MAX_FRAME_BYTES.
        let mut frame = [0u8; MAX_FRAME_BYTES];
        let framed = encode_host(&HostToBoard::ForPeer(payload), &mut frame).unwrap();
        assert!(framed <= MAX_FRAME_BYTES);

        // Gateway decodes the wrapper, then the remote decodes the inner bytes.
        match decode_host(&mut frame[..framed]).unwrap() {
            HostToBoard::ForPeer(bytes) => {
                assert!(matches!(
                    decode_host_payload(&bytes).unwrap(),
                    HostToBoard::SetMotorPwm { duty: 42 }
                ));
            }
            other => panic!("expected ForPeer, got {other:?}"),
        }
    }

    #[test]
    fn from_peer_roundtrip() {
        let telemetry = BoardToHost::MotorState { duty: -123 };
        let mut buf = [0u8; RELAY_PAYLOAD_MAX];
        let n = encode_board_payload(&telemetry, &mut buf).unwrap();
        let payload = RelayPayload::from_slice(&buf[..n]).unwrap();

        let mut frame = [0u8; MAX_FRAME_BYTES];
        let framed = encode_board(
            &BoardToHost::FromPeer {
                source: BoardKind::Vehicle,
                payload,
            },
            &mut frame,
        )
        .unwrap();
        assert!(framed <= MAX_FRAME_BYTES);

        match decode_board(&mut frame[..framed]).unwrap() {
            BoardToHost::FromPeer { source, payload } => {
                assert_eq!(source, BoardKind::Vehicle);
                assert!(matches!(
                    decode_board_payload(&payload).unwrap(),
                    BoardToHost::MotorState { duty: -123 }
                ));
            }
            other => panic!("expected FromPeer, got {other:?}"),
        }
    }

    #[test]
    fn current_sense_roundtrip() {
        let telemetry = BoardToHost::CurrentSense {
            r_ma: 12_345,
            l_ma: 0,
        };

        // Non-COBS tunnel payload round-trip.
        let mut buf = [0u8; RELAY_PAYLOAD_MAX];
        let n = encode_board_payload(&telemetry, &mut buf).unwrap();
        assert!(matches!(
            decode_board_payload(&buf[..n]).unwrap(),
            BoardToHost::CurrentSense {
                r_ma: 12_345,
                l_ma: 0
            }
        ));

        // COBS USB-frame round-trip; must fit MAX_FRAME_BYTES.
        let mut usb = [0u8; MAX_FRAME_BYTES];
        let framed = encode_board(&telemetry, &mut usb).unwrap();
        assert!(framed <= MAX_FRAME_BYTES);
        assert!(matches!(
            decode_board(&mut usb[..framed]).unwrap(),
            BoardToHost::CurrentSense {
                r_ma: 12_345,
                l_ma: 0
            }
        ));
    }
}
