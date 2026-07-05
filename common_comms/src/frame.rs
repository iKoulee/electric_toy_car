//! Typed ESP-NOW frame envelope shared by controller and vehicle.
//!
//! Every ESP-NOW payload now begins with a one-byte [`FrameKind`] discriminant;
//! the remaining bytes are the kind-specific body. This multiplexes the periodic
//! control stream with the bidirectional USB-host tunnel and the pairing
//! handshake over a single ESP-NOW link.
//!
//! Wire layout: `[kind: u8][body: ..]`
//!
//! - [`FrameKind::Control`] — body is the 8-byte [`ControlPacket`](crate::protocol::ControlPacket)
//!   (see [`crate::protocol::CONTROL_PACKET_LEN`]). Only these frames drive the
//!   link watchdog and sequence-freshness logic.
//! - [`FrameKind::TunnelCmd`] — body is opaque postcard bytes of a host→board
//!   message, relayed by the gateway board to the remote board.
//! - [`FrameKind::TunnelEvt`] — body is opaque postcard bytes of a board→host
//!   telemetry message, sent by the remote board back to the gateway.
//! - [`FrameKind::PairAck`] — an empty marker sent (unicast) during pairing to
//!   confirm receipt; the recipient learns the sender's MAC from the frame's
//!   source address, so both boards can switch from broadcast to unicast.
//!
//! Discriminants are the wire format — append-only, never reorder.

use crate::protocol::CONTROL_PACKET_LEN;

/// Largest frame body this protocol emits. Tunnelled host-protocol messages are
/// the biggest; sized with headroom and well under the 250-byte ESP-NOW limit.
pub const MAX_FRAME_BODY: usize = 64;

/// Maximum encoded frame size: one discriminant byte plus the largest body.
pub const MAX_ENCODED_FRAME: usize = 1 + MAX_FRAME_BODY;

/// Wire discriminant for the ESP-NOW frame envelope.
///
/// The `u8` values are the wire format and must remain stable and append-only.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum FrameKind {
    /// Periodic joystick control packet (drives the watchdog/fail-safe).
    Control = 0x01,
    /// Tunnelled host→board command bytes, relayed to the remote board.
    TunnelCmd = 0x02,
    /// Tunnelled board→host telemetry bytes, sent from the remote board.
    TunnelEvt = 0x03,
    /// Empty pairing acknowledgement; recipient learns the MAC from the source.
    PairAck = 0x04,
}

impl FrameKind {
    fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x01 => Some(FrameKind::Control),
            0x02 => Some(FrameKind::TunnelCmd),
            0x03 => Some(FrameKind::TunnelEvt),
            0x04 => Some(FrameKind::PairAck),
            _ => None,
        }
    }
}

/// A decoded ESP-NOW frame borrowing its body from the receive buffer.
///
/// `Control`/`TunnelCmd`/`TunnelEvt` bodies are returned as slices into the
/// caller's buffer; `PairAck` is copied out because it is a fixed small value.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Frame<'a> {
    Control(&'a [u8]),
    TunnelCmd(&'a [u8]),
    TunnelEvt(&'a [u8]),
    PairAck,
}

/// Errors from envelope encode/decode.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum FrameError {
    /// Received buffer was empty (no discriminant byte).
    Empty,
    /// Discriminant byte did not match a known [`FrameKind`].
    UnknownKind,
    /// A `Control` body was not [`CONTROL_PACKET_LEN`] bytes.
    BadControlLength,
    /// The body did not fit the destination buffer during encode.
    BufferTooSmall,
}

/// Encode `[kind][body]` into `out`, returning the number of bytes written.
///
/// Used by the link layer for tunnel and pairing frames; control frames use the
/// [`encode_control`] convenience wrapper.
pub fn encode_frame(kind: FrameKind, body: &[u8], out: &mut [u8]) -> Result<usize, FrameError> {
    let total = 1 + body.len();
    if out.len() < total {
        return Err(FrameError::BufferTooSmall);
    }
    out[0] = kind as u8;
    out[1..total].copy_from_slice(body);
    Ok(total)
}

/// Encode a control packet body as a [`FrameKind::Control`] frame.
pub fn encode_control(
    packet_bytes: &[u8; CONTROL_PACKET_LEN],
    out: &mut [u8],
) -> Result<usize, FrameError> {
    encode_frame(FrameKind::Control, packet_bytes, out)
}

/// Decode a raw ESP-NOW payload into a borrowed [`Frame`].
pub fn decode_frame(buf: &[u8]) -> Result<Frame<'_>, FrameError> {
    let (&kind_byte, body) = buf.split_first().ok_or(FrameError::Empty)?;
    let kind = FrameKind::from_u8(kind_byte).ok_or(FrameError::UnknownKind)?;
    match kind {
        FrameKind::Control => {
            if body.len() != CONTROL_PACKET_LEN {
                return Err(FrameError::BadControlLength);
            }
            Ok(Frame::Control(body))
        }
        FrameKind::TunnelCmd => Ok(Frame::TunnelCmd(body)),
        FrameKind::TunnelEvt => Ok(Frame::TunnelEvt(body)),
        // The body (if any) is ignored; the MAC is learned from the source.
        FrameKind::PairAck => Ok(Frame::PairAck),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::ControlPacket;

    #[test]
    fn control_roundtrip() {
        let packet = ControlPacket::new(0x1234, 10, 240, ControlPacket::BUTTON_A);
        let bytes = packet.to_bytes();
        let mut out = [0u8; MAX_ENCODED_FRAME];
        let n = encode_control(&bytes, &mut out).unwrap();
        assert_eq!(n, 1 + CONTROL_PACKET_LEN);
        assert_eq!(out[0], FrameKind::Control as u8);

        match decode_frame(&out[..n]).unwrap() {
            Frame::Control(body) => {
                assert_eq!(ControlPacket::from_bytes(body).unwrap(), packet);
            }
            other => panic!("expected Control, got {other:?}"),
        }
    }

    #[test]
    fn tunnel_roundtrip() {
        let payload = [0xDE, 0xAD, 0xBE, 0xEF];
        let mut out = [0u8; MAX_ENCODED_FRAME];

        let n = encode_frame(FrameKind::TunnelCmd, &payload, &mut out).unwrap();
        assert_eq!(decode_frame(&out[..n]).unwrap(), Frame::TunnelCmd(&payload));

        let n = encode_frame(FrameKind::TunnelEvt, &payload, &mut out).unwrap();
        assert_eq!(decode_frame(&out[..n]).unwrap(), Frame::TunnelEvt(&payload));
    }

    #[test]
    fn pair_ack_roundtrip() {
        let mut out = [0u8; MAX_ENCODED_FRAME];
        let n = encode_frame(FrameKind::PairAck, &[], &mut out).unwrap();
        assert_eq!(n, 1);
        assert_eq!(decode_frame(&out[..n]).unwrap(), Frame::PairAck);
    }

    #[test]
    fn rejects_empty_and_unknown() {
        assert_eq!(decode_frame(&[]), Err(FrameError::Empty));
        assert_eq!(decode_frame(&[0x00]), Err(FrameError::UnknownKind));
        assert_eq!(decode_frame(&[0xFF, 1, 2]), Err(FrameError::UnknownKind));
    }

    #[test]
    fn rejects_bad_control_length() {
        // Control body must be exactly CONTROL_PACKET_LEN.
        let mut out = [0u8; MAX_ENCODED_FRAME];
        let n = encode_frame(FrameKind::Control, &[1, 2, 3], &mut out).unwrap();
        assert_eq!(decode_frame(&out[..n]), Err(FrameError::BadControlLength));
    }

    #[test]
    fn encode_rejects_small_buffer() {
        let mut out = [0u8; 2];
        assert_eq!(
            encode_frame(FrameKind::TunnelCmd, &[1, 2, 3, 4], &mut out),
            Err(FrameError::BufferTooSmall)
        );
    }
}
