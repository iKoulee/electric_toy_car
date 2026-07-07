//! Bidirectional ESP-NOW link shared by controller and vehicle.
//!
//! [`EspNowLink`] wraps an [`EspNowTransport`] and multiplexes three concerns
//! over one radio using the [`crate::frame`] envelope:
//!
//! - the periodic **control** stream (sequence-checked, drives the fail-safe),
//! - a bidirectional **tunnel** that relays USB host-protocol bytes between the
//!   two boards (see the gateway design in `docs/espnow-shared-protocol.md`),
//! - the **pairing** handshake that swaps MAC addresses so both boards can move
//!   from broadcast to unicast.
//!
//! The link is transport-agnostic and fully host-testable via a mock
//! [`EspNowTransport`]; the concrete ESP32-C6 adapter lives in each board crate.

use crate::frame::{
    decode_frame, encode_control, encode_frame, Frame, FrameError, FrameKind, MAX_ENCODED_FRAME,
};
use crate::protocol::{is_newer_sequence, ControlPacket, PacketError};

/// The ESP-NOW broadcast address, used only until a peer is paired.
pub const BROADCAST_ADDRESS: [u8; 6] = [0xff; 6];

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct ReceivedMeta {
    /// Source MAC of the frame (the sender).
    pub peer_mac: [u8; 6],
    /// Destination MAC. Equals [`BROADCAST_ADDRESS`] for broadcast frames, which
    /// signals the sender has not paired yet and lets the receiver reply.
    pub dst_mac: [u8; 6],
    pub len: usize,
    pub rssi_dbm: Option<i8>,
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct ReceivedControl {
    pub packet: ControlPacket,
    pub meta: ReceivedMeta,
}

/// Abstraction over the raw ESP-NOW transport.
///
/// Send-to-one-MAC / receive-any, plus peer-list management required for
/// unicast. Implemented by a board-specific adapter over the real radio, and by
/// a mock in host tests.
pub trait EspNowTransport {
    type Error;

    fn send(&mut self, peer_mac: [u8; 6], payload: &[u8]) -> Result<(), Self::Error>;

    fn receive(&mut self, rx_buffer: &mut [u8]) -> Result<Option<ReceivedMeta>, Self::Error>;

    /// Register `mac` as a unicast peer. Must be idempotent: adding an
    /// already-known peer succeeds without error. Required before a unicast
    /// `send` to a learned peer (pairing / tunnel).
    fn add_peer(&mut self, mac: [u8; 6]) -> Result<(), Self::Error>;

    /// Whether `mac` is currently registered as a peer.
    fn peer_exists(&self, mac: [u8; 6]) -> bool;
}

/// Errors from the ESP-NOW link layer.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum LinkError<E> {
    /// Underlying transport error.
    Transport(E),
    /// A control body failed to parse as a [`ControlPacket`].
    Packet(PacketError),
    /// A control packet was a duplicate or older than the last accepted one.
    StaleSequence,
    /// The received frame envelope was malformed.
    Frame(FrameError),
    /// A unicast-only frame (tunnel / pair-ack) was requested while unpaired.
    NotPaired,
}

/// A decoded, demultiplexed inbound frame.
///
/// Tunnel bodies borrow from the caller's receive buffer; `Control` is already
/// parsed and sequence-validated before it is handed back.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Inbound<'a> {
    /// Fresh control packet — the vehicle actuates motors on this.
    Control(ReceivedControl),
    /// Tunnelled host→board command bytes for this board to execute locally.
    TunnelCmd { peer: [u8; 6], bytes: &'a [u8] },
    /// Tunnelled board→host telemetry bytes for this (gateway) board to forward.
    TunnelEvt { peer: [u8; 6], bytes: &'a [u8] },
    /// Pairing acknowledgement announcing the peer's MAC.
    PairAck { peer: [u8; 6] },
}

/// Bidirectional ESP-NOW link with pairing, control, and tunnel support.
pub struct EspNowLink<T: EspNowTransport> {
    transport: T,
    paired_peer: Option<[u8; 6]>,
    last_sequence: Option<u16>,
}

impl<T: EspNowTransport> EspNowLink<T> {
    /// Create an unpaired link.
    pub const fn new(transport: T) -> Self {
        Self {
            transport,
            paired_peer: None,
            last_sequence: None,
        }
    }

    /// The currently paired peer MAC, if any.
    pub fn paired_peer(&self) -> Option<[u8; 6]> {
        self.paired_peer
    }

    /// Whether a peer has been paired (unicast is available).
    pub fn is_paired(&self) -> bool {
        self.paired_peer.is_some()
    }

    /// Register `mac` as the paired peer and add it to the transport peer list.
    ///
    /// Called when a peer MAC is learned from a control packet, a `PairAck`, or
    /// loaded from persistent storage at boot.
    pub fn learn_peer(&mut self, mac: [u8; 6]) -> Result<(), LinkError<T::Error>> {
        self.transport.add_peer(mac).map_err(LinkError::Transport)?;
        self.paired_peer = Some(mac);
        Ok(())
    }

    /// Forget the paired peer and reset sequence tracking (re-pairing). Control
    /// sends fall back to broadcast until a new peer is learned.
    pub fn reset_pairing(&mut self) {
        self.paired_peer = None;
        self.last_sequence = None;
    }

    /// Reset sequence tracking so the next control packet is always accepted.
    /// Used by the vehicle on link timeout.
    pub fn reset_sequence(&mut self) {
        self.last_sequence = None;
    }

    // ── Sending ───────────────────────────────────────────────────────────────

    fn send_frame(
        &mut self,
        dst: [u8; 6],
        kind: FrameKind,
        body: &[u8],
    ) -> Result<(), LinkError<T::Error>> {
        let mut buf = [0u8; MAX_ENCODED_FRAME];
        let n = encode_frame(kind, body, &mut buf).map_err(LinkError::Frame)?;
        self.transport
            .send(dst, &buf[..n])
            .map_err(LinkError::Transport)
    }

    /// Send a control packet: unicast to the paired peer, or broadcast while
    /// still unpaired (pairing bootstrap).
    pub fn send_control(&mut self, packet: ControlPacket) -> Result<(), LinkError<T::Error>> {
        let dst = self.paired_peer.unwrap_or(BROADCAST_ADDRESS);
        let mut buf = [0u8; MAX_ENCODED_FRAME];
        let n = encode_control(&packet.to_bytes(), &mut buf).map_err(LinkError::Frame)?;
        self.transport
            .send(dst, &buf[..n])
            .map_err(LinkError::Transport)
    }

    /// Relay host→board command bytes to the paired peer (gateway → remote).
    pub fn send_tunnel_cmd(&mut self, bytes: &[u8]) -> Result<(), LinkError<T::Error>> {
        let dst = self.paired_peer.ok_or(LinkError::NotPaired)?;
        self.send_frame(dst, FrameKind::TunnelCmd, bytes)
    }

    /// Send board→host telemetry bytes to the paired peer (remote → gateway).
    pub fn send_tunnel_evt(&mut self, bytes: &[u8]) -> Result<(), LinkError<T::Error>> {
        let dst = self.paired_peer.ok_or(LinkError::NotPaired)?;
        self.send_frame(dst, FrameKind::TunnelEvt, bytes)
    }

    /// Send an empty pairing acknowledgement to the paired peer, completing the
    /// handshake. The peer learns our MAC from the frame's source address.
    pub fn send_pair_ack(&mut self) -> Result<(), LinkError<T::Error>> {
        let dst = self.paired_peer.ok_or(LinkError::NotPaired)?;
        self.send_frame(dst, FrameKind::PairAck, &[])
    }

    // ── Receiving ─────────────────────────────────────────────────────────────

    /// Poll for one inbound frame and demultiplex it.
    ///
    /// Only [`Inbound::Control`] frames are sequence-validated and update the
    /// watchdog-relevant state; tunnel and pairing frames bypass freshness.
    /// Returns `Ok(None)` when the receive queue is empty.
    pub fn try_receive<'b>(
        &mut self,
        rx_buffer: &'b mut [u8],
    ) -> Result<Option<Inbound<'b>>, LinkError<T::Error>> {
        let meta = match self
            .transport
            .receive(rx_buffer)
            .map_err(LinkError::Transport)?
        {
            Some(meta) => meta,
            None => return Ok(None),
        };

        match decode_frame(&rx_buffer[..meta.len]).map_err(LinkError::Frame)? {
            Frame::Control(body) => {
                let packet = ControlPacket::from_bytes(body).map_err(LinkError::Packet)?;
                if let Some(last) = self.last_sequence {
                    if !is_newer_sequence(last, packet.sequence) {
                        return Err(LinkError::StaleSequence);
                    }
                }
                self.last_sequence = Some(packet.sequence);
                Ok(Some(Inbound::Control(ReceivedControl { packet, meta })))
            }
            Frame::TunnelCmd(bytes) => Ok(Some(Inbound::TunnelCmd {
                peer: meta.peer_mac,
                bytes,
            })),
            Frame::TunnelEvt(bytes) => Ok(Some(Inbound::TunnelEvt {
                peer: meta.peer_mac,
                bytes,
            })),
            Frame::PairAck => Ok(Some(Inbound::PairAck {
                peer: meta.peer_mac,
            })),
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::frame::encode_control as enc_control;
    use std::vec::Vec;

    const CTRL: [u8; 6] = [0x10, 0, 0, 0, 0, 0x01];
    const VEH: [u8; 6] = [0x20, 0, 0, 0, 0, 0x02];

    #[derive(Default)]
    struct MockTransport {
        inbox: Vec<(Vec<u8>, [u8; 6])>,
        sent: Vec<(Vec<u8>, [u8; 6])>,
        peers: Vec<[u8; 6]>,
    }

    impl MockTransport {
        fn queue_control(&mut self, src: [u8; 6], packet: ControlPacket) {
            let mut buf = [0u8; MAX_ENCODED_FRAME];
            let n = enc_control(&packet.to_bytes(), &mut buf).unwrap();
            self.inbox.push((buf[..n].to_vec(), src));
        }
        fn queue_raw(&mut self, src: [u8; 6], kind: FrameKind, body: &[u8]) {
            let mut buf = [0u8; MAX_ENCODED_FRAME];
            let n = encode_frame(kind, body, &mut buf).unwrap();
            self.inbox.push((buf[..n].to_vec(), src));
        }
    }

    impl EspNowTransport for MockTransport {
        type Error = ();
        fn send(&mut self, peer_mac: [u8; 6], payload: &[u8]) -> Result<(), ()> {
            self.sent.push((payload.to_vec(), peer_mac));
            Ok(())
        }
        fn receive(&mut self, rx_buffer: &mut [u8]) -> Result<Option<ReceivedMeta>, ()> {
            if self.inbox.is_empty() {
                return Ok(None);
            }
            let (frame, src) = self.inbox.remove(0);
            let n = frame.len().min(rx_buffer.len());
            rx_buffer[..n].copy_from_slice(&frame[..n]);
            Ok(Some(ReceivedMeta {
                peer_mac: src,
                dst_mac: BROADCAST_ADDRESS,
                len: n,
                rssi_dbm: None,
            }))
        }
        fn add_peer(&mut self, mac: [u8; 6]) -> Result<(), ()> {
            if !self.peers.contains(&mac) {
                self.peers.push(mac);
            }
            Ok(())
        }
        fn peer_exists(&self, mac: [u8; 6]) -> bool {
            self.peers.contains(&mac)
        }
    }

    #[test]
    fn control_sequence_freshness_applies_only_to_control() {
        let mut t = MockTransport::default();
        t.queue_control(CTRL, ControlPacket::new(1, 0, 0, 0));
        t.queue_control(CTRL, ControlPacket::new(1, 0, 0, 0)); // duplicate → stale
        t.queue_control(CTRL, ControlPacket::new(2, 0, 0, 0)); // newer → ok
        let mut link = EspNowLink::new(t);
        let mut rx = [0u8; MAX_ENCODED_FRAME];

        assert!(matches!(
            link.try_receive(&mut rx).unwrap(),
            Some(Inbound::Control(_))
        ));
        assert_eq!(link.try_receive(&mut rx), Err(LinkError::StaleSequence));
        assert!(matches!(
            link.try_receive(&mut rx).unwrap(),
            Some(Inbound::Control(_))
        ));
    }

    #[test]
    fn tunnel_frames_bypass_sequence_and_demux() {
        let mut t = MockTransport::default();
        t.queue_raw(CTRL, FrameKind::TunnelCmd, &[1, 2, 3]);
        t.queue_raw(VEH, FrameKind::TunnelEvt, &[4, 5]);
        let mut link = EspNowLink::new(t);
        let mut rx = [0u8; MAX_ENCODED_FRAME];

        match link.try_receive(&mut rx).unwrap() {
            Some(Inbound::TunnelCmd { peer, bytes }) => {
                assert_eq!(peer, CTRL);
                assert_eq!(bytes, &[1, 2, 3]);
            }
            other => panic!("expected TunnelCmd, got {other:?}"),
        }
        match link.try_receive(&mut rx).unwrap() {
            Some(Inbound::TunnelEvt { peer, bytes }) => {
                assert_eq!(peer, VEH);
                assert_eq!(bytes, &[4, 5]);
            }
            other => panic!("expected TunnelEvt, got {other:?}"),
        }
    }

    #[test]
    fn control_is_broadcast_until_paired_then_unicast() {
        let mut link = EspNowLink::new(MockTransport::default());

        link.send_control(ControlPacket::new(1, 0, 0, 0)).unwrap();
        link.learn_peer(VEH).unwrap();
        link.send_control(ControlPacket::new(2, 0, 0, 0)).unwrap();

        // Reach into the transport via a second poll-free path: re-borrow.
        // (MockTransport is owned by the link; assert on its recorded sends.)
        let sent = &link.transport.sent;
        assert_eq!(sent[0].1, BROADCAST_ADDRESS);
        assert_eq!(sent[1].1, VEH);
        assert!(link.transport.peers.contains(&VEH));
    }

    #[test]
    fn tunnel_and_pair_ack_require_pairing() {
        let mut link = EspNowLink::new(MockTransport::default());
        assert_eq!(link.send_tunnel_cmd(&[1]), Err(LinkError::NotPaired));
        assert_eq!(link.send_pair_ack(), Err(LinkError::NotPaired));

        link.learn_peer(CTRL).unwrap();
        link.send_pair_ack().unwrap();
        let (body, dst) = link.transport.sent.last().cloned().unwrap();
        assert_eq!(dst, CTRL);
        assert_eq!(decode_frame(&body).unwrap(), Frame::PairAck);
    }
}
