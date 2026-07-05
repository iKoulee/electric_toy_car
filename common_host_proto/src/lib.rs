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
    /// Positive → RPWM active, LPWM=0. Negative → LPWM active, RPWM=0. Zero → both 0 (coast).
    SetMotorPwm { duty: i8 },
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
