#![no_std]

pub mod espnow;
pub mod keepalive;
pub mod protocol;

pub use keepalive::{LinkState, LinkWatchdog};
pub use protocol::{CONTROL_PACKET_LEN, CONTROL_TX_INTERVAL_MS, ControlPacket, LINK_TIMEOUT_MS, PacketError};
