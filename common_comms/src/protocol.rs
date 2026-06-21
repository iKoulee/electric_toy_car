pub const CONTROL_PACKET_LEN: usize = 7;
pub const CONTROL_TX_INTERVAL_MS: u64 = 100;
pub const LINK_TIMEOUT_MS: u64 = 500;

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(C, packed)]
pub struct ControlPacket {
    pub sequence: u16,
    pub x: u8,
    pub y: u8,
    pub buttons: u8,
    pub reserved: u8,
    pub checksum: u8,
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum PacketError {
    InvalidLength,
    BadChecksum,
}

impl ControlPacket {
    pub const BUTTON_JOY: u8 = 1 << 0;
    pub const BUTTON_C: u8 = 1 << 1;
    pub const BUTTON_A: u8 = 1 << 2;
    pub const BUTTON_B: u8 = 1 << 3;
    pub const BUTTON_D: u8 = 1 << 4;

    pub fn new(sequence: u16, x: u8, y: u8, buttons: u8) -> Self {
        let mut packet = Self {
            sequence,
            x,
            y,
            buttons,
            reserved: 0,
            checksum: 0,
        };
        packet.checksum = packet.compute_checksum();
        packet
    }

    pub fn compute_checksum(&self) -> u8 {
        let [seq_lo, seq_hi] = self.sequence.to_le_bytes();
        seq_lo ^ seq_hi ^ self.x ^ self.y ^ self.buttons ^ self.reserved
    }

    pub fn to_bytes(self) -> [u8; CONTROL_PACKET_LEN] {
        let [seq_lo, seq_hi] = self.sequence.to_le_bytes();
        [
            seq_lo,
            seq_hi,
            self.x,
            self.y,
            self.buttons,
            self.reserved,
            self.checksum,
        ]
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, PacketError> {
        if bytes.len() != CONTROL_PACKET_LEN {
            return Err(PacketError::InvalidLength);
        }

        let packet = Self {
            sequence: u16::from_le_bytes([bytes[0], bytes[1]]),
            x: bytes[2],
            y: bytes[3],
            buttons: bytes[4],
            reserved: bytes[5],
            checksum: bytes[6],
        };

        if packet.compute_checksum() != packet.checksum {
            return Err(PacketError::BadChecksum);
        }

        Ok(packet)
    }
}

pub fn is_newer_sequence(last_sequence: u16, candidate: u16) -> bool {
    let delta = candidate.wrapping_sub(last_sequence);
    delta != 0 && delta < 0x8000
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packet_roundtrip() {
        let packet = ControlPacket::new(0x1234, 128, 200, ControlPacket::BUTTON_A | ControlPacket::BUTTON_B);
        let bytes = packet.to_bytes();
        assert_eq!(bytes.len(), CONTROL_PACKET_LEN);

        let decoded = ControlPacket::from_bytes(&bytes).unwrap();
        assert_eq!(decoded, packet);
    }

    #[test]
    fn sequence_wraparound_is_newer() {
        assert!(is_newer_sequence(65535, 0));
        assert!(!is_newer_sequence(10, 10));
        assert!(!is_newer_sequence(10, 9));
    }
}
