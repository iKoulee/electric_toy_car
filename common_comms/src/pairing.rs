//! Persistent pairing record: the paired peer MAC survives reboots so the link
//! comes up in unicast immediately without re-running the broadcast handshake.
//!
//! This module is the pure, host-testable serialization and integrity layer.
//! The actual flash I/O (locating the `nvs` partition, read/write) is
//! board-specific and lives in each firmware crate, because it depends on
//! `esp-hal`/`esp-storage`.
//!
//! Record layout ([`PAIRING_RECORD_LEN`] bytes):
//! `[magic: u32 LE][mac: 6][crc16: u16 LE]`, where the CRC covers the magic and
//! MAC bytes. A record whose magic or CRC does not validate is treated as
//! "unpaired" — this also naturally covers erased flash (all `0xFF`).

/// Total length of a serialized pairing record.
pub const PAIRING_RECORD_LEN: usize = 12;

/// Marker identifying a valid record ("PAIR" in ASCII).
const MAGIC: u32 = 0x5041_4952;

/// Serialize a paired peer MAC into a fixed-size record.
pub fn serialize(mac: [u8; 6]) -> [u8; PAIRING_RECORD_LEN] {
    let mut rec = [0u8; PAIRING_RECORD_LEN];
    rec[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    rec[4..10].copy_from_slice(&mac);
    let crc = crc16_ccitt(&rec[0..10]);
    rec[10..12].copy_from_slice(&crc.to_le_bytes());
    rec
}

/// Parse a pairing record, returning the MAC only if magic and CRC validate.
pub fn deserialize(bytes: &[u8]) -> Option<[u8; 6]> {
    if bytes.len() < PAIRING_RECORD_LEN {
        return None;
    }
    let magic = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    if magic != MAGIC {
        return None;
    }
    let stored_crc = u16::from_le_bytes([bytes[10], bytes[11]]);
    if crc16_ccitt(&bytes[0..10]) != stored_crc {
        return None;
    }
    let mut mac = [0u8; 6];
    mac.copy_from_slice(&bytes[4..10]);
    Some(mac)
}

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). Small and adequate for a
/// 10-byte record written rarely; not a security measure.
fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let mac = [0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC];
        assert_eq!(deserialize(&serialize(mac)), Some(mac));
    }

    #[test]
    fn erased_flash_is_unpaired() {
        assert_eq!(deserialize(&[0xFF; PAIRING_RECORD_LEN]), None);
        assert_eq!(deserialize(&[0x00; PAIRING_RECORD_LEN]), None);
    }

    #[test]
    fn rejects_bad_crc() {
        let mut rec = serialize([1, 2, 3, 4, 5, 6]);
        rec[5] ^= 0xFF; // corrupt a MAC byte, leave CRC stale
        assert_eq!(deserialize(&rec), None);
    }

    #[test]
    fn rejects_short_buffer() {
        assert_eq!(deserialize(&[0u8; 4]), None);
    }
}
