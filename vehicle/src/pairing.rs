//! Board-side flash persistence of the paired peer MAC.
//!
//! Stores a [`common_comms::pairing`] record at the start of the `nvs`
//! partition. [`FlashStorage::write`] performs a safe sector read-erase-write,
//! so the rest of the sector is preserved; the MAC is written only at
//! (re)pairing, so flash wear is negligible.

use common_comms::pairing::{deserialize, serialize, PAIRING_RECORD_LEN};
use embedded_storage::{ReadStorage, Storage};
use esp_bootloader_esp_idf::partitions::{
    read_partition_table, DataPartitionSubType, PartitionType, PARTITION_TABLE_MAX_LEN,
};
use esp_hal::peripherals::FLASH;
use esp_storage::FlashStorage;

/// Owns the flash driver and the resolved `nvs` partition offset.
pub struct PairingStore<'d> {
    flash: FlashStorage<'d>,
    nvs_offset: u32,
}

impl<'d> PairingStore<'d> {
    /// Locate the `nvs` partition and prepare the store. Returns `None` if the
    /// partition table cannot be read or has no `nvs` partition.
    ///
    /// Must be called at most once per boot ([`FlashStorage::new`] panics if
    /// constructed twice).
    pub fn new(flash: FLASH<'d>) -> Option<Self> {
        let mut flash = FlashStorage::new(flash);
        let mut buf = [0u8; PARTITION_TABLE_MAX_LEN];
        let table = read_partition_table(&mut flash, &mut buf).ok()?;
        let entry = table
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .ok()??;
        let nvs_offset = entry.offset();
        Some(Self { flash, nvs_offset })
    }

    /// Load a validated paired MAC, or `None` when unpaired / erased / corrupt.
    pub fn load(&mut self) -> Option<[u8; 6]> {
        let mut rec = [0u8; PAIRING_RECORD_LEN];
        self.flash.read(self.nvs_offset, &mut rec).ok()?;
        deserialize(&rec)
    }

    /// Persist a paired peer MAC. Returns `false` on flash error.
    pub fn save(&mut self, mac: [u8; 6]) -> bool {
        self.flash.write(self.nvs_offset, &serialize(mac)).is_ok()
    }

    /// Clear any stored pairing (forces re-pairing on next boot).
    pub fn clear(&mut self) -> bool {
        self.flash
            .write(self.nvs_offset, &[0xFF; PAIRING_RECORD_LEN])
            .is_ok()
    }
}
