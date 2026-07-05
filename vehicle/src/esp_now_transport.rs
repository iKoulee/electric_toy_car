use common_comms::espnow::{EspNowTransport, ReceivedMeta};
use esp_radio::esp_now::{EspNow, EspNowError, EspNowWifiInterface, PeerInfo};

pub struct Esp32C6EspNow<'d> {
    inner: EspNow<'d>,
}

impl<'d> Esp32C6EspNow<'d> {
    pub fn new(esp_now: EspNow<'d>) -> Self {
        Self { inner: esp_now }
    }
}

impl EspNowTransport for Esp32C6EspNow<'_> {
    type Error = EspNowError;

    fn send(&mut self, peer_mac: [u8; 6], payload: &[u8]) -> Result<(), Self::Error> {
        self.inner.send(&peer_mac, payload)?.wait()
    }

    fn receive(&mut self, rx_buffer: &mut [u8]) -> Result<Option<ReceivedMeta>, Self::Error> {
        match self.inner.receive() {
            Some(data) => {
                let src = data.data();
                let copy_len = src.len().min(rx_buffer.len());
                rx_buffer[..copy_len].copy_from_slice(&src[..copy_len]);
                Ok(Some(ReceivedMeta {
                    peer_mac: data.info.src_address,
                    dst_mac: data.info.dst_address,
                    len: copy_len,
                    rssi_dbm: Some(data.info.rx_control.rssi as i8),
                }))
            }
            None => Ok(None),
        }
    }

    fn add_peer(&mut self, mac: [u8; 6]) -> Result<(), Self::Error> {
        // Idempotent: the ESP-NOW peer list rejects duplicates with PeerExists,
        // so skip the add when the peer is already registered.
        if self.inner.peer_exists(&mac) {
            return Ok(());
        }
        self.inner.add_peer(PeerInfo {
            interface: EspNowWifiInterface::Sta,
            peer_address: mac,
            lmk: None,
            channel: None,
            encrypt: false,
        })
    }

    fn peer_exists(&self, mac: [u8; 6]) -> bool {
        self.inner.peer_exists(&mac)
    }
}
