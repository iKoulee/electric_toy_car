use crate::protocol::{
    CONTROL_PACKET_LEN,
    ControlPacket,
    PacketError,
    is_newer_sequence,
};

// TODO(next): Implement a board-specific EspNowTransport adapter in controller/vehicle crates
//             and pass it into ControllerLink/VehicleLink.

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct ReceivedMeta {
    pub peer_mac: [u8; 6],
    pub len: usize,
    pub rssi_dbm: Option<i8>,
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct ReceivedControl {
    pub packet: ControlPacket,
    pub meta: ReceivedMeta,
}

pub trait EspNowTransport {
    type Error;

    fn send(&mut self, peer_mac: [u8; 6], payload: &[u8]) -> Result<(), Self::Error>;

    fn receive(&mut self, rx_buffer: &mut [u8]) -> Result<Option<ReceivedMeta>, Self::Error>;
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum LinkError<E> {
    Transport(E),
    Packet(PacketError),
    StaleSequence,
}

pub struct ControllerLink<T: EspNowTransport> {
    transport: T,
    peer_mac: [u8; 6],
}

impl<T: EspNowTransport> ControllerLink<T> {
    pub const fn new(transport: T, peer_mac: [u8; 6]) -> Self {
        Self {
            transport,
            peer_mac,
        }
    }

    pub fn send_control(&mut self, packet: ControlPacket) -> Result<(), LinkError<T::Error>> {
        let bytes = packet.to_bytes();
        self.transport
            .send(self.peer_mac, &bytes)
            .map_err(LinkError::Transport)
    }
}

pub struct VehicleLink<T: EspNowTransport> {
    transport: T,
    last_sequence: Option<u16>,
}

impl<T: EspNowTransport> VehicleLink<T> {
    pub const fn new(transport: T) -> Self {
        Self {
            transport,
            last_sequence: None,
        }
    }

    pub fn try_receive_control(
        &mut self,
        rx_buffer: &mut [u8; CONTROL_PACKET_LEN],
    ) -> Result<Option<ReceivedControl>, LinkError<T::Error>> {
        let meta = match self
            .transport
            .receive(rx_buffer)
            .map_err(LinkError::Transport)?
        {
            Some(meta) => meta,
            None => return Ok(None),
        };

        let packet = ControlPacket::from_bytes(rx_buffer).map_err(LinkError::Packet)?;

        if let Some(last) = self.last_sequence {
            if !is_newer_sequence(last, packet.sequence) {
                return Err(LinkError::StaleSequence);
            }
        }

        self.last_sequence = Some(packet.sequence);

        Ok(Some(ReceivedControl { packet, meta }))
    }
}
