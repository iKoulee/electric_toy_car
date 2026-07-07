use common_host_proto::{
    BoardKind, BoardToHost, HostError, HostToBoard, MAX_FRAME_BYTES, PROTOCOL_VERSION,
    decode_host, encode_board,
};
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embedded_io_async::{Read, Write};
use esp_hal::{Async, usb_serial_jtag::UsbSerialJtag};

/// Events from the main loop to the USB host (joystick readings, link state).
pub static EVENTS: Channel<CriticalSectionRawMutex, BoardToHost, 8> = Channel::new();

/// Commands from the USB host to the main loop (LED overrides, etc.).
pub static CMDS: Channel<CriticalSectionRawMutex, HostToBoard, 4> = Channel::new();

#[embassy_executor::task]
pub async fn task(usb: UsbSerialJtag<'static, Async>) {
    let (mut rx, mut tx) = usb.split();
    let mut rx_frame = [0u8; MAX_FRAME_BYTES];
    let mut rx_pos: usize = 0;

    loop {
        let mut chunk = [0u8; 16];
        match select(EVENTS.receive(), rx.read(&mut chunk)).await {
            Either::First(msg) => {
                let mut buf = [0u8; MAX_FRAME_BYTES];
                if let Ok(n) = encode_board(&msg, &mut buf) {
                    // Async write: when no USB host is draining the FIFO this
                    // yields (pends) instead of spin-waiting, so the Embassy
                    // executor keeps running the main control loop.
                    let _ = tx.write_all(&buf[..n]).await;
                    let _ = tx.flush().await;
                }
            }
            Either::Second(Ok(n)) => {
                for i in 0..n {
                    let b = chunk[i];
                    if rx_pos < rx_frame.len() {
                        rx_frame[rx_pos] = b;
                        rx_pos += 1;
                    } else {
                        rx_pos = 0;
                    }
                    if b == 0x00 && rx_pos > 0 {
                        let reply = handle_cmd(&mut rx_frame[..rx_pos]);
                        rx_pos = 0;
                        if let Some(reply) = reply {
                            let mut buf = [0u8; MAX_FRAME_BYTES];
                            if let Ok(n) = encode_board(&reply, &mut buf) {
                                let _ = tx.write_all(&buf[..n]).await;
                                let _ = tx.flush().await;
                            }
                        }
                    }
                }
            }
            Either::Second(Err(_)) => rx_pos = 0,
        }
    }
}

fn handle_cmd(frame: &mut [u8]) -> Option<BoardToHost> {
    match decode_host(frame) {
        Ok(HostToBoard::Ping { version: _ }) => {
            Some(BoardToHost::Pong { version: PROTOCOL_VERSION, board: BoardKind::Controller })
        }
        Ok(cmd @ HostToBoard::SetLed(_))
        | Ok(cmd @ HostToBoard::ForPeer(_))
        | Ok(cmd @ HostToBoard::EnableRemoteTelemetry { .. })
        | Ok(cmd @ HostToBoard::Repair) => {
            // Forwarded to the main loop, which owns the radio and pairing store.
            if CMDS.try_send(cmd).is_err() {
                Some(BoardToHost::Error(HostError::Busy))
            } else {
                None
            }
        }
        Ok(_) => Some(BoardToHost::Error(HostError::NotApplicable)),
        Err(_) => Some(BoardToHost::Error(HostError::ParseError)),
    }
}
