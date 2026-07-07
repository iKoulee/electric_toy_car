//! Serial-port discovery, opening, and the background reader thread.

use std::error::Error;
use std::io::{self, Write};
use std::sync::mpsc::Sender;
use std::time::Duration;

use postcard::accumulator::{CobsAccumulator, FeedResult};
use serialport::{available_ports, SerialPort, SerialPortInfo, SerialPortType};

use common_host_proto::BoardToHost;

/// Open a serial port by name at the given baud rate.
/// (USB Serial JTAG ignores baud, but the `serialport` API requires a value.)
pub fn open_port(name: &str, baud: u32) -> Result<Box<dyn SerialPort>, Box<dyn Error>> {
    serialport::new(name, baud)
        .timeout(Duration::from_millis(100))
        .open()
        .map_err(|e| format!("Failed to open {name}: {e}").into())
}

/// Resolve which serial port to use when `--port` was not supplied.
///
/// - no ports  → error explaining nothing is connected
/// - one port  → auto-select it
/// - many      → print a numbered list and prompt for a choice on stdin
///
/// Runs before the TUI is entered, so plain stdout/stdin is fine here.
pub fn auto_select_port() -> Result<String, Box<dyn Error>> {
    let ports = available_ports()?;
    match ports.len() {
        0 => Err("No serial ports found. Connect a board via USB, or pass --port.".into()),
        1 => {
            let name = ports[0].port_name.clone();
            println!("Auto-selected the only serial port: {name} ({})", describe(&ports[0]));
            Ok(name)
        }
        _ => {
            println!("Multiple serial ports found:");
            for (i, p) in ports.iter().enumerate() {
                println!("  [{i}] {}  —  {}", p.port_name, describe(p));
            }
            print!("Select port number: ");
            io::stdout().flush()?;
            let mut line = String::new();
            io::stdin().read_line(&mut line)?;
            let idx: usize = line
                .trim()
                .parse()
                .map_err(|_| "Invalid selection (expected a number).")?;
            ports
                .get(idx)
                .map(|p| p.port_name.clone())
                .ok_or_else(|| "Selection out of range.".into())
        }
    }
}

/// Human-readable description of a discovered port (USB product/VID:PID when known).
fn describe(info: &SerialPortInfo) -> String {
    match &info.port_type {
        SerialPortType::UsbPort(usb) => {
            let product = usb.product.clone().unwrap_or_else(|| "USB serial".into());
            format!("{product} [{:04x}:{:04x}]", usb.vid, usb.pid)
        }
        SerialPortType::PciPort => "PCI serial".into(),
        SerialPortType::BluetoothPort => "Bluetooth serial".into(),
        SerialPortType::Unknown => "unknown".into(),
    }
}

/// Blocking serial reader: reassembles COBS frames and forwards decoded
/// `BoardToHost` messages over `tx`. On fatal I/O error it reports the reason
/// via `tx_exit` and returns. Malformed/oversized frames are silently dropped
/// (the TUI owns the screen, so we can't print warnings here).
pub fn reader_thread(
    mut port: Box<dyn SerialPort>,
    tx: Sender<BoardToHost>,
    tx_exit: Sender<String>,
) {
    let mut cobs_buf = CobsAccumulator::<64>::new();
    let mut raw = [0u8; 32];

    loop {
        match port.read(&mut raw) {
            Ok(0) => continue,
            Ok(n) => {
                let mut window = &raw[..n];
                'feed: while !window.is_empty() {
                    window = match cobs_buf.feed::<BoardToHost>(window) {
                        FeedResult::Consumed => break 'feed,
                        FeedResult::OverFull(w) => w,
                        FeedResult::DeserError(w) => w,
                        FeedResult::Success { data, remaining } => {
                            if tx.send(data).is_err() {
                                let _ = tx_exit.send("main loop exited".into());
                                return;
                            }
                            remaining
                        }
                    };
                }
            }
            Err(e) if e.kind() == io::ErrorKind::TimedOut => continue,
            Err(e) => {
                let _ = tx_exit.send(format!("read error: {e}"));
                return;
            }
        }
    }
}
