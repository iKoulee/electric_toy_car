use std::io::{self, BufRead, Write as IoWrite};
use std::sync::{mpsc, Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use crossterm::style::{Color, Print, ResetColor, SetForegroundColor};
use crossterm::ExecutableCommand;
use postcard::accumulator::{CobsAccumulator, FeedResult};
use serialport::SerialPort;

use common_host_proto::{
    encode_host, BoardToHost, HostToBoard, LinkStateKind, BoardKind,
    MAX_FRAME_BYTES, PROTOCOL_VERSION,
};

#[derive(Parser)]
#[command(about = "Terminal tool for communicating with controller/vehicle boards over USB")]
struct Args {
    /// Serial port path (e.g. /dev/ttyACM0)
    #[arg(long, default_value = "/dev/ttyACM0")]
    port: String,

    /// Baud rate (USB Serial JTAG ignores this, but serialport requires it)
    #[arg(long, default_value_t = 115200)]
    baud: u32,

    /// Print raw bytes sent and received for debugging
    #[arg(long)]
    debug: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let debug = Arc::new(AtomicBool::new(args.debug));

    let port = serialport::new(&args.port, args.baud)
        .timeout(Duration::from_millis(100))
        .open()
        .map_err(|e| format!("Failed to open {}: {}", args.port, e))?;

    let read_port = port.try_clone()?;
    let write_port: Arc<Mutex<Box<dyn SerialPort>>> = Arc::new(Mutex::new(port));

    let (tx_msg, rx_msg) = mpsc::channel::<BoardToHost>();
    let (tx_cmd, rx_cmd) = mpsc::channel::<String>();
    // Signals the main loop that the reader thread has exited.
    let (tx_reader_exit, rx_reader_exit) = mpsc::channel::<String>();

    let debug_reader = Arc::clone(&debug);
    thread::spawn(move || reader_thread(read_port, tx_msg, tx_reader_exit, debug_reader));
    thread::spawn(move || stdin_thread(tx_cmd));

    print_info(&format!("Opened {}.", args.port));
    eprint!("Connecting");
    let _ = io::stderr().flush();

    // Retry loop: the board may reset when the port is opened (DTR glitch on devkits wires
    // DTR → EN).  Keep pinging every 500 ms until a Pong arrives or the timeout expires.
    let connected = 'connect: {
        let deadline = Instant::now() + Duration::from_secs(10);
        let mut next_ping = Instant::now(); // ping immediately on first iteration
        while Instant::now() < deadline {
            if let Ok(reason) = rx_reader_exit.try_recv() {
                eprintln!();
                print_error(&format!("USB lost during connect: {reason}"));
                return Err("USB connection lost".into());
            }
            while let Ok(msg) = rx_msg.try_recv() {
                if matches!(msg, BoardToHost::Pong { .. }) {
                    eprintln!();
                    print_board_msg(&msg);
                    break 'connect true;
                }
                // Discard any other unsolicited messages during the connect phase.
            }
            if Instant::now() >= next_ping {
                send_host_msg(
                    &write_port,
                    &HostToBoard::Ping { version: PROTOCOL_VERSION },
                    &debug,
                )?;
                next_ping = Instant::now() + Duration::from_millis(500);
                eprint!(".");
                let _ = io::stderr().flush();
            }
            thread::sleep(Duration::from_millis(20));
        }
        eprintln!();
        false
    };

    if !connected {
        print_error("No response from board after 10 s.");
        print_error("Check: firmware flashed (cargo run -p controller --release), USB cable, port.");
        return Ok(());
    }

    print_info("Commands: ping | led R G B | led off | motor_en R_EN L_EN | motor_pwm DUTY (-100..100) | quit");

    loop {
        // Drain all pending board messages.
        while let Ok(msg) = rx_msg.try_recv() {
            print_board_msg(&msg);
        }

        // Check if the reader thread has died.
        if let Ok(reason) = rx_reader_exit.try_recv() {
            print_error(&format!("Reader thread exited: {reason}"));
            print_error("No further messages will be received. Check USB connection.");
            break;
        }

        if let Ok(line) = rx_cmd.try_recv() {
            match line.as_str() {
                "quit" | "exit" | "q" => break,
                "" => {}
                input => match parse_command(input) {
                    Some(cmd) => send_host_msg(&write_port, &cmd, &debug)?,
                    None => {
                        print_error(&format!("Unknown command: {input}"));
                        print_info("Commands: ping | led R G B | led off | motor_en R_EN L_EN | motor_pwm DUTY (-100..100) | quit");
                    }
                },
            }
        }

        thread::sleep(Duration::from_millis(10));
    }

    print_info("Disconnected.");
    Ok(())
}

fn reader_thread(
    mut port: Box<dyn SerialPort>,
    tx: mpsc::Sender<BoardToHost>,
    tx_exit: mpsc::Sender<String>,
    debug: Arc<AtomicBool>,
) {
    let mut cobs_buf = CobsAccumulator::<64>::new();
    let mut raw = [0u8; 32];

    loop {
        match port.read(&mut raw) {
            Ok(0) => continue,
            Ok(n) => {
                if debug.load(Ordering::Relaxed) {
                    eprint!("[rx {} bytes]", n);
                    for b in &raw[..n] { eprint!(" {:02x}", b); }
                    eprintln!();
                }
                let mut window = &raw[..n];
                'feed: while !window.is_empty() {
                    window = match cobs_buf.feed::<BoardToHost>(window) {
                        FeedResult::Consumed => break 'feed,
                        FeedResult::OverFull(w) => {
                            eprintln!("[warn] COBS frame too large, discarding");
                            w
                        }
                        FeedResult::DeserError(w) => {
                            eprintln!("[warn] COBS deserialize error, discarding frame");
                            w
                        }
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

fn stdin_thread(tx: mpsc::Sender<String>) {
    let stdin = io::stdin();
    for line in stdin.lock().lines() {
        match line {
            Ok(l) => {
                if tx.send(l).is_err() {
                    break;
                }
            }
            Err(_) => break,
        }
    }
}

fn parse_bool(s: &str) -> Option<bool> {
    match s {
        "1" | "true" | "on" => Some(true),
        "0" | "false" | "off" => Some(false),
        _ => None,
    }
}

fn parse_command(input: &str) -> Option<HostToBoard> {
    let parts: Vec<&str> = input.split_whitespace().collect();
    match parts.as_slice() {
        ["ping"] => Some(HostToBoard::Ping { version: PROTOCOL_VERSION }),
        ["led", "off"] => Some(HostToBoard::SetLed(None)),
        ["led", r, g, b] => {
            let r = r.parse::<u8>().ok()?;
            let g = g.parse::<u8>().ok()?;
            let b = b.parse::<u8>().ok()?;
            Some(HostToBoard::SetLed(Some([r, g, b])))
        }
        ["motor_en", r, l] => {
            let r_en = parse_bool(r)?;
            let l_en = parse_bool(l)?;
            Some(HostToBoard::SetMotorEnable { r_en, l_en })
        }
        ["motor_pwm", d] => {
            let duty = d.parse::<i8>().ok()?.clamp(-100, 100);
            Some(HostToBoard::SetMotorPwm { duty })
        }
        _ => None,
    }
}

fn send_host_msg(
    port: &Arc<Mutex<Box<dyn SerialPort>>>,
    msg: &HostToBoard,
    debug: &Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut buf = [0u8; MAX_FRAME_BYTES];
    let n = encode_host(msg, &mut buf)?;
    if debug.load(Ordering::Relaxed) {
        eprint!("[tx {} bytes]", n);
        for b in &buf[..n] { eprint!(" {:02x}", b); }
        eprintln!();
    }
    let mut port = port.lock().unwrap();
    port.write_all(&buf[..n])?;
    port.flush()?;
    Ok(())
}

fn print_board_msg(msg: &BoardToHost) {
    match msg {
        BoardToHost::Pong { version, board } => {
            let board_str = match board {
                BoardKind::Controller => "Controller",
                BoardKind::Vehicle => "Vehicle",
            };
            print_colored(
                &format!("[PONG] version={version} board={board_str}"),
                Color::Green,
            );
        }
        BoardToHost::JoystickState { x, y, buttons } => {
            print_colored(
                &format!("[JOYSTICK] x={x} y={y} buttons={buttons:#04x}"),
                Color::Cyan,
            );
        }
        BoardToHost::EspNowLinkState(state) => {
            let state_str = match state {
                LinkStateKind::AwaitingFirstPacket => "AwaitingFirstPacket",
                LinkStateKind::Alive => "Alive",
                LinkStateKind::TimedOut => "TimedOut",
            };
            print_colored(&format!("[LINK] {state_str}"), Color::Yellow);
        }
        BoardToHost::ReceivedPacket { x, y, buttons } => {
            print_colored(
                &format!("[PACKET] x={x} y={y} buttons={buttons:#04x}"),
                Color::Cyan,
            );
        }
        BoardToHost::MotorState { duty } => {
            print_colored(&format!("[MOTOR] duty={duty}"), Color::Blue);
        }
        BoardToHost::LedAck => {
            print_colored("[LED_ACK]", Color::Green);
        }
        BoardToHost::Error(e) => {
            print_colored(&format!("[ERROR] {e:?}"), Color::Red);
        }
        _ => {
            print_colored(&format!("[UNKNOWN] {msg:?}"), Color::DarkGrey);
        }
    }
}

fn print_colored(text: &str, color: Color) {
    let mut stdout = io::stdout();
    let _ = stdout
        .execute(SetForegroundColor(color))
        .and_then(|s| s.execute(Print(text)))
        .and_then(|s| s.execute(ResetColor))
        .and_then(|s| s.execute(Print("\n")));
    let _ = stdout.flush();
}

fn print_info(text: &str) {
    println!("{text}");
}

fn print_error(text: &str) {
    print_colored(text, Color::Red);
}
