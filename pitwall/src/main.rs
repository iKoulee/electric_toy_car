//! pitwall — terminal telemetry dashboard for the controller/vehicle boards.
//!
//! Connects to a board over USB serial, renders both boards' telemetry as a
//! live TUI (link state, joystick, motor duty), and sends interactive commands.

mod app;
mod command;
mod serial;
mod ui;

use std::error::Error;
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use ratatui::style::Color;
use ratatui::DefaultTerminal;
use serialport::SerialPort;

use common_host_proto::{
    encode_host, BoardKind, BoardToHost, HostToBoard, MAX_FRAME_BYTES, PROTOCOL_VERSION,
};

use app::AppState;

type Port = Arc<Mutex<Box<dyn SerialPort>>>;

#[derive(Parser)]
#[command(about = "Terminal telemetry dashboard for the controller/vehicle boards over USB")]
struct Args {
    /// Serial port (e.g. /dev/ttyACM0 or COM3). Auto-detected if omitted.
    #[arg(long)]
    port: Option<String>,

    /// Baud rate (USB Serial JTAG ignores this, but serialport requires a value).
    #[arg(long, default_value_t = 115200)]
    baud: u32,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    let port_name = match args.port {
        Some(p) => p,
        None => serial::auto_select_port()?,
    };

    let port = serial::open_port(&port_name, args.baud)?;
    let read_port = port.try_clone()?;
    let write_port: Port = Arc::new(Mutex::new(port));

    let (tx_msg, rx_msg) = mpsc::channel::<BoardToHost>();
    let (tx_exit, rx_exit) = mpsc::channel::<String>();
    thread::spawn(move || serial::reader_thread(read_port, tx_msg, tx_exit));

    // Handshake before entering the TUI so failures print plainly.
    println!("Opened {port_name}. Connecting...");
    let gateway = match handshake(&write_port, &rx_msg, &rx_exit)? {
        Some(board) => board,
        None => {
            eprintln!("No response from board after 10 s.");
            eprintln!("Check: firmware flashed, USB cable, and the selected port.");
            return Ok(());
        }
    };

    let mut app = AppState::new(port_name);
    app.gateway = Some(gateway);
    app.connected = true;
    app.push_log(
        format!("Connected — gateway board: {}", app::board_kind_str(&gateway)),
        Color::Green,
    );

    let mut terminal = ratatui::init();
    let res = run(&mut terminal, &mut app, &write_port, &rx_msg, &rx_exit);
    ratatui::restore();
    res
}

/// Ping the board until it replies with `Pong` (returning the board kind), or
/// give up after 10 s. Mirrors the original devkit-reset-tolerant retry loop.
fn handshake(
    port: &Port,
    rx_msg: &Receiver<BoardToHost>,
    rx_exit: &Receiver<String>,
) -> Result<Option<BoardKind>, Box<dyn Error>> {
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut next_ping = Instant::now();
    while Instant::now() < deadline {
        if let Ok(reason) = rx_exit.try_recv() {
            return Err(format!("USB lost during connect: {reason}").into());
        }
        while let Ok(msg) = rx_msg.try_recv() {
            if let BoardToHost::Pong { board, .. } = msg {
                return Ok(Some(board));
            }
        }
        if Instant::now() >= next_ping {
            send_host_msg(port, &HostToBoard::Ping { version: PROTOCOL_VERSION })?;
            next_ping = Instant::now() + Duration::from_millis(500);
        }
        thread::sleep(Duration::from_millis(20));
    }
    Ok(None)
}

fn run(
    terminal: &mut DefaultTerminal,
    app: &mut AppState,
    port: &Port,
    rx_msg: &Receiver<BoardToHost>,
    rx_exit: &Receiver<String>,
) -> Result<(), Box<dyn Error>> {
    loop {
        while let Ok(msg) = rx_msg.try_recv() {
            app.ingest(&msg);
        }
        if let Ok(reason) = rx_exit.try_recv() {
            app.connected = false;
            app.push_log(format!("USB lost: {reason}"), Color::Red);
        }

        terminal.draw(|frame| ui::draw(frame, app))?;

        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                if key.kind == KeyEventKind::Press {
                    handle_key(app, port, key.code, key.modifiers)?;
                }
            }
        }

        if app.should_quit {
            break;
        }
    }
    Ok(())
}

fn handle_key(
    app: &mut AppState,
    port: &Port,
    code: KeyCode,
    mods: KeyModifiers,
) -> Result<(), Box<dyn Error>> {
    // While the help popup is open, any key (except Ctrl+C) just closes it.
    if app.show_help {
        if code == KeyCode::Char('c') && mods.contains(KeyModifiers::CONTROL) {
            app.should_quit = true;
        } else {
            app.show_help = false;
        }
        return Ok(());
    }

    match code {
        KeyCode::Esc => app.should_quit = true,
        KeyCode::F(1) => app.show_help = true,
        KeyCode::Char('c') if mods.contains(KeyModifiers::CONTROL) => app.should_quit = true,
        KeyCode::Enter => {
            let line = app.input.trim().to_string();
            app.input.clear();
            match line.as_str() {
                "" => {}
                "quit" | "exit" | "q" => app.should_quit = true,
                "help" | "h" | "?" => app.show_help = true,
                input => match command::parse_command(input) {
                    Some(cmd) => {
                        send_host_msg(port, &cmd)?;
                        app.push_log(format!("> {input}"), Color::Gray);
                    }
                    None => app.push_log(format!("Unknown command: {input}"), Color::Red),
                },
            }
        }
        KeyCode::Backspace => {
            app.input.pop();
        }
        KeyCode::Char(c) => app.input.push(c),
        _ => {}
    }
    Ok(())
}

fn send_host_msg(port: &Port, msg: &HostToBoard) -> Result<(), Box<dyn Error>> {
    let mut buf = [0u8; MAX_FRAME_BYTES];
    let n = encode_host(msg, &mut buf)?;
    let mut port = port.lock().unwrap();
    port.write_all(&buf[..n])?;
    port.flush()?;
    Ok(())
}
