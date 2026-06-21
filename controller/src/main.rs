#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    i2c::master::{
        AcknowledgeCheckFailedReason,
        Config as I2cConfig,
        Error as I2cError,
        I2c,
    },
    rmt::Rmt,
    time::Duration,
    time::Rate,
};

esp_bootloader_esp_idf::esp_app_desc!();

const I2C_SCAN_START_ADDR: u8 = 0x10;
const I2C_SCAN_END_ADDR: u8 = 0x77;
const I2C_FREQUENCY_KHZ: u32 = 10;
const RUN_I2C_SCAN: bool = false;
const RUN_STARTUP_PROBES: bool = false;
const JOYSTICK_DEFAULT_ADDRESS: u8 = 0x5A;
const JOYSTICK_ADDRESS: u8 = 0x24;
const JOYSTICK_ADDRESS_ALT_1: u8 = 0x12;
const JOYSTICK_ADDRESS_ALT_2: u8 = 0x48;
const JOYSTICK_ADDRESS_ALT_3: u8 = 0x5A;
const JOYSTICK_CANDIDATE_ADDRESSES: [u8; 4] = [
    JOYSTICK_DEFAULT_ADDRESS,
    JOYSTICK_ADDRESS,
    JOYSTICK_ADDRESS_ALT_1,
    JOYSTICK_ADDRESS_ALT_2,
];
const JOYSTICK_START_REGISTER: u8 = 0x00;
const JOYSTICK_POLL_INTERVAL_MS: u64 = 100;
const JOYSTICK_PRINT_ON_CHANGE_ONLY: bool = false;
const JOYSTICK_RUNTIME_START_REGISTER: u8 = 0x10;
const JOYSTICK_RUNTIME_FRAME_LEN: usize = 0x26; // Registers 0x10..=0x35
const JOYSTICK_PROBE_WINDOW_SIZE: usize = 8;
const JOYSTICK_PROBE_WINDOW_COUNT: usize = 8;
const JOYSTICK_PROBE_WINDOWS: [u8; JOYSTICK_PROBE_WINDOW_COUNT] = [
    0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38,
];
const JOYSTICK_PROBE_SAMPLES: u8 = 30;
const JOYSTICK_PROBE_INTERVAL_MS: u64 = 50;
const JOYSTICK_PROBE_MAX_CHANGE_PRINTS: u16 = 64;

#[derive(Copy, Clone)]
struct I2cScanSummary {
    found_count: u8,
    first_found: Option<u8>,
}

#[derive(Copy, Clone, Eq, PartialEq)]
struct JoystickButtons {
    joy: bool,
    c: bool,
    a: bool,
    b: bool,
    d: bool,
}

#[derive(Copy, Clone, Eq, PartialEq)]
struct JoystickState {
    x: u8,
    y: u8,
    buttons: JoystickButtons,
    raw_buttons: [u8; 5],
}

#[repr(C, packed)]
#[derive(Copy, Clone, Eq, PartialEq)]
struct ControllerInputPacket {
    sequence: u16,
    x: u8,
    y: u8,
    buttons: u8,
    reserved: u8,
    checksum: u8,
}

impl ControllerInputPacket {
    const LEN: usize = 7;
    const BUTTON_JOY: u8 = 1 << 0;
    const BUTTON_C: u8 = 1 << 1;
    const BUTTON_A: u8 = 1 << 2;
    const BUTTON_B: u8 = 1 << 3;
    const BUTTON_D: u8 = 1 << 4;

    fn from_state(sequence: u16, state: &JoystickState) -> Self {
        let mut buttons = 0u8;

        if state.buttons.joy {
            buttons |= Self::BUTTON_JOY;
        }
        if state.buttons.c {
            buttons |= Self::BUTTON_C;
        }
        if state.buttons.a {
            buttons |= Self::BUTTON_A;
        }
        if state.buttons.b {
            buttons |= Self::BUTTON_B;
        }
        if state.buttons.d {
            buttons |= Self::BUTTON_D;
        }

        let mut packet = Self {
            sequence,
            x: state.x,
            y: state.y,
            buttons,
            reserved: 0,
            checksum: 0,
        };

        packet.checksum = packet.compute_checksum();
        packet
    }

    fn compute_checksum(&self) -> u8 {
        let [seq_lo, seq_hi] = self.sequence.to_le_bytes();
        seq_lo ^ seq_hi ^ self.x ^ self.y ^ self.buttons ^ self.reserved
    }

    fn to_bytes(self) -> [u8; Self::LEN] {
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
}

fn device_responded(i2c: &mut I2c<'_, esp_hal::Blocking>, address: u8) -> Result<bool, I2cError> {
    // Use address-only write probe so data-phase ACK rules do not create false positives.
    match i2c.write(address, &[]) {
        Ok(()) => Ok(true),
        Err(I2cError::AcknowledgeCheckFailed(AcknowledgeCheckFailedReason::Address)) => Ok(false),
        Err(I2cError::AcknowledgeCheckFailed(AcknowledgeCheckFailedReason::Unknown)) => Ok(false),
        Err(error) => Err(error),
    }
}

fn try_read_joystick_runtime_frame_at(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    address: u8,
) -> Result<[u8; JOYSTICK_RUNTIME_FRAME_LEN], I2cError> {
    let mut data = [0u8; JOYSTICK_RUNTIME_FRAME_LEN];
    i2c.write_read(address, &[JOYSTICK_RUNTIME_START_REGISTER], &mut data)?;
    Ok(data)
}

fn read_joystick_runtime_frame(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    preferred_address: Option<u8>,
) -> Result<(u8, [u8; JOYSTICK_RUNTIME_FRAME_LEN]), I2cError> {
    let mut first_error: Option<I2cError> = None;

    if let Some(address) = preferred_address {
        if !JOYSTICK_CANDIDATE_ADDRESSES.contains(&address) {
            match try_read_joystick_runtime_frame_at(i2c, address) {
                Ok(sample) => return Ok((address, sample)),
                Err(error) => first_error = Some(error),
            }
        }
    }

    for address in JOYSTICK_CANDIDATE_ADDRESSES {
        match try_read_joystick_runtime_frame_at(i2c, address) {
            Ok(sample) => return Ok((address, sample)),
            Err(error) => {
                if first_error.is_none() {
                    first_error = Some(error);
                }
            }
        }
    }

    if let Some(address) = preferred_address {
        if JOYSTICK_CANDIDATE_ADDRESSES.contains(&address) {
            match try_read_joystick_runtime_frame_at(i2c, address) {
                Ok(sample) => return Ok((address, sample)),
                Err(error) => {
                    if first_error.is_none() {
                        first_error = Some(error);
                    }
                }
            }
        }
    }

    Err(first_error.unwrap_or(I2cError::AcknowledgeCheckFailed(
        AcknowledgeCheckFailedReason::Unknown,
    )))
}

fn button_pressed(raw: u8) -> bool {
    // User-verified behavior: released ~= 8, pressed = 0.
    raw == 0
}

fn decode_joystick_state(frame: &[u8; JOYSTICK_RUNTIME_FRAME_LEN]) -> JoystickState {
    let x = frame[(0x10 - JOYSTICK_RUNTIME_START_REGISTER) as usize];
    let y = frame[(0x11 - JOYSTICK_RUNTIME_START_REGISTER) as usize];

    let raw_buttons = [
        frame[(0x20 - JOYSTICK_RUNTIME_START_REGISTER) as usize],
        frame[(0x21 - JOYSTICK_RUNTIME_START_REGISTER) as usize],
        frame[(0x22 - JOYSTICK_RUNTIME_START_REGISTER) as usize],
        frame[(0x23 - JOYSTICK_RUNTIME_START_REGISTER) as usize],
        frame[(0x24 - JOYSTICK_RUNTIME_START_REGISTER) as usize],
    ];

    let buttons = JoystickButtons {
        joy: button_pressed(raw_buttons[0]),
        c: button_pressed(raw_buttons[1]),
        a: button_pressed(raw_buttons[2]),
        b: button_pressed(raw_buttons[3]),
        d: button_pressed(raw_buttons[4]),
    };

    JoystickState {
        x,
        y,
        buttons,
        raw_buttons,
    }
}

fn print_runtime_state(
    seq: u32,
    address: u8,
    state: &JoystickState,
    packet: &ControllerInputPacket,
) {
    esp_println::println!(
        "Joystick sample #{:08} from 0x{:02X}: x={} y={} buttons=[JOY:{} C:{} A:{} B:{} D:{}] raw_btn={:02X?} pkt={:02X?}",
        seq,
        address,
        state.x,
        state.y,
        if state.buttons.joy { "P" } else { "R" },
        if state.buttons.c { "P" } else { "R" },
        if state.buttons.a { "P" } else { "R" },
        if state.buttons.b { "P" } else { "R" },
        if state.buttons.d { "P" } else { "R" },
        state.raw_buttons,
        packet.to_bytes(),
    );
}

fn print_probe_result(op: &str, address: u8, result: Result<(), I2cError>) {
    match result {
        Ok(()) => esp_println::println!("  {} @ 0x{:02X}: OK", op, address),
        Err(error) => esp_println::println!("  {} @ 0x{:02X}: {}", op, address, error),
    }
}

fn run_i2c_joystick_diagnostics(i2c: &mut I2c<'_, esp_hal::Blocking>, discovered_address: Option<u8>) {
    esp_println::println!("I2C joystick diagnostics (candidate addresses):");

    let mut addresses = [
        JOYSTICK_ADDRESS,
        JOYSTICK_ADDRESS_ALT_1,
        JOYSTICK_ADDRESS_ALT_2,
        JOYSTICK_ADDRESS_ALT_3,
    ];

    if let Some(discovered) = discovered_address {
        addresses[0] = discovered;
    }

    for address in addresses {
        let mut read1 = [0u8; 1];
        let mut read4 = [0u8; 4];

        print_probe_result("write-empty", address, i2c.write(address, &[]));
        print_probe_result("write-reg0", address, i2c.write(address, &[JOYSTICK_START_REGISTER]));
        print_probe_result(
            "write-read1",
            address,
            i2c.write_read(address, &[JOYSTICK_START_REGISTER], &mut read1),
        );
        print_probe_result("read1", address, i2c.read(address, &mut read1));
        print_probe_result(
            "write-read4",
            address,
            i2c.write_read(address, &[JOYSTICK_START_REGISTER], &mut read4),
        );
    }
}

fn capture_probe_windows(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    address: u8,
    windows: &mut [[u8; JOYSTICK_PROBE_WINDOW_SIZE]; JOYSTICK_PROBE_WINDOW_COUNT],
    valid: &mut [bool; JOYSTICK_PROBE_WINDOW_COUNT],
) {
    for (idx, start_reg) in JOYSTICK_PROBE_WINDOWS.iter().copied().enumerate() {
        match i2c.write_read(address, &[start_reg], &mut windows[idx]) {
            Ok(()) => valid[idx] = true,
            Err(_) => valid[idx] = false,
        }
    }
}

fn run_joystick_dynamic_probe(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    delay: &Delay,
    address: u8,
) {
    esp_println::println!(
        "Dynamic joystick probe on 0x{:02X}: move stick and press buttons now...",
        address
    );

    let mut baseline = [[0u8; JOYSTICK_PROBE_WINDOW_SIZE]; JOYSTICK_PROBE_WINDOW_COUNT];
    let mut current = [[0u8; JOYSTICK_PROBE_WINDOW_SIZE]; JOYSTICK_PROBE_WINDOW_COUNT];
    let mut baseline_valid = [false; JOYSTICK_PROBE_WINDOW_COUNT];
    let mut current_valid = [false; JOYSTICK_PROBE_WINDOW_COUNT];
    let mut seen_change = [false; JOYSTICK_PROBE_WINDOW_COUNT * JOYSTICK_PROBE_WINDOW_SIZE];
    let mut printable_changes: u16 = 0;

    capture_probe_windows(i2c, address, &mut baseline, &mut baseline_valid);

    for (idx, start_reg) in JOYSTICK_PROBE_WINDOWS.iter().copied().enumerate() {
        if baseline_valid[idx] {
            esp_println::println!("  Baseline reg 0x{:02X}: {:02X?}", start_reg, baseline[idx]);
        }
    }

    for sample_idx in 1..=JOYSTICK_PROBE_SAMPLES {
        delay.delay(Duration::from_millis(JOYSTICK_PROBE_INTERVAL_MS));
        capture_probe_windows(i2c, address, &mut current, &mut current_valid);

        for window_idx in 0..JOYSTICK_PROBE_WINDOW_COUNT {
            if !baseline_valid[window_idx] || !current_valid[window_idx] {
                continue;
            }

            for byte_idx in 0..JOYSTICK_PROBE_WINDOW_SIZE {
                let old = baseline[window_idx][byte_idx];
                let new = current[window_idx][byte_idx];

                if old != new {
                    let linear_idx = window_idx * JOYSTICK_PROBE_WINDOW_SIZE + byte_idx;
                    if !seen_change[linear_idx]
                        && printable_changes < JOYSTICK_PROBE_MAX_CHANGE_PRINTS
                    {
                        seen_change[linear_idx] = true;
                        printable_changes += 1;
                        let reg = JOYSTICK_PROBE_WINDOWS[window_idx].wrapping_add(byte_idx as u8);
                        esp_println::println!(
                            "  Change at sample {:02} reg 0x{:02X}: {:02X} -> {:02X}",
                            sample_idx,
                            reg,
                            old,
                            new
                        );
                    }
                }
            }
        }
    }

    if printable_changes == 0 {
        esp_println::println!(
            "Dynamic probe saw no changes in windows 0x00..0x3F; next step is widening register range or testing command/register paging."
        );
    }
}

fn scan_i2c_bus(i2c: &mut I2c<'_, esp_hal::Blocking>) -> I2cScanSummary {
    esp_println::println!(
        "Scanning I2C bus (0x{:02X}..=0x{:02X})...",
        I2C_SCAN_START_ADDR,
        I2C_SCAN_END_ADDR
    );

    let mut found_any = false;
    let mut found_count = 0u8;
    let mut first_found = None;
    let mut data_nack_count = 0usize;

    for address in I2C_SCAN_START_ADDR..=I2C_SCAN_END_ADDR {
        match device_responded(i2c, address) {
            Ok(true) => {
                found_any = true;
                found_count = found_count.saturating_add(1);
                if first_found.is_none() {
                    first_found = Some(address);
                }
                esp_println::println!("I2C device found at 0x{:02X}", address);
            }
            Ok(false) => {}
            Err(I2cError::AcknowledgeCheckFailed(AcknowledgeCheckFailedReason::Data)) => {
                data_nack_count += 1;
            }
            Err(error) => {
                esp_println::println!(
                    "I2C probe error at 0x{:02X}: {}",
                    address,
                    error
                );
            }
        }
    }

    if !found_any {
        esp_println::println!("No I2C devices found on bus.");
    }

    if data_nack_count > 0 {
        esp_println::println!(
            "Scan observed {} data-phase NACK probes; this can indicate unsupported empty-write probing, address interpretation mismatch, or bus wiring/ground issues.",
            data_nack_count
        );
    }

    if !found_any && data_nack_count > 0 {
        esp_println::println!(
            "Hint: this pattern often means pin-label mismatch (board pin number vs GPIO number), swapped SDA/SCL wiring, or no valid target responding on this bus."
        );
    }

    I2cScanSummary {
        found_count,
        first_found,
    }
}

fn resolve_active_joystick_address(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    scan_summary: I2cScanSummary,
) -> Option<u8> {
    let mut probe_order = [0u8; 6];
    let mut count = 0usize;

    let mut push_unique = |address: u8| {
        if !probe_order[..count].contains(&address) {
            probe_order[count] = address;
            count += 1;
        }
    };

    push_unique(JOYSTICK_DEFAULT_ADDRESS);

    for address in JOYSTICK_CANDIDATE_ADDRESSES {
        push_unique(address);
    }

    if let Some(found) = scan_summary.first_found {
        push_unique(found);
    }

    for address in probe_order[..count].iter().copied() {
        match device_responded(i2c, address) {
            Ok(true) => {
                if try_read_joystick_runtime_frame_at(i2c, address).is_ok() {
                    return Some(address);
                }
            }
            Ok(false) => {}
            Err(_) => {}
        }
    }

    None
}

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();
    let mut led_toggle = false;

    esp_println::println!("Controller Board initialized!");

    esp_println::println!("Configuring I2C0 at {} kHz on GPIO6(SDA)/GPIO7(SCL)", I2C_FREQUENCY_KHZ);
    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(I2C_FREQUENCY_KHZ));
    let mut i2c = I2c::new(peripherals.I2C0, i2c_config)
        .expect("Failed to initialize I2C0")
        .with_sda(peripherals.GPIO6)
        .with_scl(peripherals.GPIO7);

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("Failed to initialize RMT");
    let mut led = common_led::new_ws2812::<_, _, { common_led::LED_BUFFER_SIZE }>(
        rmt.channel0,
        peripherals.GPIO8,
    )
    .expect("Failed to initialize WS2812B LED");

    if let Err(error) = common_led::set_rgb(&mut led, 0, 16, 0) {
        esp_println::println!("Failed to set controller boot LED color: {:?}", error);
    }

    let scan_summary = if RUN_I2C_SCAN {
        let summary = scan_i2c_bus(&mut i2c);
        if let Some(found) = summary.first_found {
            esp_println::println!(
                "I2C scan summary: {} device(s), first at 0x{:02X}.",
                summary.found_count,
                found
            );
        }
        summary
    } else {
        esp_println::println!(
            "Skipping full I2C scan; using validated joystick candidate probing."
        );
        I2cScanSummary {
            found_count: 0,
            first_found: None,
        }
    };

    if RUN_STARTUP_PROBES {
        run_i2c_joystick_diagnostics(&mut i2c, scan_summary.first_found);
    }

    let active_joystick_address = resolve_active_joystick_address(&mut i2c, scan_summary);

    if RUN_STARTUP_PROBES {
        if let Some(address) = active_joystick_address {
            run_joystick_dynamic_probe(&mut i2c, &delay, address);
        }
    }

    if let Some(address) = active_joystick_address {
        esp_println::println!("Joystick active address resolved to 0x{:02X}.", address);
    } else {
        esp_println::println!(
            "Warning: could not resolve a working joystick address; last known default is 0x{:02X}.",
            JOYSTICK_DEFAULT_ADDRESS
        );
    }

    // TODO: Initialize communication (e.g. ESP-NOW) to connect with the vehicle

    let mut sample_seq: u32 = 0;
    let mut last_state = JoystickState {
        x: 0,
        y: 0,
        buttons: JoystickButtons {
            joy: false,
            c: false,
            a: false,
            b: false,
            d: false,
        },
        raw_buttons: [0; 5],
    };
    let mut has_last_state = false;

    loop {
        // Main control loop processing input
        sample_seq = sample_seq.wrapping_add(1);

        match read_joystick_runtime_frame(&mut i2c, active_joystick_address) {
            Ok((address, frame)) => {
                let state = decode_joystick_state(&frame);
                let packet = ControllerInputPacket::from_state(sample_seq as u16, &state);
                let should_print = !JOYSTICK_PRINT_ON_CHANGE_ONLY || !has_last_state || state != last_state;

                if should_print {
                    print_runtime_state(sample_seq, address, &state, &packet);
                }

                last_state = state;
                has_last_state = true;
            }
            Err(error) => {
                esp_println::println!(
                    "Joystick read failed at 0x{:02X}: {}",
                    active_joystick_address.unwrap_or(JOYSTICK_DEFAULT_ADDRESS),
                    error
                );
            }
        }

        let color = if led_toggle { (0, 0, 16) } else { (16, 0, 0) };
        led_toggle = !led_toggle;

        if let Err(error) = common_led::set_rgb(&mut led, color.0, color.1, color.2) {
            esp_println::println!("Failed to update controller LED color: {:?}", error);
        }

        delay.delay(Duration::from_millis(JOYSTICK_POLL_INTERVAL_MS));
    }
}
