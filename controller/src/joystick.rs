use crate::config::i2c::{SCAN_END_ADDR, SCAN_START_ADDR};
use crate::config::joystick::{
    CANDIDATE_ADDRESSES, DEFAULT_ADDRESS, PROBE_MAX_CHANGE_PRINTS, PROBE_SAMPLES, PROBE_WINDOWS,
    PROBE_WINDOW_COUNT, PROBE_WINDOW_SIZE, RUNTIME_FRAME_LEN, RUNTIME_START_REGISTER,
    START_REGISTER,
};
use crate::config::timing::PROBE_INTERVAL_MS;
use common_comms::protocol::ControlPacket;
use esp_hal::{
    delay::Delay,
    i2c::master::{AcknowledgeCheckFailedReason, Error as I2cError, I2c},
    time::Duration,
};

#[derive(Copy, Clone)]
pub struct I2cScanSummary {
    pub found_count: u8,
    pub first_found: Option<u8>,
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub struct JoystickButtons {
    pub joy: bool,
    pub c: bool,
    pub a: bool,
    pub b: bool,
    pub d: bool,
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub struct JoystickState {
    pub x: u8,
    pub y: u8,
    pub buttons: JoystickButtons,
    pub raw_buttons: [u8; 5],
}

pub fn encode_buttons(buttons: &JoystickButtons) -> u8 {
    let mut packed = 0u8;
    if buttons.joy {
        packed |= ControlPacket::BUTTON_JOY;
    }
    if buttons.c {
        packed |= ControlPacket::BUTTON_C;
    }
    if buttons.a {
        packed |= ControlPacket::BUTTON_A;
    }
    if buttons.b {
        packed |= ControlPacket::BUTTON_B;
    }
    if buttons.d {
        packed |= ControlPacket::BUTTON_D;
    }
    packed
}

pub const fn neutral_joystick_state() -> JoystickState {
    JoystickState {
        x: 128,
        y: 128,
        buttons: JoystickButtons {
            joy: false,
            c: false,
            a: false,
            b: false,
            d: false,
        },
        // Released state reads ~8; using 8 so raw_buttons reflects actual released values.
        raw_buttons: [8; 5],
    }
}

fn button_pressed(raw: u8) -> bool {
    // User-verified behavior: released ~= 8, pressed = 0.
    raw == 0
}

pub fn decode_joystick_state(frame: &[u8; RUNTIME_FRAME_LEN]) -> JoystickState {
    let offset = RUNTIME_START_REGISTER as usize;
    let x = frame[0x10 - offset];
    let y = frame[0x11 - offset];

    let raw_buttons = [
        frame[0x20 - offset],
        frame[0x21 - offset],
        frame[0x22 - offset],
        frame[0x23 - offset],
        frame[0x24 - offset],
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

pub fn print_runtime_state(
    seq: u32,
    address: u8,
    state: &JoystickState,
    packet: &ControlPacket,
    reason: &str,
) {
    esp_println::println!(
        "Joystick tx #{:08} ({}) from 0x{:02X}: x={} y={} buttons=[JOY:{} C:{} A:{} B:{} D:{}] raw_btn={:02X?} pkt={:02X?}",
        seq, reason, address,
        state.x, state.y,
        if state.buttons.joy { "P" } else { "R" },
        if state.buttons.c   { "P" } else { "R" },
        if state.buttons.a   { "P" } else { "R" },
        if state.buttons.b   { "P" } else { "R" },
        if state.buttons.d   { "P" } else { "R" },
        state.raw_buttons,
        packet.to_bytes(),
    );
}

pub fn print_joystick_status(state: &JoystickState, consecutive_read_failures: u8) {
    esp_println::println!(
        "Joystick status: x={} y={} buttons=[JOY:{} C:{} A:{} B:{} D:{}] failures={}",
        state.x,
        state.y,
        if state.buttons.joy { "P" } else { "R" },
        if state.buttons.c { "P" } else { "R" },
        if state.buttons.a { "P" } else { "R" },
        if state.buttons.b { "P" } else { "R" },
        if state.buttons.d { "P" } else { "R" },
        consecutive_read_failures,
    );
}

fn try_read_joystick_runtime_frame_at(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    address: u8,
) -> Result<[u8; RUNTIME_FRAME_LEN], I2cError> {
    let mut data = [0u8; RUNTIME_FRAME_LEN];
    i2c.write_read(address, &[RUNTIME_START_REGISTER], &mut data)?;
    Ok(data)
}

async fn try_read_joystick_runtime_frame_at_async(
    i2c: &mut I2c<'_, esp_hal::Async>,
    address: u8,
) -> Result<[u8; RUNTIME_FRAME_LEN], I2cError> {
    let mut data = [0u8; RUNTIME_FRAME_LEN];
    i2c.write_read_async(address, &[RUNTIME_START_REGISTER], &mut data)
        .await?;
    Ok(data)
}

/// Try `preferred_address` first (if not already in the candidate list), then try all candidates.
pub async fn read_joystick_runtime_frame_async(
    i2c: &mut I2c<'_, esp_hal::Async>,
    preferred_address: Option<u8>,
) -> Result<(u8, [u8; RUNTIME_FRAME_LEN]), I2cError> {
    let mut first_error: Option<I2cError> = None;

    if let Some(addr) = preferred_address {
        if !CANDIDATE_ADDRESSES.contains(&addr) {
            match try_read_joystick_runtime_frame_at_async(i2c, addr).await {
                Ok(frame) => return Ok((addr, frame)),
                Err(e) => first_error = Some(e),
            }
        }
    }

    for addr in CANDIDATE_ADDRESSES {
        match try_read_joystick_runtime_frame_at_async(i2c, addr).await {
            Ok(frame) => return Ok((addr, frame)),
            Err(e) => first_error = first_error.or(Some(e)),
        }
    }

    Err(first_error.unwrap_or(I2cError::AcknowledgeCheckFailed(
        AcknowledgeCheckFailedReason::Unknown,
    )))
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

pub fn scan_i2c_bus(i2c: &mut I2c<'_, esp_hal::Blocking>) -> I2cScanSummary {
    esp_println::println!(
        "Scanning I2C bus (0x{:02X}..=0x{:02X})...",
        SCAN_START_ADDR,
        SCAN_END_ADDR
    );

    let mut found_any = false;
    let mut found_count = 0u8;
    let mut first_found = None;
    let mut data_nack_count = 0usize;

    for address in SCAN_START_ADDR..=SCAN_END_ADDR {
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
                esp_println::println!("I2C probe error at 0x{:02X}: {}", address, error);
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

pub fn resolve_active_joystick_address(
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

    push_unique(DEFAULT_ADDRESS);
    for address in CANDIDATE_ADDRESSES {
        push_unique(address);
    }
    if let Some(found) = scan_summary.first_found {
        push_unique(found);
    }

    for address in probe_order[..count].iter().copied() {
        match device_responded(i2c, address) {
            Ok(true) if try_read_joystick_runtime_frame_at(i2c, address).is_ok() => {
                return Some(address);
            }
            _ => {}
        }
    }

    None
}

fn print_probe_result(op: &str, address: u8, result: Result<(), I2cError>) {
    match result {
        Ok(()) => esp_println::println!("  {} @ 0x{:02X}: OK", op, address),
        Err(error) => esp_println::println!("  {} @ 0x{:02X}: {}", op, address, error),
    }
}

pub fn run_i2c_joystick_diagnostics(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    discovered_address: Option<u8>,
) {
    esp_println::println!("I2C joystick diagnostics (candidate addresses):");

    let mut addresses = CANDIDATE_ADDRESSES;
    if let Some(discovered) = discovered_address {
        addresses[0] = discovered;
    }

    for address in addresses {
        let mut read1 = [0u8; 1];
        let mut read4 = [0u8; 4];

        print_probe_result("write-empty", address, i2c.write(address, &[]));
        print_probe_result("write-reg0", address, i2c.write(address, &[START_REGISTER]));
        print_probe_result(
            "write-read1",
            address,
            i2c.write_read(address, &[START_REGISTER], &mut read1),
        );
        print_probe_result("read1", address, i2c.read(address, &mut read1));
        print_probe_result(
            "write-read4",
            address,
            i2c.write_read(address, &[START_REGISTER], &mut read4),
        );
    }
}

fn capture_probe_windows(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    address: u8,
    windows: &mut [[u8; PROBE_WINDOW_SIZE]; PROBE_WINDOW_COUNT],
    valid: &mut [bool; PROBE_WINDOW_COUNT],
) {
    for (idx, start_reg) in PROBE_WINDOWS.iter().copied().enumerate() {
        valid[idx] = i2c
            .write_read(address, &[start_reg], &mut windows[idx])
            .is_ok();
    }
}

pub fn run_joystick_dynamic_probe(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    delay: &Delay,
    address: u8,
) {
    esp_println::println!(
        "Dynamic joystick probe on 0x{:02X}: move stick and press buttons now...",
        address
    );

    let mut baseline = [[0u8; PROBE_WINDOW_SIZE]; PROBE_WINDOW_COUNT];
    let mut current = [[0u8; PROBE_WINDOW_SIZE]; PROBE_WINDOW_COUNT];
    let mut baseline_valid = [false; PROBE_WINDOW_COUNT];
    let mut current_valid = [false; PROBE_WINDOW_COUNT];
    let mut seen_change = [false; PROBE_WINDOW_COUNT * PROBE_WINDOW_SIZE];
    let mut printable_changes: u16 = 0;

    capture_probe_windows(i2c, address, &mut baseline, &mut baseline_valid);

    for (idx, start_reg) in PROBE_WINDOWS.iter().copied().enumerate() {
        if baseline_valid[idx] {
            esp_println::println!("  Baseline reg 0x{:02X}: {:02X?}", start_reg, baseline[idx]);
        }
    }

    for sample_idx in 1..=PROBE_SAMPLES {
        delay.delay(Duration::from_millis(PROBE_INTERVAL_MS));
        capture_probe_windows(i2c, address, &mut current, &mut current_valid);

        for window_idx in 0..PROBE_WINDOW_COUNT {
            if !baseline_valid[window_idx] || !current_valid[window_idx] {
                continue;
            }

            for byte_idx in 0..PROBE_WINDOW_SIZE {
                let old = baseline[window_idx][byte_idx];
                let new = current[window_idx][byte_idx];

                if old != new {
                    let linear_idx = window_idx * PROBE_WINDOW_SIZE + byte_idx;
                    if !seen_change[linear_idx] && printable_changes < PROBE_MAX_CHANGE_PRINTS {
                        seen_change[linear_idx] = true;
                        printable_changes += 1;
                        let reg = PROBE_WINDOWS[window_idx].wrapping_add(byte_idx as u8);
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
