//! Central tuning constants (numeric/bool) for the controller firmware.
//!
//! Grouped by concern so a parameter can be found and tuned without grepping the
//! logic modules. Every value here is `const`, so it is inlined at compile time —
//! no runtime cost. The physical GPIO map lives separately in [`crate::board`].
//!
//! `CONTROL_TX_INTERVAL_MS` (the keepalive/TX cadence) is intentionally *not* here:
//! it is a protocol-level constant owned by `common_comms::protocol`.

/// Control-loop and joystick-read timing.
pub mod timing {
    /// Control-loop tick period.
    pub const LOOP_TICK_MS: u64 = 10;
    /// Per-read timeout for the async joystick I2C transfer. At 10 kHz a 38-byte
    /// `write_read` can exceed 30 ms; keep this above the transfer worst-case.
    pub const READ_TIMEOUT_MS: u64 = 80;
    /// Minimum spacing between joystick status log lines.
    pub const STATUS_LOG_INTERVAL_MS: u64 = 250;
    /// Delay between samples in the diagnostic dynamic probe.
    pub const PROBE_INTERVAL_MS: u64 = 50;
}

/// Control-loop and radio buffering.
pub mod control {
    /// Max ESP-NOW frames drained per tick (pairing acks + tunnel frames).
    pub const MAX_RX_DRAIN_PER_TICK: usize = 8;
    /// Consecutive joystick read failures before sending a neutral keepalive.
    pub const READ_FAILURES_BEFORE_NEUTRAL_KEEPALIVE: u8 = 3;
}

/// Diagnostic / logging toggles and startup probes.
pub mod diag {
    /// Log every joystick sample (subject to [`PRINT_ON_CHANGE_ONLY`]).
    pub const SAMPLE_LOGS_ENABLED: bool = false;
    /// When sample logging is on, only print when the state changed.
    pub const PRINT_ON_CHANGE_ONLY: bool = true;
    /// Log each control-packet transmission.
    pub const TX_LOGS_ENABLED: bool = false;
    /// Print one error line every N consecutive read failures.
    pub const ERROR_LOG_PERIOD: u8 = 10;
    /// Run the I2C bus scan at startup.
    pub const RUN_SCAN: bool = true;
    /// Run the extra joystick register probes at startup.
    pub const RUN_STARTUP_PROBES: bool = false;
}

/// I2C bus configuration and address scan range.
pub mod i2c {
    /// I2C0 master clock.
    pub const FREQUENCY_KHZ: u32 = 100;
    /// First address probed by the bus scan.
    pub const SCAN_START_ADDR: u8 = 0x10;
    /// Last address probed by the bus scan.
    pub const SCAN_END_ADDR: u8 = 0x77;
}

/// Joystick register map, candidate addresses, and diagnostic probe parameters.
pub mod joystick {
    /// Default joystick I2C address.
    pub const DEFAULT_ADDRESS: u8 = 0x5A;
    /// Addresses tried when resolving the active joystick.
    pub const CANDIDATE_ADDRESSES: [u8; 4] = [0x5A, 0x24, 0x12, 0x48];

    /// Register the startup probes read from.
    pub const START_REGISTER: u8 = 0x00;
    /// First register of the runtime frame.
    pub const RUNTIME_START_REGISTER: u8 = 0x10;
    /// Runtime frame length (registers 0x10..=0x35).
    pub const RUNTIME_FRAME_LEN: usize = 0x26;

    /// Bytes per probe window.
    pub const PROBE_WINDOW_SIZE: usize = 8;
    /// Number of probe windows.
    pub const PROBE_WINDOW_COUNT: usize = 8;
    /// Start register of each probe window.
    pub const PROBE_WINDOWS: [u8; PROBE_WINDOW_COUNT] =
        [0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38];
    /// Samples captured per probe run.
    pub const PROBE_SAMPLES: u8 = 30;
    /// Cap on the number of per-byte change lines the probe prints.
    pub const PROBE_MAX_CHANGE_PRINTS: u16 = 64;
}
