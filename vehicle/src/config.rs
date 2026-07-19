//! Central tuning constants (numeric/bool) for the vehicle firmware.
//!
//! Grouped by concern so a parameter can be found and tuned without grepping the
//! logic modules. Every value here is `const`, so it is inlined at compile time —
//! no runtime cost. The physical GPIO map lives separately in [`crate::board`].

/// Joystick-axis → signed-duty shaping (see [`crate::drive`]).
pub mod motor {
    /// Half-width of the centred dead zone, in raw joystick counts.
    pub const DEAD_ZONE: u8 = 10;
    /// Joystick axis centre value (0–255 range).
    pub const CENTER: u8 = 127;
}

/// Control-loop timing.
pub mod timing {
    /// Vehicle control-loop tick period.
    pub const LOOP_INTERVAL_MS: u64 = 50;
    /// Max ESP-NOW frames drained per tick, so a burst of control + tunnel frames
    /// does not starve either path (the radio RX queue is 10 deep).
    pub const MAX_RX_DRAIN_PER_TICK: usize = 8;
}

/// IBT-2 (BTS7960) IS current-sense scaling and sampling (see [`crate::ibt2`]).
pub mod current_sense {
    use esp_hal::analog::adc::Attenuation;

    /// ADC attenuation for the IS pins (full-scale ≈ 3.3 V).
    pub const IS_ATTENUATION: Attenuation = Attenuation::_11dB;

    /// Sense scale as a rational `num/den` mA-per-mV, derived empirically from the
    /// resistive-load sweep in `docs/callibration_measurement.ods` (averaged ADC mV vs the
    /// `I_REF` True-RMS load current) — the divider + RC ratio is baked in, so this is not
    /// the datasheet `8.5 mA/mV`. Confirm against a fresh sweep after any change to the
    /// analog chain (see [`crate::ibt2`] module docs); provisional value pending final
    /// RC-filter build.
    pub const IS_SCALE_NUM: u32 = 16;
    pub const IS_SCALE_DEN: u32 = 1;

    /// Samples averaged per channel in `Ibt2::read_current`. The RC low-pass on the IS
    /// node does the heavy lifting; this averaging is cheap insurance against ADC noise.
    pub const AVG_SAMPLES: u32 = 16;
    /// Samples averaged per channel when capturing the idle baseline.
    pub const CAL_SAMPLES: u32 = 32;
}
