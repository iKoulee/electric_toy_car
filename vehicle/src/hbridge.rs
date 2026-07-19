//! Motor power-stage abstraction shared by every driver (DIP): the control layer
//! depends on this trait, not on a concrete driver, so different power stages
//! (IBT-2, L298N, …) can be dropped in without touching the control loop.
//!
//! Current sensing is part of the same trait but **optional** — [`HBridge::read_current`]
//! returns `Option<CurrentReading>` so a driver without a usable current-sense output
//! (e.g. an L298N module with its SENSE pins grounded) can report `None`.

/// One current-sense reading: converted load current (mA) plus the raw averaged
/// sense voltage (mV) for calibration/telemetry.
#[derive(Debug, Clone, Copy, Default)]
pub struct CurrentReading {
    /// Forward/right high-side (`R_IS`) load current, offset-subtracted and scaled.
    pub r_ma: u16,
    /// Reverse/left (`L_IS`) load current, offset-subtracted and scaled.
    pub l_ma: u16,
    /// Raw averaged `R_IS` sense voltage in mV (before offset subtraction).
    pub r_mv: u16,
    /// Raw averaged `L_IS` sense voltage in mV (before offset subtraction).
    pub l_mv: u16,
}

/// Abstraction over the motor power stage so the control layer does not depend on a
/// concrete driver. `set_pwm` takes a signed duty (`+` forward / `-` reverse / `0`
/// coast); the joystick→duty mapping is a control-layer concern (see `drive.rs`).
#[allow(async_fn_in_trait)] // crate-internal, single-threaded embassy use; no Send needed
pub trait HBridge {
    /// Directly set PWM duty (-100–100). `+` → RPWM active, `-` → LPWM active, `0` → coast.
    fn set_pwm(&mut self, duty: i8);
    /// Electrodynamic brake: both half-bridges enabled, both PWM inputs at 0.
    fn brake(&mut self);
    /// Fail-safe: disable both half-bridges and zero PWM outputs.
    #[allow(dead_code)] // part of the driver API; not on the current control path
    fn stop(&mut self);
    /// Re-enable both half-bridges after `stop` before resuming drive.
    fn enable(&mut self);
    fn set_r_en(&mut self, high: bool);
    fn set_l_en(&mut self, high: bool);
    /// Read the current-sense channels, or `None` if this driver has no current sense.
    async fn read_current(&mut self) -> Option<CurrentReading>;
}
