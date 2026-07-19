//! L298N dual full-bridge driver, exposed through the shared [`HBridge`] trait so the
//! control layer treats it exactly like the IBT-2 (DIP).
//!
//! # Wiring / control model
//!
//! Only **bridge A** is used (steering motor); bridge B (`IN3`/`IN4`/`ENB`) is left
//! unused with its enable jumper on. Bridge A is driven in the same shape as the
//! IBT-2's two PWM inputs: `IN1` ≙ `RPWM`, `IN2` ≙ `LPWM`, with `ENA` held enabled.
//! The L298 datasheet supports PWM on the inputs while the enable is high (Fig. 2,
//! "For INPUT switching, set EN = H"), so speed comes from the input duty and
//! direction from which input is active:
//!
//! - forward → `IN1 = duty%`, `IN2 = 0`
//! - reverse → `IN1 = 0`, `IN2 = duty%`
//! - coast/brake → both inputs low
//!
//! `ENA` is optional here: on typical modules it is tied high with the onboard jumper
//! ([`L298n::new`] with `ena = None`), so [`enable`](L298n::enable)/[`stop`](L298n::stop)
//! become no-ops. Pass an [`Output`] to drive `ENA` from a GPIO instead.
//!
//! # No current sensing
//!
//! The SENSE A/B pins are grounded on this module, so [`read_current`](L298n::read_current)
//! returns `None`.

use crate::hbridge::{CurrentReading, HBridge};
use esp_hal::{
    gpio::Output,
    ledc::{
        channel::{self, ChannelIFace},
        LowSpeed,
    },
};

/// L298N bridge-A driver: two PWM inputs (`IN1`/`IN2`) and an optional `ENA` enable.
pub struct L298n<'d> {
    in1: channel::Channel<'d, LowSpeed>,
    in2: channel::Channel<'d, LowSpeed>,
    /// `ENA` enable line, or `None` when it is tied high with the module jumper.
    ena: Option<Output<'d>>,
}

impl<'d> L298n<'d> {
    /// Wire up bridge A. `in1`/`in2` are the two LEDC PWM channels; `ena` drives the
    /// `ENA` enable pin, or `None` to leave it on the module's always-on jumper.
    pub fn new(
        in1: channel::Channel<'d, LowSpeed>,
        in2: channel::Channel<'d, LowSpeed>,
        ena: Option<Output<'d>>,
    ) -> Self {
        Self { in1, in2, ena }
    }

    fn set_ena(&mut self, high: bool) {
        if let Some(ena) = self.ena.as_mut() {
            if high {
                ena.set_high();
            } else {
                ena.set_low();
            }
        }
    }
}

impl HBridge for L298n<'_> {
    /// Zero the inactive input before activating the active one, mirroring the IBT-2
    /// belt-and-suspenders ordering.
    fn set_pwm(&mut self, duty: i8) {
        let pct = duty.unsigned_abs().min(100);
        if duty > 0 {
            let _ = self.in2.set_duty(0);
            let _ = self.in1.set_duty(pct);
        } else if duty < 0 {
            let _ = self.in1.set_duty(0);
            let _ = self.in2.set_duty(pct);
        } else {
            let _ = self.in1.set_duty(0);
            let _ = self.in2.set_duty(0);
        }
    }

    /// Both inputs low with the bridge enabled shorts the motor terminals to GND.
    fn brake(&mut self) {
        let _ = self.in1.set_duty(0);
        let _ = self.in2.set_duty(0);
        self.set_ena(true);
    }

    fn stop(&mut self) {
        let _ = self.in1.set_duty(0);
        let _ = self.in2.set_duty(0);
        self.set_ena(false);
    }

    fn enable(&mut self) {
        self.set_ena(true);
    }

    fn set_r_en(&mut self, high: bool) {
        self.set_ena(high);
    }

    fn set_l_en(&mut self, high: bool) {
        self.set_ena(high);
    }

    /// No current sense on this module (SENSE pins grounded).
    async fn read_current(&mut self) -> Option<CurrentReading> {
        None
    }
}
