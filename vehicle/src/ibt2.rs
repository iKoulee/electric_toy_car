//! IBT-2 (BTS7960) H-bridge driver: motor control **and** current sensing for the
//! one physical module, behind the [`HBridge`] trait so the control layer depends on
//! an abstraction and a different power stage can be dropped in later (DIP).
//!
//! # Current sensing
//!
//! Each BTS7960 half-bridge drives an `IS` pin that, in normal operation, sources a
//! current proportional to its high-side load current (datasheet §4.4.4). The IBT-2
//! ties each `IS` to GND through a sense resistor (nominally 1 kΩ); with the nominal
//! ratio `k_ILIS = I_L / I_IS ≈ 8500` that gives `V_IS = I_L / 8.5` → `8.5 mA/mV`.
//!
//! The two IS outputs are read with ADC1 on GPIO0 (`R_IS`) and GPIO1 (`L_IS`) using
//! calibrated curve reads that return millivolts directly.
//!
//! ## ⚠️ Calibration is UNVERIFIED
//!
//! Field measurements show a large (~1.84 V) reading at true-zero current, weakly
//! correlated with the real load — the signature of a **floating / unloaded IS pin**
//! (missing or wrong IS→GND resistor), and the active channel appears **swapped**
//! versus the direction assumption below. [`Ibt2::calibrate_offset`] captures the
//! idle baseline and [`read_current`](Ibt2::read_current) subtracts it and averages
//! several samples, but the scale ([`IS_SCALE_NUM`] / [`IS_SCALE_DEN`]) and the
//! R/L↔direction mapping still need a proper resistive-load sweep to lock in.
//!
//! ### Hardware verification + calibration procedure
//! 1. **Power off**, measure resistance IS→GND for each channel — expect ~1 kΩ.
//!    If open/high the IS pin is unloaded; fit a 1 kΩ (per datasheet) IS→GND resistor.
//!    This is the prime suspect for the phantom baseline.
//! 2. Confirm which physical IS pin maps to `R_IS`/`L_IS` and to GPIO0/GPIO1.
//! 3. Drive fixed PWM steps into a known power resistor, record multimeter amps vs the
//!    `CurrentSenseRaw` telemetry (raw mV + commanded duty) across the range, then
//!    linear-fit → [`IS_SCALE_NUM`]/[`IS_SCALE_DEN`] and confirm which channel
//!    responds to forward vs reverse.

use esp_hal::{
    analog::adc::{Adc, AdcCalCurve, AdcConfig, AdcPin, Attenuation},
    gpio::Output,
    ledc::{
        channel::{self, ChannelIFace},
        LowSpeed,
    },
    peripherals::{ADC1, GPIO0, GPIO1},
    Async,
};

/// ADC attenuation for the IS pins (full-scale ≈ 3.3 V).
const IS_ATTENUATION: Attenuation = Attenuation::_11dB;

/// Sense scale as a rational `num/den` mA-per-mV. **UNVERIFIED** — datasheet nominal
/// for the BTS7960 (`k_ILIS ≈ 8500`, 1 kΩ IS resistor) is `8.5 mA/mV = 17/2`; the
/// observed slope was closer to ~1.25 mA/mV, so this must be re-derived from a
/// resistive-load sweep (see module docs).
const IS_SCALE_NUM: u32 = 17;
const IS_SCALE_DEN: u32 = 2;

/// Samples averaged per channel in [`Ibt2::read_current`] to suppress PWM-chop noise.
const AVG_SAMPLES: u32 = 8;
/// Samples averaged per channel when capturing the idle baseline.
const CAL_SAMPLES: u32 = 32;

type IsPin<'d, GPIO> = AdcPin<GPIO, ADC1<'d>, AdcCalCurve<ADC1<'d>>>;

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
/// concrete driver. `set_pwm` takes a signed duty (`+` forward / `-` reverse); the
/// joystick→duty mapping is a control-layer concern (see `drive.rs`).
#[allow(async_fn_in_trait)] // crate-internal, single-threaded embassy use; no Send needed
pub trait HBridge {
    /// Directly set PWM duty (-100–100). `+` → RPWM active, `-` → LPWM active. Note
    /// that `0` with the enables still HIGH is an electrodynamic **brake** (both
    /// low-side FETs conduct, shorting the motor to GND), *not* a coast — use
    /// [`coast`](HBridge::coast)/[`stop`](HBridge::stop) (EN low) to freewheel.
    fn set_pwm(&mut self, duty: i8);
    /// Electrodynamic brake: both half-bridges enabled, both PWM inputs at 0.
    fn brake(&mut self);
    /// Coast / freewheel: drop both half-bridge enables (EN low → high-impedance
    /// outputs) so the motor spins down on its own. EN is lowered *before* PWM is
    /// zeroed so there is no low-side-brake transient. Electrically identical to
    /// [`stop`](HBridge::stop), but a normal control state rather than a fail-safe.
    fn coast(&mut self);
    /// Fail-safe: disable both half-bridges and zero PWM outputs.
    #[allow(dead_code)] // part of the driver API; not on the current control path
    fn stop(&mut self);
    /// Re-enable both half-bridges after `stop` before resuming drive.
    fn enable(&mut self);
    fn set_r_en(&mut self, high: bool);
    fn set_l_en(&mut self, high: bool);
    /// Read both IS channels (averaged, offset-subtracted, scaled).
    async fn read_current(&mut self) -> CurrentReading;
}

/// IBT-2 module: PWM/EN control and both IS current-sense ADC channels.
pub struct Ibt2<'d> {
    rpwm: channel::Channel<'d, LowSpeed>,
    lpwm: channel::Channel<'d, LowSpeed>,
    r_en: Output<'d>,
    l_en: Output<'d>,
    adc: Adc<'d, ADC1<'d>, Async>,
    r_is: IsPin<'d, GPIO0<'d>>,
    l_is: IsPin<'d, GPIO1<'d>>,
    /// Idle baselines captured by [`calibrate_offset`](Ibt2::calibrate_offset) (mV).
    r_offset_mv: u16,
    l_offset_mv: u16,
}

impl<'d> Ibt2<'d> {
    /// Wire up the control channels/enables and ADC1 curve-calibrated reads on the IS
    /// pins. `r_is` = forward/right high-side (GPIO0), `l_is` = reverse/left (GPIO1).
    pub fn new(
        rpwm: channel::Channel<'d, LowSpeed>,
        lpwm: channel::Channel<'d, LowSpeed>,
        r_en: Output<'d>,
        l_en: Output<'d>,
        adc1: ADC1<'d>,
        r_is: GPIO0<'d>,
        l_is: GPIO1<'d>,
    ) -> Self {
        let mut config = AdcConfig::new();
        let r_is = config.enable_pin_with_cal::<_, AdcCalCurve<ADC1<'d>>>(r_is, IS_ATTENUATION);
        let l_is = config.enable_pin_with_cal::<_, AdcCalCurve<ADC1<'d>>>(l_is, IS_ATTENUATION);
        let adc = Adc::new(adc1, config).into_async();
        Self {
            rpwm,
            lpwm,
            r_en,
            l_en,
            adc,
            r_is,
            l_is,
            r_offset_mv: 0,
            l_offset_mv: 0,
        }
    }

    /// Average `CAL_SAMPLES` reads of each IS channel and store them as the idle
    /// baseline. Call once at boot **while the motor is idle** (PWM = 0); the baseline
    /// is then subtracted by [`read_current`](Ibt2::read_current).
    pub async fn calibrate_offset(&mut self) {
        self.set_pwm(0);
        let mut r_sum = 0u32;
        let mut l_sum = 0u32;
        for _ in 0..CAL_SAMPLES {
            r_sum += self.adc.read_oneshot(&mut self.r_is).await as u32;
            l_sum += self.adc.read_oneshot(&mut self.l_is).await as u32;
        }
        self.r_offset_mv = (r_sum / CAL_SAMPLES) as u16;
        self.l_offset_mv = (l_sum / CAL_SAMPLES) as u16;
    }

    async fn avg_mv(&mut self, channel: IsChannel) -> u16 {
        let mut sum = 0u32;
        for _ in 0..AVG_SAMPLES {
            sum += match channel {
                IsChannel::R => self.adc.read_oneshot(&mut self.r_is).await as u32,
                IsChannel::L => self.adc.read_oneshot(&mut self.l_is).await as u32,
            };
        }
        (sum / AVG_SAMPLES) as u16
    }
}

enum IsChannel {
    R,
    L,
}

impl HBridge for Ibt2<'_> {
    /// The IBT-2 uses BTS7960 half-bridge drivers with internal shoot-through
    /// protection and matched propagation delays, so software dead-time is not
    /// required. Zeroing the inactive channel before activating the active one is kept
    /// as a belt-and-suspenders measure.
    fn set_pwm(&mut self, duty: i8) {
        let pct = duty.unsigned_abs().min(100);
        if duty > 0 {
            let _ = self.lpwm.set_duty(0);
            let _ = self.rpwm.set_duty(pct);
        } else if duty < 0 {
            let _ = self.rpwm.set_duty(0);
            let _ = self.lpwm.set_duty(pct);
        } else {
            let _ = self.rpwm.set_duty(0);
            let _ = self.lpwm.set_duty(0);
        }
    }

    /// Both low-side FETs conduct, shorting the motor terminals to GND and dissipating
    /// back-EMF as braking torque. H-bridges stay enabled.
    fn brake(&mut self) {
        let _ = self.rpwm.set_duty(0);
        let _ = self.lpwm.set_duty(0);
        self.r_en.set_high();
        self.l_en.set_high();
    }

    fn coast(&mut self) {
        // Drop the enables first so the outputs go high-impedance before PWM is
        // zeroed — no momentary low-side brake as the duty falls to 0.
        self.r_en.set_low();
        self.l_en.set_low();
        let _ = self.rpwm.set_duty(0);
        let _ = self.lpwm.set_duty(0);
    }

    fn stop(&mut self) {
        let _ = self.rpwm.set_duty(0);
        let _ = self.lpwm.set_duty(0);
        self.r_en.set_low();
        self.l_en.set_low();
    }

    fn enable(&mut self) {
        self.r_en.set_high();
        self.l_en.set_high();
    }

    fn set_r_en(&mut self, high: bool) {
        if high {
            self.r_en.set_high()
        } else {
            self.r_en.set_low()
        }
    }

    fn set_l_en(&mut self, high: bool) {
        if high {
            self.l_en.set_high()
        } else {
            self.l_en.set_low()
        }
    }

    async fn read_current(&mut self) -> CurrentReading {
        let r_mv = self.avg_mv(IsChannel::R).await;
        let l_mv = self.avg_mv(IsChannel::L).await;
        CurrentReading {
            r_ma: mv_to_ma(r_mv, self.r_offset_mv),
            l_ma: mv_to_ma(l_mv, self.l_offset_mv),
            r_mv,
            l_mv,
        }
    }
}

/// Convert a raw sense voltage (mV) to load current (mA): subtract the idle baseline,
/// then apply the `IS_SCALE_NUM/IS_SCALE_DEN` scale, saturating into `u16`.
fn mv_to_ma(mv: u16, offset_mv: u16) -> u16 {
    let net = mv.saturating_sub(offset_mv) as u32;
    ((net * IS_SCALE_NUM) / IS_SCALE_DEN).min(u16::MAX as u32) as u16
}

#[cfg(test)]
mod tests {
    use super::mv_to_ma;

    #[test]
    fn conversion_matches_datasheet_ratio() {
        // No offset: 1 kΩ, ratio 8.5 → 1000 mV -> 8.5 A.
        assert_eq!(mv_to_ma(0, 0), 0);
        assert_eq!(mv_to_ma(1000, 0), 8500);
        // Saturates rather than wrapping past the u16 ceiling.
        assert_eq!(mv_to_ma(u16::MAX, 0), u16::MAX);
    }

    #[test]
    fn offset_is_subtracted_and_saturates_at_zero() {
        // Baseline removed before scaling.
        assert_eq!(mv_to_ma(1200, 200), 8500);
        // Readings below the baseline clamp to zero, never wrap.
        assert_eq!(mv_to_ma(100, 200), 0);
    }
}
