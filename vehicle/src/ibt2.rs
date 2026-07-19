//! IBT-2 (BTS7960) H-bridge driver: motor control **and** current sensing for the
//! one physical module, behind the [`HBridge`] trait so the control layer depends on
//! an abstraction and a different power stage can be dropped in later (DIP).
//!
//! # Current sensing
//!
//! Each BTS7960 half-bridge drives an `IS` pin that, in normal operation, sources a
//! current proportional to its high-side load current (datasheet §4.4.4). On this
//! board the `IS` output feeds an analog chain before the ADC:
//!
//! ```text
//! IS ──[ 10 kΩ IS→GND sense R ]── voltage divider ── RC low-pass ── ADC1 (GPIO0/1)
//! ```
//!
//! The IBT-2's `IS`→GND resistor is **10 kΩ** (not the 1 kΩ the datasheet examples
//! assume). With `k_ILIS = I_L / I_IS ≈ 8500` that raises `V_IS` above the 3.3 V ADC
//! reference at full load, so a **voltage divider** was added on the `IS` output to
//! bring it back into range. An **RC low-pass** (fc ≈ 30–100 Hz) on the divider node
//! smooths the ~9.77 kHz PWM ripple so the ADC sees stable DC — without it a handful
//! of ADC samples alias the pulse-train and the reading is noisy and biased low.
//!
//! The two divider/RC outputs are read with ADC1 on GPIO0 (`R_IS`) and GPIO1
//! (`L_IS`) using calibrated curve reads that return millivolts directly.
//!
//! ## Calibration
//!
//! Because the divider and RC ratio are baked into the measured slope, the scale is
//! derived empirically rather than from the datasheet ratio: a resistive-load sweep
//! (`docs/callibration_measurement.ods`) maps averaged ADC mV to the True-RMS load
//! current (`I_REF` column), giving ≈ 16 mA/mV → [`IS_SCALE_NUM`]/[`IS_SCALE_DEN`].
//! [`Ibt2::calibrate_offset`] captures the idle baseline (~120 mV) and
//! [`read_current`](Ibt2::read_current) subtracts it before scaling. R active on
//! forward PWM, L active on reverse — confirmed by the sweep. See
//! `docs/current-sense-calibration.md`.
//!
//! ### Re-calibration procedure
//! Drive fixed PWM steps into a known load, record `CurrentSenseRaw` (averaged mV +
//! commanded duty) vs the True-RMS load current across the range in both directions,
//! then linear-fit averaged-mV → mA and update [`IS_SCALE_NUM`]/[`IS_SCALE_DEN`].

use crate::config::current_sense::{
    AVG_SAMPLES, CAL_SAMPLES, IS_ATTENUATION, IS_SCALE_DEN, IS_SCALE_NUM,
};
use crate::hbridge::{CurrentReading, HBridge};
use esp_hal::{
    analog::adc::{Adc, AdcCalCurve, AdcConfig, AdcPin},
    gpio::Output,
    ledc::{
        channel::{self, ChannelIFace},
        LowSpeed,
    },
    peripherals::{ADC1, GPIO0, GPIO1},
    Async,
};

type IsPin<'d, GPIO> = AdcPin<GPIO, ADC1<'d>, AdcCalCurve<ADC1<'d>>>;

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

    async fn read_current(&mut self) -> Option<CurrentReading> {
        let r_mv = self.avg_mv(IsChannel::R).await;
        let l_mv = self.avg_mv(IsChannel::L).await;
        Some(CurrentReading {
            r_ma: mv_to_ma(r_mv, self.r_offset_mv),
            l_ma: mv_to_ma(l_mv, self.l_offset_mv),
            r_mv,
            l_mv,
        })
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
    fn conversion_matches_calibrated_scale() {
        // Empirical scale from the load sweep: 16 mA/mV → 1000 mV -> 16 A.
        assert_eq!(mv_to_ma(0, 0), 0);
        assert_eq!(mv_to_ma(1000, 0), 16000);
        // Saturates rather than wrapping past the u16 ceiling.
        assert_eq!(mv_to_ma(u16::MAX, 0), u16::MAX);
    }

    #[test]
    fn offset_is_subtracted_and_saturates_at_zero() {
        // Baseline removed before scaling: (1200 - 200) * 16 = 16000.
        assert_eq!(mv_to_ma(1200, 200), 16000);
        // Readings below the baseline clamp to zero, never wrap.
        assert_eq!(mv_to_ma(100, 200), 0);
    }
}
