//! IBT-2 (BTS7960) current-sense reader.
//!
//! Each BTS7960 half-bridge drives an `IS` pin that, in normal operation,
//! sources a current proportional to its high-side load current (datasheet
//! §4.4.4, "current sense mode"). The IBT-2 module ties each `IS` to GND through
//! a 1 kΩ resistor, so with the nominal ratio `k_ILIS = I_L / I_IS ≈ 8500`:
//!
//! ```text
//! V_IS = I_L / 8.5   (volts)   =>   I_L[mA] = V_IS[mV] * 8.5
//! ```
//!
//! Only the *active* high-side switch sources sense current, so during forward
//! drive `R_IS` reads the load and `L_IS` reads ~0, and vice-versa in reverse.
//!
//! The two IS outputs are read with ADC1 on GPIO0 (`R_IS`) and GPIO1 (`L_IS`)
//! using calibrated curve reads, which return millivolts directly. At the `11dB`
//! attenuation used here the ADC full-scale is ≈3.3 V, so the reading saturates
//! around `3.3 V * 8.5 ≈ 28 A` — well above what a toy car draws.

use esp_hal::{
    analog::adc::{Adc, AdcCalCurve, AdcConfig, AdcPin, Attenuation},
    peripherals::{ADC1, GPIO0, GPIO1},
    Async,
};

/// ADC attenuation for the IS pins (full-scale ≈ 3.3 V).
const IS_ATTENUATION: Attenuation = Attenuation::_11dB;

type IsPin<'d, GPIO> = AdcPin<GPIO, ADC1<'d>, AdcCalCurve<ADC1<'d>>>;

/// Reads both IBT-2 current-sense channels and converts them to milliamps.
pub struct CurrentSense<'d> {
    adc: Adc<'d, ADC1<'d>, Async>,
    r_is: IsPin<'d, GPIO0<'d>>,
    l_is: IsPin<'d, GPIO1<'d>>,
}

impl<'d> CurrentSense<'d> {
    /// Wire up ADC1 with calibrated curve reads on the two IS pins.
    /// `r_is` = forward/right high-side (GPIO0), `l_is` = reverse/left (GPIO1).
    pub fn new(adc1: ADC1<'d>, r_is: GPIO0<'d>, l_is: GPIO1<'d>) -> Self {
        let mut config = AdcConfig::new();
        let r_is = config.enable_pin_with_cal::<_, AdcCalCurve<ADC1<'d>>>(r_is, IS_ATTENUATION);
        let l_is = config.enable_pin_with_cal::<_, AdcCalCurve<ADC1<'d>>>(l_is, IS_ATTENUATION);
        let adc = Adc::new(adc1, config).into_async();
        Self { adc, r_is, l_is }
    }

    /// Read both IS channels and return their load currents `(r_ma, l_ma)` in mA.
    pub async fn read_ma(&mut self) -> (u16, u16) {
        let r_mv = self.adc.read_oneshot(&mut self.r_is).await;
        let l_mv = self.adc.read_oneshot(&mut self.l_is).await;
        (mv_to_ma(r_mv), mv_to_ma(l_mv))
    }
}

/// Convert a sense voltage in millivolts to load current in milliamps.
/// `I_L[mA] = V_IS[mV] * 8.5`, saturating into `u16` at the ADC ceiling.
fn mv_to_ma(mv: u16) -> u16 {
    ((mv as u32 * 17) / 2).min(u16::MAX as u32) as u16
}

#[cfg(test)]
mod tests {
    use super::mv_to_ma;

    #[test]
    fn conversion_matches_datasheet_ratio() {
        assert_eq!(mv_to_ma(0), 0);
        // 1 kΩ, ratio 8.5: 1000 mV -> 8.5 A.
        assert_eq!(mv_to_ma(1000), 8500);
        // Saturates rather than wrapping past the u16 ceiling.
        assert_eq!(mv_to_ma(u16::MAX), u16::MAX);
    }
}
