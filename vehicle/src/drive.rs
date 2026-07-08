//! Control-layer policy that maps a joystick Y axis to a signed motor duty.
//!
//! This is deliberately separate from the [`crate::ibt2`] power-stage driver (SRP):
//! the driver only knows about a signed PWM duty, while dead-zone and range shaping
//! are a control decision that a different input source could replace.

const DEAD_ZONE: u8 = 10;
const CENTER: u8 = 127;

/// Per-tick slew limit (duty units per 50 ms tick) while the speed magnitude is
/// increasing. 8/tick ⇒ ~0.6 s for a full 0→100 sweep (≈13 ticks). Tunable.
pub const ACCEL_STEP: i16 = 8;
/// Per-tick slew limit while slowing down or reversing toward zero. Larger than
/// [`ACCEL_STEP`] so the car sheds speed faster than it gains it (safety).
/// 12/tick ⇒ ~0.4 s for 100→0 (≈9 ticks). Tunable.
pub const DECEL_STEP: i16 = 12;

/// Advance `applied` one tick toward `target`, rate-limiting the change so PWM
/// steps stay smooth. The magnitude may move by at most `accel_step` when speeding
/// up (moving away from 0) or `decel_step` when slowing down / reversing toward 0.
/// The step is floored at 1 so a positive target is always eventually reached.
/// Never overshoots `target`.
pub fn ramp_duty(applied: i16, target: i16, accel_step: i16, decel_step: i16) -> i16 {
    let delta = target - applied;
    if delta == 0 {
        return applied;
    }
    let increasing_magnitude = (delta > 0 && applied >= 0) || (delta < 0 && applied <= 0);
    let step = if increasing_magnitude { accel_step } else { decel_step }.max(1);
    if delta.abs() <= step {
        target
    } else {
        applied + step * delta.signum()
    }
}

/// Convert joystick Y (0–255, center = 127, dead-zone ±10) to a signed duty level
/// (positive = forward, negative = reverse, 0 = coast), clamped to ±100.
pub fn y_to_duty(y: u8) -> i16 {
    let offset = (y as i16) - (CENTER as i16);
    if offset.unsigned_abs() <= DEAD_ZONE as u16 {
        0
    } else if offset > 0 {
        ((offset as u16 * 100) / (255 - CENTER as u16)).min(100) as i16
    } else {
        -((((-offset) as u16 * 100) / CENTER as u16).min(100) as i16)
    }
}

#[cfg(test)]
mod tests {
    use super::{ramp_duty, y_to_duty};

    #[test]
    fn center_and_dead_zone_are_coast() {
        assert_eq!(y_to_duty(127), 0);
        assert_eq!(y_to_duty(127 + 10), 0);
        assert_eq!(y_to_duty(127 - 10), 0);
    }

    #[test]
    fn full_scale_clamps_to_plus_minus_100() {
        assert_eq!(y_to_duty(255), 100);
        assert_eq!(y_to_duty(0), -100);
    }

    #[test]
    fn reaches_target_within_one_step() {
        assert_eq!(ramp_duty(0, 3, 8, 12), 3);
        assert_eq!(ramp_duty(0, 0, 8, 12), 0);
    }

    #[test]
    fn accel_is_rate_limited_and_symmetric() {
        assert_eq!(ramp_duty(0, 100, 8, 12), 8);
        assert_eq!(ramp_duty(20, 100, 8, 12), 28);
        assert_eq!(ramp_duty(-20, -100, 8, 12), -28);
    }

    #[test]
    fn decel_is_faster_than_accel() {
        assert_eq!(ramp_duty(100, 0, 8, 12), 88);
        assert_eq!(ramp_duty(50, 20, 8, 12), 38);
    }

    #[test]
    fn reversal_ramps_through_zero_without_jumping() {
        // From +10 toward -100: decelerate to 0 (staying positive), only then
        // accelerate the other way — never leaps across the sign in one tick.
        let a = ramp_duty(10, -100, 8, 12); // decel toward 0
        assert_eq!(a, -2);
        let b = ramp_duty(a, -100, 8, 12); // now accelerating in reverse
        assert_eq!(b, -10);
        let c = ramp_duty(b, -100, 8, 12);
        assert_eq!(c, -18);
    }

    #[test]
    fn never_overshoots() {
        assert_eq!(ramp_duty(98, 100, 8, 12), 100);
        assert_eq!(ramp_duty(-95, -100, 8, 12), -100);
    }

    #[test]
    fn zero_step_is_floored_to_one() {
        assert_eq!(ramp_duty(0, 100, 0, 12), 1);
    }
}
