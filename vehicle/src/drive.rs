//! Control-layer policy that maps joystick axes to signed motor duties.
//!
//! This is deliberately separate from the power-stage drivers (SRP): a driver only
//! knows about a signed PWM duty, while dead-zone and range shaping are a control
//! decision that a different input source could replace. The Y axis drives traction
//! (IBT-2); the X axis steers (L298N).

use crate::config::motor::{CENTER, DEAD_ZONE};

/// Map a joystick axis (0–255, center = 127, dead-zone ±10) to a signed duty level
/// (positive = high end, negative = low end, 0 = centred), clamped to ±100.
fn axis_to_duty(v: u8) -> i16 {
    let offset = (v as i16) - (CENTER as i16);
    if offset.unsigned_abs() <= DEAD_ZONE as u16 {
        0
    } else if offset > 0 {
        ((offset as u16 * 100) / (255 - CENTER as u16)).min(100) as i16
    } else {
        -((((-offset) as u16 * 100) / CENTER as u16).min(100) as i16)
    }
}

/// Convert joystick Y to a traction duty (positive = forward, negative = reverse,
/// 0 = coast), clamped to ±100.
pub fn y_to_duty(y: u8) -> i16 {
    axis_to_duty(y)
}

/// Convert joystick X to a steering duty (positive = one side, negative = the other,
/// 0 = centred), clamped to ±100.
pub fn x_to_steer(x: u8) -> i16 {
    axis_to_duty(x)
}

#[cfg(test)]
mod tests {
    use super::{x_to_steer, y_to_duty};

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
    fn steering_shares_the_axis_mapping() {
        assert_eq!(x_to_steer(127), 0);
        assert_eq!(x_to_steer(127 + 10), 0);
        assert_eq!(x_to_steer(255), 100);
        assert_eq!(x_to_steer(0), -100);
    }
}
