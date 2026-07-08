//! Control-layer policy that maps a joystick Y axis to a signed motor duty.
//!
//! This is deliberately separate from the [`crate::ibt2`] power-stage driver (SRP):
//! the driver only knows about a signed PWM duty, while dead-zone and range shaping
//! are a control decision that a different input source could replace.

const DEAD_ZONE: u8 = 10;
const CENTER: u8 = 127;

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
    use super::y_to_duty;

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
}
