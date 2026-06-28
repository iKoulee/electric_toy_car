use esp_hal::{
    gpio::Output,
    ledc::{channel::{self, ChannelIFace}, LowSpeed},
};

const DEAD_ZONE: u8 = 10;
const CENTER: u8 = 127;

pub struct Ibt2Motor<'d> {
    rpwm: channel::Channel<'d, LowSpeed>,
    lpwm: channel::Channel<'d, LowSpeed>,
    r_en: Output<'d>,
    l_en: Output<'d>,
}

impl<'d> Ibt2Motor<'d> {
    pub fn new(
        rpwm: channel::Channel<'d, LowSpeed>,
        lpwm: channel::Channel<'d, LowSpeed>,
        r_en: Output<'d>,
        l_en: Output<'d>,
    ) -> Self {
        Self { rpwm, lpwm, r_en, l_en }
    }

    /// Drive motor from joystick Y (0–255, center=127, dead-zone ±10).
    /// Forward: RPWM active. Reverse: LPWM active.
    pub fn set_drive(&mut self, y: u8) {
        let offset = (y as i16) - (CENTER as i16);
        if offset.unsigned_abs() <= DEAD_ZONE as u16 {
            let _ = self.rpwm.set_duty(0);
            let _ = self.lpwm.set_duty(0);
        } else if offset > 0 {
            let pct = ((offset as u16 * 100) / (255 - CENTER as u16)).min(100) as u8;
            let _ = self.lpwm.set_duty(0);
            let _ = self.rpwm.set_duty(pct);
        } else {
            let pct = ((-offset as u16 * 100) / CENTER as u16).min(100) as u8;
            let _ = self.rpwm.set_duty(0);
            let _ = self.lpwm.set_duty(pct);
        }
    }

    /// Fail-safe: disable both half-bridges and zero PWM outputs.
    pub fn stop(&mut self) {
        let _ = self.rpwm.set_duty(0);
        let _ = self.lpwm.set_duty(0);
        self.r_en.set_low();
        self.l_en.set_low();
    }

    /// Re-enable after stop before resuming set_drive.
    pub fn enable(&mut self) {
        self.r_en.set_high();
        self.l_en.set_high();
    }
}

/// Convert joystick Y to a signed duty level (positive = forward, negative = reverse, 0 = coast).
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
