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

    /// Electrodynamic brake: both half-bridges enabled, both PWM inputs at 0.
    /// Both low-side FETs conduct, shorting the motor terminals to GND and
    /// dissipating back-EMF as braking torque.
    pub fn brake(&mut self) {
        let _ = self.rpwm.set_duty(0);
        let _ = self.lpwm.set_duty(0);
        self.r_en.set_high();
        self.l_en.set_high();
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

    pub fn set_r_en(&mut self, high: bool) {
        if high { self.r_en.set_high() } else { self.r_en.set_low() }
    }

    pub fn set_l_en(&mut self, high: bool) {
        if high { self.l_en.set_high() } else { self.l_en.set_low() }
    }

    /// Directly set PWM duty (-100–100).
    /// Positive → RPWM active, LPWM=0. Negative → LPWM active, RPWM=0. Zero → coast.
    pub fn set_pwm(&mut self, duty: i8) {
        let pct = (duty.unsigned_abs()).min(100);
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
