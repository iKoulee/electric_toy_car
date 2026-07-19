//! Physical GPIO map of the controller board (ESP32-C6) — the single source of truth
//! for which pin does what. esp-hal 1.0 makes every GPIO a distinct singleton type
//! moved by ownership, so the map cannot be a C-style `#define` table of numbers;
//! instead [`ControllerPins`] groups the typed pin handles and the [`controller_pins!`]
//! macro fills it straight from the HAL `peripherals`.
//!
//! ```text
//! GPIO6 / GPIO7 — I2C0 SDA / SCL (joystick)
//! GPIO8         — WS2812B RGB LED (RMT)
//! GPIO9         — BOOT button (re-pair gesture, Input pull-up)
//! ```
//! GPIO0–GPIO5 are unused.

use esp_hal::peripherals::{GPIO6, GPIO7, GPIO8, GPIO9};

/// The controller's GPIO pins, claimed from `peripherals`.
pub struct ControllerPins<'d> {
    /// I2C0 SDA (joystick).
    pub sda: GPIO6<'d>,
    /// I2C0 SCL (joystick).
    pub scl: GPIO7<'d>,
    /// WS2812B RGB status LED (RMT).
    pub led: GPIO8<'d>,
    /// BOOT button — re-pair gesture (Input pull-up).
    pub boot: GPIO9<'d>,
}

/// Build [`ControllerPins`] from the HAL `peripherals`. Expands in the caller's scope
/// so each `$p.GPIOx` is an ordinary partial move out of `peripherals`.
macro_rules! controller_pins {
    ($p:expr) => {
        $crate::board::ControllerPins {
            sda: $p.GPIO6,
            scl: $p.GPIO7,
            led: $p.GPIO8,
            boot: $p.GPIO9,
        }
    };
}
pub(crate) use controller_pins;
