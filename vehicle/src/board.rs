//! Physical GPIO map of the vehicle board (ESP32-C6) — the single source of truth
//! for which pin does what. esp-hal 1.0 makes every GPIO a distinct singleton type
//! moved by ownership, so the map cannot be a C-style `#define` table of numbers;
//! instead [`VehiclePins`] groups the typed pin handles and the [`vehicle_pins!`]
//! macro fills it straight from the HAL `peripherals`.
//!
//! ```text
//! GPIO0 / GPIO1 — traction R_IS / L_IS current sense (ADC1)
//! GPIO2 / GPIO3 — traction RPWM / LPWM              (LEDC Ch0 / Ch1)
//! GPIO4 / GPIO5 — traction R_EN / L_EN              (Output)
//! GPIO6 / GPIO7 — steering IN1 / IN2 (L298N)        (LEDC Ch2 / Ch3)
//! GPIO8         — WS2812B RGB LED                   (RMT)
//! GPIO9         — BOOT button (re-pair gesture)     (Input pull-up)
//! ```

use esp_hal::peripherals::{GPIO0, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO7, GPIO8, GPIO9};

/// The vehicle's GPIO pins, claimed from `peripherals` and handed to the drivers.
pub struct VehiclePins<'d> {
    /// Traction R_IS current sense (ADC1).
    pub r_is: GPIO0<'d>,
    /// Traction L_IS current sense (ADC1).
    pub l_is: GPIO1<'d>,
    /// Traction RPWM (LEDC Channel0).
    pub rpwm: GPIO2<'d>,
    /// Traction LPWM (LEDC Channel1).
    pub lpwm: GPIO3<'d>,
    /// Traction R_EN enable.
    pub r_en: GPIO4<'d>,
    /// Traction L_EN enable.
    pub l_en: GPIO5<'d>,
    /// Steering L298N IN1 (LEDC Channel2).
    pub steer_in1: GPIO6<'d>,
    /// Steering L298N IN2 (LEDC Channel3).
    pub steer_in2: GPIO7<'d>,
    /// WS2812B RGB status LED (RMT).
    pub led: GPIO8<'d>,
    /// BOOT button — re-pair gesture (Input pull-up).
    pub boot: GPIO9<'d>,
}

/// Build [`VehiclePins`] from the HAL `peripherals`. Expands in the caller's scope so
/// each `$p.GPIOx` is an ordinary partial move out of `peripherals`.
macro_rules! vehicle_pins {
    ($p:expr) => {
        $crate::board::VehiclePins {
            r_is: $p.GPIO0,
            l_is: $p.GPIO1,
            rpwm: $p.GPIO2,
            lpwm: $p.GPIO3,
            r_en: $p.GPIO4,
            l_en: $p.GPIO5,
            steer_in1: $p.GPIO6,
            steer_in2: $p.GPIO7,
            led: $p.GPIO8,
            boot: $p.GPIO9,
        }
    };
}
pub(crate) use vehicle_pins;
