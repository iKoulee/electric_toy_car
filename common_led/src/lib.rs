#![no_std]

use esp_hal::{
    Blocking,
    gpio::interconnect::PeripheralOutput,
    rmt::{PulseCode, TxChannelCreator},
};
use esp_hal_smartled::{LedAdapterError, SmartLedsAdapter, buffer_size};
use rgb::Grb;
use smart_leds_trait::SmartLedsWrite;

pub use esp_hal::rmt::PulseCode as LedPulseCode;

pub const LED_COUNT: usize = 1;
pub const LED_BUFFER_SIZE: usize = buffer_size(LED_COUNT);

pub type Ws2812Led<'a, const N: usize> = SmartLedsAdapter<'a, N>;

/// Allocate an RMT pulse buffer sized for `LED_COUNT` LEDs.
/// Declare this before `new_ws2812` so it outlives the adapter:
/// ```ignore
/// let mut buf = ws2812_buffer!();
/// let mut led = new_ws2812(ch, pin, &mut buf);
/// ```
#[macro_export]
macro_rules! ws2812_buffer {
    () => {
        [$crate::LedPulseCode::end_marker(); $crate::LED_BUFFER_SIZE]
    };
}

pub fn new_ws2812<'a, C, O, const N: usize>(
    channel: C,
    pin: O,
    buffer: &'a mut [PulseCode; N],
) -> Ws2812Led<'a, N>
where
    O: PeripheralOutput<'a>,
    C: TxChannelCreator<'a, Blocking>,
{
    SmartLedsAdapter::new(channel, pin, buffer)
}

pub fn set_rgb<const N: usize>(
    led: &mut Ws2812Led<'_, N>,
    r: u8,
    g: u8,
    b: u8,
) -> Result<(), LedAdapterError> {
    // WS2812B expects GRB byte order; Grb<u8> ComponentSlice yields [g, r, b].
    led.write([Grb { g, r, b }])
}
