#![no_std]

use esp_hal::{
    Blocking,
    gpio::interconnect::PeripheralOutput,
    rmt::TxChannelCreator,
};
use esp_hal_smartled::{
    AdapterError,
    RmtSmartLeds,
    buffer_size,
    color_order,
    Ws2812bTiming,
};
use smart_leds_trait::{
    RGB8,
    SmartLedsWrite,
};

pub const LED_COUNT: usize = 1;
pub const LED_BUFFER_SIZE: usize = buffer_size::<RGB8>(LED_COUNT);

pub type Ws2812Led<'a, const N: usize> =
    RmtSmartLeds<'a, N, Blocking, RGB8, color_order::Rgb, Ws2812bTiming>;

pub fn new_ws2812<'a, C, O, const N: usize>(channel: C, pin: O) -> Result<Ws2812Led<'a, N>, esp_hal::rmt::ConfigError>
where
    O: PeripheralOutput<'a>,
    C: TxChannelCreator<'a, Blocking>,
{
    Ws2812Led::new(channel, pin)
}

pub fn set_rgb<const N: usize>(
    led: &mut Ws2812Led<'_, N>,
    r: u8,
    g: u8,
    b: u8,
) -> Result<(), AdapterError> {
    led.write([RGB8 { r, g, b }].into_iter())
}
