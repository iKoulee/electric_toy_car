# IBT-2 current-sense (IS) calibration

How the vehicle turns the BTS7960 `IS` pins into a load-current telemetry value,
and how to re-calibrate it.

## Analog chain

```
IS ──[ 10 kΩ IS→GND sense R ]── voltage divider ── RC low-pass ── ADC1
                                                                   R_IS → GPIO0
                                                                   L_IS → GPIO1
```

- **10 kΩ IS→GND resistor.** The IBT-2 on this board uses a **10 kΩ** `IS`→GND
  resistor, not the 1 kΩ the BTS7960 datasheet examples assume. With
  `k_ILIS = I_L / I_IS ≈ 8500`, that is `V_IS ≈ 1.18 V/A`, which exceeds the
  3.3 V ADC reference near full load.
- **Voltage divider.** Added on the `IS` output to bring the full-load voltage
  back below 3.3 V.
- **RC low-pass** (fc ≈ 30–100 Hz). The motor PWM runs at ~9.77 kHz, so the raw
  `IS` voltage is a pulse-train. Without filtering, the few ADC samples per read
  alias that pulse-train and the reading is noisy and biased low. The RC filter
  presents smooth DC to the ADC. Keep the ADC source impedance ≤ 10 kΩ (ESP32-C6
  requirement) — e.g. 2.2 kΩ + 1 µF (fc ≈ 72 Hz) or 10 kΩ + 1 µF (fc ≈ 16 Hz).
  Apply identically on both channels.

## Conversion

Firmware (`vehicle/src/ibt2.rs`): `current_mA = (V_mV − offset_mV) · scale`.

- `offset_mV` — idle baseline captured at boot by `Ibt2::calibrate_offset`
  (~120 mV), subtracted per channel.
- `scale` — `IS_SCALE_NUM / IS_SCALE_DEN`, currently **16 mA/mV**, derived
  empirically (the divider + RC ratio is baked in — the datasheet 8.5 mA/mV does
  **not** apply).

Channel mapping (confirmed by the sweep): **R active on forward PWM, L active on
reverse.**

## Where the scale comes from

`docs/callibration_measurement.ods`, sheet `List2`: a PWM sweep (0→±100) into a
load, recording per channel the ESP-read voltage, a meter-measured IS voltage,
and the load current. The **`I_REF`** column (True-RMS load current, ~0 A at
idle) is the calibration authority — the per-channel `R_I`/`L_I` columns carry a
~0.9 A idle phantom and are not used. A linear fit of averaged ADC mV → `I_REF`
gives ≈ 16 mA/mV, consistent across both directions.

## Re-calibration procedure

Re-run whenever the analog chain changes (resistor, divider, or RC values):

1. Flash the vehicle and open `pitwall`.
2. Drive fixed PWM steps into a known load across 0→100 and 0→−100.
3. At each step, record the `CurrentSenseRaw` averaged mV (now steady, thanks to
   the RC filter) against the True-RMS load current.
4. Least-squares fit averaged-mV → mA; update `IS_SCALE_NUM` / `IS_SCALE_DEN` in
   `vehicle/src/ibt2.rs` and confirm the reading reads ~0 A at idle and tracks
   the meter within a few percent in both directions.
