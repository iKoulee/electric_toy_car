# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Dual-ESP32-C6 firmware for a remote-controlled electric toy car. Two boards communicate via ESP-NOW:
- **Controller board** — reads an I2C joystick, transmits `ControlPacket` at 100 ms keepalive intervals
- **Vehicle board** — receives packets, drives motors via H-bridge, stops motors on link timeout (fail-safe)

## Build & Flash Commands

```bash
# Check all crates
cargo check

# Build a specific board (release is required for flashing)
cargo build -p controller --release
cargo build -p vehicle --release

# Flash via espflash (configured as cargo runner in .cargo/config.toml)
cargo run -p controller --release
cargo run -p vehicle --release

# Run host-side tests (common_comms only — no hardware needed)
cargo test -p common_comms
```

Target: `riscv32imac-unknown-none-elf` (ESP32-C6). The runner `espflash flash --monitor` is wired in `.cargo/config.toml`, so `cargo run` flashes and opens the serial monitor.

## Workspace Crates

| Crate | Role |
|---|---|
| `common_comms` | Protocol, ESP-NOW transport trait, link watchdog — fully testable on host |
| `common_led` | WS2812B LED helper via RMT (`set_rgb`) |
| `controller` | Async firmware (Embassy + esp-rtos): joystick → TX; ESP-NOW pairing + USB-host tunnel |
| `vehicle` | Async firmware (Embassy + esp-rtos): RX → motor control (fail-safe supervised); ESP-NOW pairing + USB-host tunnel |

## Architecture

### Communication protocol (`common_comms`)
- Every ESP-NOW payload is wrapped in a 1-byte `FrameKind` envelope (`frame.rs`): `Control`, `TunnelCmd`, `TunnelEvt`, `PairAck` — see `docs/espnow-shared-protocol.md`
- `ControlPacket` is 8 bytes, little-endian: `seq: u16`, `x: u8`, `y: u8`, `buttons: u8`, `reserved: [u8; 3]` (no checksum — ESP-NOW CRCs the frame)
- Sequence freshness: delta must be in `(0, 0x8000)` — drops stale/replayed packets; applies only to `Control` frames
- `EspNowTransport` trait abstracts the transport (send/receive + peer management); `EspNowLink` is a single bidirectional wrapper (control + tunnel + pairing) used by both boards — host-testable via a mock transport
- Pairing: peer MACs are learned during a broadcast→unicast handshake and persisted in the `nvs` flash partition (`pairing.rs`, pure record; board-side flash I/O in each firmware crate)
- `LinkWatchdog` is a pure state machine (`AwaitingFirstPacket → Alive → TimedOut`) driven by elapsed-time updates; timeout threshold is `LINK_TIMEOUT_MS = 500`

### Controller (`controller/src/main.rs`)
- Async with `esp_rtos::main` and Embassy executor
- I2C0 on GPIO6/GPIO7 at 100 kHz; joystick detected by scanning addresses `[0x5A, 0x24, 0x12, 0x48]`
- 10 ms tick loop; transmits on state change or every 100 ms (keepalive); sends neutral packet after 3+ consecutive I2C failures
- Also drains inbound ESP-NOW frames each tick: `PairAck` (learn/persist the vehicle MAC), and tunnel frames when acting as USB gateway/remote

### Vehicle (`vehicle/src/main.rs`)
- Async with `esp_rtos::main` and Embassy executor; 50 ms tick loop
- `setup()` initialises radio/transport/LED; `run()` contains the main control loop
- LED reflects link state: Yellow = awaiting, Green = alive, Red blink = timed out (fail-safe)
- Motor drive goes through a single per-tick apply stage (priority: manual override > safety brake > dead-zone coast > ramped drive). Joystick duty is fed through a **slew-rate limiter** (`drive::ramp_duty`, `ACCEL_STEP`/`DECEL_STEP`) so PWM changes are gradual — there is no speed sensor, so a closed-loop PID is not applicable
- Releasing the joystick to the dead zone **coasts** (`HBridge::coast` drops both EN pins *before* zeroing PWM) so the motor freewheels instead of hard-braking. `set_pwm(0)` with EN high is an electrodynamic brake, not a coast — that distinction matters on the IBT-2/BTS7960
- On link timeout: `brake()` (H-bridges stay enabled) so USB `SetMotorPwm` commands take effect immediately — a host computer controlling the car over USB cable can override in this state
- USB/tunnel `SetMotorPwm` is a **latched manual override** (`VehicleRunState::manual_pwm`): re-asserted every tick so control packets and the timeout brake can't zero it. Cleared automatically when the physical operator reclaims control (joystick out of dead zone, or brake button) — safety wins over remote control. `SetManualPwmRamp { on }` toggles whether the override ramps or applies instantly (default instant)

### USB-host gateway (both boards)
- The board on USB acts as a **gateway**; the other is the **remote**. The PC controls/monitors both through one USB link — see `docs/espnow-shared-protocol.md`
- `HostToBoard::ForPeer(bytes)` relays a raw host command to the remote over the tunnel; `BoardToHost::FromPeer { source, .. }` returns the remote's telemetry, source-tagged
- Remote telemetry streaming is off by default; enable per-board with `EnableRemoteTelemetry { on }`. `Repair` (or BOOT/GPIO9 at reset) clears the stored pairing
- `pitwall` (host-side TUI dashboard, `pitwall/` — standalone crate outside the workspace) commands: `peer <cmd>`, `remote_tele on|off`, `manual_pwm_ramp on|off`, `repair`

### Shared LED (`common_led`)
- `new_ws2812(rmt_channel, gpio_pin, clocks)` → `SmartLedsAdapter`
- `set_rgb(led, r, g, b)` — single WS2812B on GPIO8 (RMT)

## Embedded Constraints

- **No `std`**, no heap allocation in hot paths, no blocking delays in control loops
- Motor H-bridge: the IBT-2 (BTS7960) includes internal shoot-through protection — software dead-time is not required; zero the inactive channel before activating the active channel as a belt-and-suspenders measure
- All hardware bus errors (I2C, ESP-NOW) must be handled — never panic in production paths
- RISC-V only — do not suggest Xtensa-specific features or assembly
- Both controller and vehicle use async (`embassy-executor` via `esp_rtos`)
