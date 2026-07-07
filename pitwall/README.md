# pitwall

Host-side terminal telemetry **dashboard** for the electric toy car. Connect either
board (controller or vehicle) to your computer with a USB cable — `pitwall` shows the
live telemetry of **both** boards side by side and lets you send commands.

The name is a racing metaphor: the *pit wall* is where the team watches the car's
telemetry during a session.

```
┌ Controller ──────────┐ ┌ Vehicle ─────────────┐
│ Link: ● Alive        │ │ Link: ● Alive        │
│ Btn: JOY C A B D     │ │ Rx pkt:  x=200 y=180 │
│ Joy X: 200           │ │ Btn: JOY C A B D     │
│ ▁▂▄▆█▇▅▃▂            │ │ Motor +42% ▓▓▓▓▓░░░░ │
│ Joy Y: 180           │ │ ▁▂▄▆█▇▅▃▂            │
│ ▂▃▅▇▆▄▂             │ │                      │
└──────────────────────┘ └──────────────────────┘
┌ Log ──────────────────────────────────────────┐
│ [PONG] version=1 board=Vehicle                 │
│ [Controller] [JOYSTICK] x=200 y=180 buttons=0x0│
│ [MOTOR] duty=42                                │
└────────────────────────────────────────────────┘
┌ /dev/ttyACM0 · gw=Vehicle · ● connected · Enter=send  Esc=quit ┐
│ > motor_pwm 40                                                 │
└────────────────────────────────────────────────────────────────┘
```

## Windows users (no build required)

1. Download `pitwall-windows` (`pitwall.exe`) from the latest **pitwall** GitHub
   Actions run (Actions tab → a green run → *Artifacts*).
2. Plug a board in via USB.
3. Double-click `pitwall.exe`, or run it in a terminal: `pitwall.exe`. With no
   `--port`, it auto-detects the serial port; if several exist it lists the `COMx`
   ports and asks you to pick one.

## Running from source

`pitwall` is a **standalone crate**, deliberately excluded from the embedded
workspace (which targets `riscv32imac`). Build/run it from this directory.

```bash
# Linux / macOS — auto-detect the port
cargo run

# …or name it explicitly
cargo run -- --port /dev/ttyACM0
```

`pitwall/.cargo/config.toml` pins the Linux-host target `x86_64-unknown-linux-gnu`,
so a bare `cargo run` works out of the box on Linux. On **Windows or macOS** that
pinned Linux target is wrong for your host, so pass your own native target:

```powershell
# Windows
cargo run --target x86_64-pc-windows-msvc
```

```bash
# macOS (Apple silicon)
cargo run --target aarch64-apple-darwin
```

> Stable Rust is sufficient — the repo config's `[unstable]` options are ignored on
> stable toolchains.

## Commands (type into the bottom bar, press Enter)

| Command | Effect |
|---|---|
| `ping` | Ping the connected board |
| `led R G B` / `led off` | Override the onboard LED / restore auto |
| `motor_en R_EN L_EN` | Vehicle: set IBT-2 enable pins (`on/off`) |
| `motor_pwm DUTY` | Vehicle: set motor PWM, `-100..100` |
| `remote_tele on\|off` | Stream the peer board's telemetry over the tunnel |
| `repair` | Forget pairing and re-run the handshake |
| `peer <cmd>` | Relay any command to the paired peer board |
| `quit` / `q` / `Esc` | Exit |

`remote_tele on` (usually as `peer remote_tele on`) makes the second board's panel
fill in via relayed `FromPeer` telemetry.
