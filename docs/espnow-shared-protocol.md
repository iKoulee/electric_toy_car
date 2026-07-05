# Shared ESP-NOW Protocol

This document defines the protocol and link supervision shared by controller and
vehicle, implemented in `common_comms`.

Since the USB-gateway feature, every ESP-NOW payload is wrapped in a one-byte
**frame envelope** that multiplexes three concerns over the single radio link:
the control stream, a bidirectional host-protocol tunnel, and the pairing
handshake.

## Frame envelope (`common_comms::frame`)

Wire layout: `[kind: u8][body: ..]`

| `kind` | Name        | Body                                                         |
|--------|-------------|-------------------------------------------------------------|
| `0x01` | `Control`   | 8-byte `ControlPacket` (below)                              |
| `0x02` | `TunnelCmd` | opaque non-COBS postcard `HostToBoard` (gateway → remote)   |
| `0x03` | `TunnelEvt` | opaque non-COBS postcard `BoardToHost` (remote → gateway)   |
| `0x04` | `PairAck`   | empty; sender MAC is learned from the frame source address  |

Discriminants are the wire format — **append-only, never reorder**. Only
`Control` frames feed the sequence-freshness check and the link watchdog; tunnel
and pairing frames bypass both.

## Packet: `ControlPacket`

Length: 8 bytes (`CONTROL_PACKET_LEN`)

Byte layout (little-endian):

1. `sequence_lo`
2. `sequence_hi`
3. `x`
4. `y`
5. `buttons`
6. `reserved[0]` (currently `0`)
7. `reserved[1]` (currently `0`)
8. `reserved[2]` (currently `0`)

There is **no checksum** — ESP-NOW already provides a frame CRC at the link
layer, and the sequence field guards against stale/duplicate frames.

## Buttons bitfield

- bit 0: JOY
- bit 1: C
- bit 2: A
- bit 3: B
- bit 4: D

## Keepalive and timeout

- Transmit policy: send immediately on control-state change and also emit
  periodic keepalive packets.
- Keepalive interval target: `100 ms` (`CONTROL_TX_INTERVAL_MS`)
- Vehicle timeout threshold: `500 ms` (`LINK_TIMEOUT_MS`)

The command stream itself is the keepalive. No separate heartbeat frame is
required. The vehicle must immediately enter fail-safe stop (`brake()`, H-bridges
left enabled) if no valid fresh `Control` frame is received for more than
`500 ms`.

## Sequence freshness

The vehicle accepts a packet as newer when `delta = candidate.wrapping_sub(last)`
is in `(0, 0x8000)`. This handles wrap-around (`65535 -> 0`) while rejecting
duplicate and stale frames. Freshness applies **only** to `Control` frames.

## Pairing and unicast

All steady-state traffic is **unicast**; broadcast is used only to bootstrap
pairing. There is no encryption (this is a toy).

Bootstrap handshake:

1. An unpaired controller broadcasts `Control` frames (destination =
   `ff:ff:ff:ff:ff:ff`).
2. The vehicle receives a broadcast `Control`, learns the controller's MAC from
   the frame source address, adds it as a unicast peer, and persists it.
3. Because the frame was a broadcast, the vehicle replies with a `PairAck`
   (unicast) so the controller can learn the vehicle in return.
4. The controller receives the `PairAck`, learns the vehicle's MAC from its
   source address, persists it, and switches its control stream to unicast.

Once both sides have persisted the peer MAC, they come up in unicast directly on
the next boot without re-running the broadcast handshake.

Once paired, each board ignores `Control` and tunnel frames whose source is not
the paired peer, and ignores further `PairAck`s — so a second nearby car cannot
drive this vehicle or hijack the pairing.

### Persistence

The paired peer MAC is stored in the `nvs` flash partition as a small record
`[magic:u32][mac:6][crc16:u16]` (`common_comms::pairing`). An invalid magic/CRC
(including erased `0xFF` flash) reads back as "unpaired". Board-side flash I/O
lives in each firmware crate's `pairing` module (`esp-storage` +
`esp-bootloader-esp-idf`).

### Re-pairing

Two triggers clear the stored MAC and restart the handshake:

- **BOOT button (GPIO9)** held low during reset.
- **`HostToBoard::Repair`** over USB; wrap it in `ForPeer` to re-pair the remote
  board through the gateway.

## USB-host tunnel (`common_host_proto`)

The board connected to a PC over USB acts as a **gateway**; the other board is
the **remote**. The PC sees both boards through one USB connection.

- Host → remote: the PC sends `HostToBoard::ForPeer(payload)`, where `payload` is
  a raw (non-COBS) postcard-encoded `HostToBoard`. The gateway forwards the bytes
  verbatim as a `TunnelCmd`; the remote decodes and executes it exactly as if it
  had arrived on the remote's own USB.
- Remote → host: the remote streams `TunnelEvt` frames carrying raw
  `BoardToHost` telemetry; the gateway wraps each in
  `BoardToHost::FromPeer { source, payload }` and writes it to USB, tagged with
  the producing board.
- Telemetry streaming is **off by default** and enabled per-board with
  `HostToBoard::EnableRemoteTelemetry { on }` (typically sent `ForPeer`-wrapped to
  the remote). This saves airtime and controller battery in normal operation.

COBS framing is used **only** on the USB byte stream. The ESP-NOW tunnel carries
the raw payload, because the frame envelope plus ESP-NOW itself already delimit
frames.

## Current implementation status

- Frame envelope, pairing state machine, watchdog, and record serialization are
  implemented and host-tested in `common_comms`.
- `EspNowLink` (bidirectional) is wired into both boards via the board-specific
  `Esp32C6EspNow` transport adapter, including peer management.
- Both `controller` and `vehicle` run the gateway/remote routing and flash-backed
  pairing; `host_tool` exposes `peer <cmd>`, `remote_tele on|off`, and `repair`.
