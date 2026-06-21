# Shared ESP-NOW Control Protocol

This document defines the protocol and link supervision shared by controller and vehicle.

## Packet: ControlPacket

Length: 7 bytes (`CONTROL_PACKET_LEN`)

Byte layout (little-endian):

1. `sequence_lo`
2. `sequence_hi`
3. `x`
4. `y`
5. `buttons`
6. `reserved` (must be `0`)
7. `checksum`

Checksum rule:

`checksum = sequence_lo ^ sequence_hi ^ x ^ y ^ buttons ^ reserved`

## Buttons bitfield

- bit 0: JOY
- bit 1: C
- bit 2: A
- bit 3: B
- bit 4: D

## Keepalive and timeout

- Transmit interval target: `100 ms` (`CONTROL_TX_INTERVAL_MS`)
- Vehicle timeout threshold: `500 ms` (`LINK_TIMEOUT_MS`)

The command stream itself is the keepalive. No separate heartbeat frame is required.

Vehicle must immediately enter fail-safe stop if no valid fresh packet is received for more than `500 ms`.

## Sequence freshness

Vehicle accepts a packet as newer when `delta = candidate.wrapping_sub(last)` is in `(0, 0x8000)`.

This handles wrap-around (`65535 -> 0`) while rejecting duplicate and stale frames.

## Current implementation status

- Shared protocol and watchdog are implemented in `common_comms`.
- Controller packet generation now uses `common_comms::protocol::ControlPacket`.
- Vehicle watchdog scaffolding is active and already transitions to timeout state.
- ESP-NOW hardware transport hookup is prepared via `common_comms::espnow` traits/wrappers and still needs board-specific driver binding.
