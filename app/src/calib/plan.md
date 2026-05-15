# Calibration Sequencer — Design Plan

The encoder calibration is **implemented in the motor controller firmware** (`calibration.c`).
This document describes the Rust app's role: triggering calibration over CAN and displaying progress.

---

## Encoder Background

The encoder is a **WaveSculptor22 absolute magnetic encoder** — 14-bit, 16384 counts/rev.
There is no index pulse. The calibration corrects two error sources:

| Error source | Correction |
|---|---|
| Mounting offset (fixed angular gap between encoder zero and motor electrical zero) | `encoder_offset` — a single integer subtracted from every reading |
| Non-linearity (magnet eccentricity, pole asymmetry) | `encoder_lut[64]` — 64-bin lookup table of additive corrections |

Both are persisted to STM32 flash at `0x0807F800` and survive power cycles.

---

## Firmware Procedures

### `calibrate_offset(Ud, steps, delay)`
- Drive rotor open-loop through N equally-spaced electrical angles
- At each step: `offset[i] = raw_position - commanded_position`
- Result: `mean(wrap(offset[i]))` → stored as `encoder_offset`
- Typical params: Ud = 1.6 V, steps = 256, delay = 500 ms

### `calibrate(Ud, Vbus)`
- Full mechanical sweep: 1024 steps (= LUT_SIZE × POLE_PAIRS = 64 × 16)
- Computes global offset + residuals per step
- Re-bins residuals into 64 LUT entries (mean within each 256-count bin)
- Saves offset + LUT to flash
- Duration: ~4.3 minutes at 250 ms/step

### `calibrate_clear()`
- Zeros offset and LUT, saves to flash

---

## CAN Interface (not yet implemented on firmware side)

The firmware currently only runs calibration from `main.c` over UART debug.
The following CAN interface is planned to trigger it remotely:

| Frame | Direction | Payload |
|---|---|---|
| `base_mc + 0x10` | GUI → MC | `[cmd: u8, Ud: f32, steps: u16, delay: u16]` — start offset calib |
| `base_mc + 0x11` | GUI → MC | `[cmd: u8]` — start full LUT calib |
| `base_mc + 0x12` | GUI → MC | `[cmd: u8]` — clear calibration |
| `base_mc + 0x18` | MC → GUI | `[step: u16, total: u16, offset: i16, pad: u16]` — progress |
| `base_mc + 0x19` | MC → GUI | `[offset: i16, lut_valid: u8, crc: u32, pad: u8]` — result |

These IDs are outside the standard WS22 range and are custom extensions.

---

## App Sequencer States

```
Idle
  │ user clicks Start
  ▼
OffsetCalib   == sends calibrate_offset command ==► firmware sweeps
  │ receives progress frames
  ▼
LutCalib      == sends calibrate_lut command ==► firmware sweeps (~4 min)
  │ receives progress frames
  ▼
Complete      == displays result: offset counts, LUT valid, CRC
  │
  └== or Aborted / Failed at any point
```

For **guided mode**, the sequencer pauses after `OffsetCalib` and waits for user
confirmation before starting `LutCalib`.

---

## Implementation Notes

- The app sequencer is a pure state machine — no threads. It processes incoming
  CAN frames (from `GuiSnapshot`) and emits outgoing `Command::SendFrame` commands.
- Progress is reported as a `(step, total)` pair → compute `pct = step / total`.
- The LUT CRC should be verified client-side against the raw LUT data before
  reporting the calibration as successful.
- Add `Command::StartCalib`, `Command::AbortCalib` back to `shared.rs` when
  implementing. The `Event::CalibProgress` and `Event::CalibResult` events are
  already designed for this — they just need re-adding.

---

## Known Firmware Bugs to Fix Before Wiring Up

1. `motor_interface_save_calibration()` — `EncoderCalData cal` is read before init (UB). Add `= {0}`.
2. LUT index uses `raw >> 5` (expecting 512 entries) but `LUT_SIZE = 64` (needs `raw >> 8`).
3. No CRC computation on flash write — result verification is blind.
