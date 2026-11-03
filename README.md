# fitcycle-bridge

An ESP32-C6 bridge that lets a Garmin watch use a **Tunturi FitCycle 50i** exercise bike as a smart trainer, with a small status screen that tracks your total energy output.

```
FitCycle 50i  <── BLE (unencrypted) ──  ESP32-C6  ── BLE (bonded) ──>  Garmin Fenix 8
                                           │
                        1.47" LCD: watts, rpm, level, kJ
                        Wi-Fi setup page (on demand): pairing, live stats, settings
```

## Why a bridge?

The FitCycle 50i speaks standard Bluetooth FTMS (Fitness Machine Service), so in theory a Garmin should pair with it directly. In practice:

1. **The bike refuses BLE pairing.** It rejects every pairing request (legacy and LE Secure Connections). Garmin watches insist on bonding with a trainer, so pairing hangs forever at "connecting".
2. **The bike's ERG and simulation modes are unusable.** Both behave as a jittery power target that constantly hunts between resistance levels (audible motor noise). Once either mode has been used, the bike ignores *Set Target Resistance*, *Reset* and *Stop* until the BLE link drops.

The bridge connects to the bike unencrypted, offers the watch a normal bondable FTMS trainer, and implements ERG and simulation itself on top of plain resistance levels, the one mode the bike does well.

## Features

- **FTMS trainer for the watch:** speed, cadence, distance, resistance, power, elapsed time. Data is re-encoded to fit 20-byte notifications, because the Fenix doesn't raise the MTU.
- **Cycling Power service** as a fallback, if you'd rather pair it as a power meter. Crank revolutions are synthesised from cadence.
- **Simulation (free ride, courses):** grade maps to a fixed resistance level, so it feels like a road and the motor stays quiet on a steady grade.
- **ERG (target power):** the bridge computes the level from a fitted power model and cadence. Hysteresis and persistence keep level changes to a minimum.
- **Set resistance:** passed straight through.
- **Self-healing:** if the bike drops or gets stuck in its own ERG mode, the bridge reconnects and restores the level. Distance and time keep counting on the watch.
- **Status screen:** connection lights, mode, watts as seven-segment digits, rpm, level, plus session and lifetime energy (kJ). The lifetime total persists in flash.
- **Setup page over Wi-Fi:** pair the bike from your phone, watch live stats and tune the ride feel. The hotspot only runs while needed, so it doesn't add heat or compete with Bluetooth during rides.
- **Pairs with one bike:** after pairing, the bridge only connects to that bike's address, so it won't grab a neighbour's or a gym's FTMS bike.

## Hardware

- [Waveshare ESP32-C6-LCD-1.47](https://www.waveshare.com/wiki/ESP32-C6-LCD-1.47): ESP32-C6, 1.47" ST7789 172×320 LCD. No wiring needed; just power it over USB near the bike.
- Tunturi FitCycle 50i (reports as `FitCycle 50i-xxx`, made by ChangYow, model `SM5081E BLE`, firmware `21.00.21.00`). Other FTMS bikes from the same OEM may work too.
- Tested with a Garmin Fenix 8.

## Build and flash

Requires [PlatformIO](https://platformio.org/) (`pip install platformio`). The ESP32-C6 needs Arduino core 3.x, so the project uses the [pioarduino](https://github.com/pioarduino/platform-espressif32) platform. BLE, Wi-Fi and the web server need more than the default app slot, so the bridge uses the `min_spiffs` partition table (1.9 MB app).

```sh
pio run -e bridge -t upload      # the bridge (default)
pio device monitor               # serial log, 115200 baud
```

## First-time setup: pair the bike

On first start (no bike paired yet) the bridge opens its setup hotspot and shows a QR code.

1. **Scan the QR code** with your phone camera to join the Wi-Fi `FitCycle-Bridge-XXXX`, or join it by hand with the password shown on the screen. The password is generated per device, so neighbours can't change your pairing.
2. The setup page opens by itself (captive portal). If it doesn't, browse to `http://192.168.4.1`.
3. **Wake the bike console** by pedalling a few strokes, and close the Tunturi app (the bike accepts only one connection).
4. Tap **Pair** next to your bike under *Nearby bikes*. The **BIKE** light on the screen turns green.

The hotspot closes 3 minutes after the page was last used. **Hold BOOT for 2 seconds** to open or close it at any time.

## Pairing with the watch

1. Make sure the **BIKE** light is green (wake the console if it isn't).
2. On the watch: **Sensors & Accessories → Add New → Search All**, and pick the trainer ("Tunturi Bridge", shown as *Indoor Trainer*).
3. Start an **Indoor Bike** activity. The **WATCH** light turns green and the mode shows `SIM`, `ERG` or `RES`.

## Setup page

| Section | What it does |
|---|---|
| Live | watts, cadence, level, speed, distance, session and lifetime energy |
| Bike / Nearby bikes | pair a bike, or **Forget bike** to go back to first-time setup |
| Watch | connection state, **Forget watch pairing** (then pair the watch again) |
| Settings | saved in flash, see below; **Reset total energy** |

| Setting | Default | Meaning |
|---|---|---|
| Flat road level | 5 | resistance level on a flat road in simulation |
| Levels per 1 % grade | 2 | extra levels per percent of grade |
| ERG: change level when off by | 1.0 levels | how far off the ideal level must be before changing... |
| ERG: for at least | 2.0 s | ...and for how long |
| Screen brightness | 60 % | full brightness mostly turns into heat in the board's regulator |

## Protocol notes

What the FitCycle 50i exposes, from the sniffer:

| Service | Characteristics |
|---|---|
| `0x1826` FTMS | Feature `0x2ACC`, Indoor Bike Data `0x2AD2` (notify, 2 Hz, 30 bytes), Training Status `0x2AD3`, Resistance Range `0x2AD6` (1.0–32.0, step 1), Power Range `0x2AD8` (0–999 W), Control Point `0x2AD9`, Machine Status `0x2ADA` |
| `0xFFF0` vendor | `0xFFF1` notify, `0xFFF2` write (silent during rides) |
| `02f00000-…-ff00` vendor | `ff00`–`ff03` (`ff02` reads `ntf_enable`) |
| `0x180A` Device Info | `ChangYow` / `SM5081E BLE` / `21.00.21.00` |

Quirks:

- **Pairing** fails within ~60 ms with SMP reason `0x08` (unspecified).
- **MTU:** the bike starts its own MTU exchange right after connecting. If the central also requests one, the two collide and the connection fails, so the bridge skips its own request.
- **ERG lock-in:** after `0x05` (target power) or `0x11` (simulation), the bike stays in ERG until disconnected. `0x01` reset, `0x07` start, `0x08` stop and `0x05` with 0 W don't release it. `0x08 01` (stop) also freezes the reported data.
- **Set Target Resistance** (`0x04`) accepts both the spec's 1-byte and Garmin's 2-byte parameter. The bike reports the resulting level (1–32) in Indoor Bike Data.
- **Power model:** the bike's reported power fits `P = (19.5 + 10 × level) × (rpm / 70)^1.72` within about 5 W across all 32 levels.
- **Hand-grip HR** reads about 80 bpm even when nobody holds the grips, so the bridge doesn't forward it.

## Tools

- `src/sniffer/`: BLE explorer firmware (`pio run -e sniffer -t upload`). It scans, connects, dumps the GATT table, subscribes to everything and decodes FTMS data. Serial commands include `p` (try pairing), `q` (request control) and `w <hex>` (write the FTMS control point).
- `tools/capture.py`: logs serial output to `logs/` and can send timed commands, e.g. `python tools/capture.py --seconds 60 --send "@15 q" --send "@17 w 04 64 00"`.

## License

[MIT](LICENSE). An independent hobby project, not affiliated with or endorsed by Tunturi or Garmin. Use at your own risk.
