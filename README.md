# Roller door controller

Capture a 433 MHz remote on a CC1101, store the last frame, and replay it from an ESP32-C3 Super Mini when you press **BOOT**.

## Hardware

| ESP32-C3 | CC1101 |
|----------|--------|
| GPIO 6   | SCK    |
| GPIO 7   | MISO   |
| GPIO 21  | MOSI   |
| GPIO 20  | CSN    |
| GPIO 10  | GDO0   |
| 3.3V     | VCC    |
| GND      | GND    |

Both boards run at **3.3V**. Do not power the CC1101 from 5V.

Leave **GPIO 2** and **GPIO 8** unconnected (strapping pins). **GPIO 9** is the onboard **BOOT** button; a short press transmits the stored code.

## Build and flash

Requires [PlatformIO](https://platformio.org/).

```bash
pio run -e esp32c3 -t upload
pio device monitor
```

Serial monitor: **115200** baud. If the monitor stays blank after upload, the project sets `monitor_dtr = 0` and `monitor_rts = 0` so USB reset does not hold the chip in reset.

## Usage

1. Power the board and open the serial monitor.
2. **Hold** the real remote button near the CC1101 until lines print (decoded bits plus `Press BOOT to send`).
3. Press **BOOT** once to send **one burst** (eight on-air repeats with the captured timings and inter-frame gap).
4. Capture again anytime; the newest press replaces the stored frame.

While a signal is being captured, BOOT send is ignored. After transmit there is a short quiet window so the radio does not treat its own echo as a new capture.

On send, serial shows either `Replay N p  guard XXXX` (full waveform replay) or `Rebuild (no waveform)` (synthetic PWM fallback).

## Rolling codes

Many garage and roller remotes use **rolling / hopping codes** (e.g. KeeLoq-style packets). One capture is **one** code from **one** press. A successful replay may open the door **at most once** for that capture. The next press on the real remote uses a new code.

For a reliable test, capture when the door receiver cannot hear the real remote, then send once with BOOT.

## Project layout

| File | Role |
|------|------|
| `src/main.cpp` | CC1101 setup, capture, decode, BOOT transmit |
| `src/replay_frame.h` | Assembles and stores raw pulse waveform for replay |
| `platformio.ini` | ESP32-C3 target |

## License

Private use. Ensure cloning or replaying radio signals complies with local law and your property access rules.
