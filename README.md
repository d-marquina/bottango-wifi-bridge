# Bottango WiFi Bridge

WiFi HMI for [Bottango](https://www.bottango.com/)-based animatronics. A classic
**ESP32 (WROOM-32 DevKit)** hosts a small web app and drives a **Raspberry Pi
Pico** running the [BottangoMPDriver](https://github.com/d-marquina/BottangoMPDriver)
over UART — speaking the exact same serial protocol Bottango uses, so the
Pico driver requires **no modification** (only an opt-in flag in its `main.py`).

Any phone, tablet or laptop can control the animatronic from its browser.
No app to install, no internet required, nothing to configure per device.

```
Browser ──WiFi/WebSocket──► ESP32 ──UART2──► RPi Pico ──► servos / steppers
         (192.168.4.1)      ├─ Web HMI (LittleFS)
                            ├─ project.json (persistent)
                            └─ Playback engine (Bottango host role)
```

## Features

- **Self-contained Access Point** — the ESP32 creates its own WiFi network;
  browse to `http://192.168.4.1`. Works anywhere, no router needed.
- **Multi-file animation library** — every `AnimationCommands.json`
  exported by Bottango is stored as its own file under `/anims/` in the
  ESP32 flash (LittleFS), persisting across reboots. Each file keeps its
  own effector setup, so different animations may use different servo
  configurations. Re-uploading a file with the same name replaces it.
- **Automatic effector re-registration** — when the selected animation
  belongs to a file whose setup differs from the one currently registered
  on the Pico, the bridge re-runs handshake + setup transparently.
- **Animation list with computed durations and source file**, play/stop
  per animation.
- **Rename / delete animations** from the web page (changes are persisted
  into their source file; a file whose last animation is deleted is
  removed).
- **Faithful Bottango host protocol** — handshake (`hRQ`/`btngoHSK`),
  effector registration, `tSYN` time sync and curve delivery (`sSY`/`sC`)
  with per-command OK flow control. Stop uses `xC` (never `STOP`, which
  would reset the Pico).
- **Live log panel** mirroring all serial traffic with the Pico.
- **Bilingual UI (Spanish / English)** — toggle with the EN/ES button in the
  header; the choice is remembered per device (localStorage).
- **Legacy-browser friendly** — the HMI runs on Chrome for Android 5–7 era
  tablets (no modern-JS-only syntax, XHR uploads, auto-reconnect WebSocket).

## Hardware

| ESP32 DevKit | RPi Pico | Purpose |
|---|---|---|
| GPIO17 (TX2) | GP1 (RX, physical pin 2) | commands → Pico |
| GPIO16 (RX2) | GP0 (TX, physical pin 1) | replies (OK/LOG) ← Pico |
| GND | GND | common ground (required) |

Both boards are 3.3 V logic — direct connection, no level shifter.

Power (suggested): one 6 V supply → servos directly; a small buck converter
(6 V → 5 V) feeds the Pico (VSYS, pin 39) and the ESP32 (VIN/5V pin).
Common ground everywhere. Do not feed VIN while the ESP32's own USB is
connected unless your board has a protection diode.

### Pico side

In the Pico's `main.py` (BottangoMPDriver), enable the bridge flag — it is
**`False` by default** (plain USB/Bottango behaviour):

```python
ENABLE_UART_BRIDGE = True   # duplicates the console on UART0 (GP0/GP1)
```

This uses `os.dupterm()` — the driver package itself is untouched, and USB
keeps working in parallel for debugging.

## Configuration

WiFi network name and password live at the top of [`src/main.cpp`](src/main.cpp):

```cpp
static const char *AP_SSID = "Animatronico";
static const char *AP_PASS = "bottango123";   // min. 8 characters (WPA2)
```

Change them and re-upload the firmware. Other tunables nearby: UART pins
(`UART_TX_PIN` / `UART_RX_PIN`), baud rate, and the reply timeout.

## Building & uploading

Two things must be uploaded: the **firmware** (`src/`) and the **filesystem
image** (`data/`, which contains the web page). After changing `data/`, you
must re-upload the filesystem image.

### PlatformIO CLI

```bash
pio run -t upload        # build + flash the firmware
pio run -t uploadfs      # flash the data/ folder (web page) to LittleFS
pio device monitor       # serial console (115200)
```

### VS Code (PlatformIO extension)

PlatformIO sidebar (alien icon) → **PROJECT TASKS** → `esp32dev`:

- **General → Upload** — firmware
- **Platform → Upload Filesystem Image** — web page (`data/`)
- **General → Monitor** — serial console

Close the serial monitor before uploading (it holds the COM port).

## Usage

1. Power the system; the ESP32 starts the `Animatronico` WiFi network.
2. Connect your device to it and browse to `http://192.168.4.1`.
3. Tap **⬆ Subir proyecto (JSON)** and pick an `AnimationCommands.json`
   exported by Bottango (*export → "Data as json"*). Repeat for as many
   exports as you like — each becomes a separate library file (rename the
   files on your device first if they share the same name).
4. Tap **▶** on any animation. **■ Detener / Stop** stops playback.

Tip: generate a WiFi QR code (`WIFI:T:WPA;S:Animatronico;P:bottango123;;`)
and stick it on the animatronic — scanning it joins the network directly.

## Limitations / roadmap

- `Animation Loop Commands`, idle animations and autoplay-on-boot are stored
  but not yet played (planned).
- Only the first controller in the exported JSON is used.

### Streaming playback

Animations of any length are supported. The driver keeps a circular buffer of
8 curves per effector, so instead of dumping everything up-front the bridge
streams each curve line ~1 s before its start time, and additionally holds a
line back until the buffer slot it would overwrite belongs to a curve that
has already finished. Flow control (one command per OK) is honoured
throughout.

## Related projects

- [BottangoMPDriver](https://github.com/d-marquina/BottangoMPDriver) — the
  MicroPython Bottango driver running on the Pico.

## License

Released under the **MIT License** — see [`LICENSE`](LICENSE).

Bottango is a product of [Bottango LLC](https://www.bottango.com/). This is an
independent community project, not affiliated with or endorsed by Bottango LLC.
