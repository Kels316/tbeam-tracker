# Buoy Tracker — Build & Flash Guide

Custom Meshtastic firmware for a GPS buoy tracker. A tracker node broadcasts position every 30 seconds on a private encrypted channel. The T-Deck receiver displays live position, bearing, speed, and range. An optional drone-mounted repeater extends LoRa range between tracker and receiver.

Three node types are supported:

| Node | Board | Role |
|---|---|---|
| **T-Beam tracker** | LilyGo T-Beam (ESP32) | Transmits GPS position |
| **Heltec V4 tracker** | Heltec WiFi LoRa 32 V4 (ESP32-S3) | Transmits GPS position |
| **Heltec V4 repeater** | Heltec WiFi LoRa 32 V4 (ESP32-S3) | Drone-mounted relay — extends range |

---

## Repo structure

```
buoy-tracker/
├── nodes/
│   ├── tbeam/                          ← T-Beam tracker node
│   │   ├── tracker_channel.h           ← PSK + channel + GPS pin config
│   │   └── src/modules/               ← TrackerModule, StrobeModule
│   └── heltec-v4/                      ← Heltec V4 nodes (tracker + repeater)
│       ├── tracker_channel.h           ← Tracker: PSK, channel, GPS pins, BT
│       ├── repeater_channel.h          ← Repeater: PSK, channel, REPEATER role
│       └── src/modules/               ← TrackerModule, StrobeModule,
│                                          RepeaterDisplayModule
├── receiver/tdeck/src/modules/        ← TrackerDisplayModule (T-Deck UI pages)
├── docs/                              ← wiring diagrams, patch reference files
└── tools/                             ← download_tiles.py (offline map prep)
```

---

## Hardware

### T-Beam tracker node

| Item | Detail |
|---|---|
| Board | LilyGo T-Beam v1.1 (ESP32) |
| GPS | External M8Q-5883 module — connected to free UART pins |
| Strobe | 5V white LED via MOSFET on GPIO 13 |
| LoRa region | ANZ |

#### T-Beam — GPS wiring (M8Q-5883 module)

| M8Q pin | T-Beam GPIO | Direction | Purpose |
|---|---|---|---|
| TX | GPIO 36 | GPS → Board | Serial data into T-Beam |
| RX | GPIO 25 | Board → GPS | Serial data from T-Beam |
| VCC | 3.3V | — | Power |
| GND | GND | — | Ground |

#### T-Beam — recovery strobe circuit (GPIO 13)

```
T-Beam 5V ── LED(+) ── LED(–) ──[56Ω]── 2N7000 Drain
2N7000 Source ── GND
GPIO 13 ──[100Ω]── 2N7000 Gate
2N7000 Gate ──[10kΩ]── GND
```

The 10kΩ gate pulldown prevents the strobe firing at boot when the GPIO floats.

---

### Heltec WiFi LoRa 32 V4 — tracker node

| Item | Detail |
|---|---|
| Board | Heltec WiFi LoRa 32 V4 (ESP32-S3R2, SX1262) |
| GPS | L76K via dedicated SH1.25-8P GNSS connector (plug-and-play) |
| OLED | 0.96" 128×64 SSD1315 (SSD1306-compatible) — shows node info |
| LED | GPIO 35 built-in — Meshtastic heartbeat blink |
| LoRa region | ANZ |

#### Heltec V4 — GPS wiring (SH1.25-8P GNSS connector)

The board has a dedicated 8-pin GNSS connector. Connect the L76K directly to it using the matching cable.

| Connector pin | Signal name | GPIO | Direction | Purpose |
|---|---|---|---|---|
| 1 | GND | GND | — | Ground |
| 2 | 3V3 | 3.3V | — | Power |
| 5 | GNSS_RST | GPIO 42 | Board → GPS | Reset |
| 6 | GNSS_PPS | GPIO 41 | GPS → Board | 1 PPS (unused) |
| 7 | GNSS_TX | GPIO 39 | Board → GPS | Serial commands to GPS |
| 8 | GNSS_RX | GPIO 38 | GPS → Board | Serial position data |

> **Pin naming note:** Meshtastic config uses `rx_gpio` / `tx_gpio` from the CPU's perspective.
> `rx_gpio = 39` (CPU receives from GPS on this pin), `tx_gpio = 38` (CPU transmits to GPS on this pin).

---

### Heltec WiFi LoRa 32 V4 — repeater node

| Item | Detail |
|---|---|
| Board | Heltec WiFi LoRa 32 V4 (ESP32-S3R2, SX1262) |
| GPS | Not required — GPS disabled in firmware |
| OLED | Shows REPEATER status page (battery, uptime, channel util, node ID) |
| LED | GPIO 35 built-in — Meshtastic heartbeat blink |
| Role | `REPEATER` — dumb relay, forwards all packets on the TRACKER channel |
| Intended use | Mount on a drone to extend LoRa range between tracker and T-Deck |
| LoRa region | ANZ |

The repeater uses the same PSK as the tracker and T-Deck on channel slot 1 (`TRACKER`). No custom modules run — it is a transparent relay.

---

## Firmware setup

There are **two separate firmware directories** on disk. Do not mix them up.

| Directory | Purpose | Boards |
|---|---|---|
| `~/Documents/firmware-2.7.15` | Patched Meshtastic 2.7.15 | T-Beam tracker, Heltec V4 tracker, Heltec V4 repeater |
| `~/Documents/firmware` | Patched Meshtastic 2.7.15 (separate clone) | T-Deck receiver only |

Both are patched copies of the same upstream release. They exist separately because the T-Deck build uses `env:t-deck-tft` (which pulls in LVGL and TFT drivers) and its build flags conflict with the tracker/repeater envs. Mixing them in the same `platformio.ini` causes build failures.

### Prerequisites

1. Clone Meshtastic firmware 2.7.15 **twice** — once for trackers, once for T-Deck:
   ```bash
   cd ~/Documents

   # Tracker / repeater builds
   git clone --branch 2.7.15 https://github.com/meshtastic/firmware.git firmware-2.7.15
   cd firmware-2.7.15 && git submodule update --init --recursive && cd ..

   # T-Deck build
   git clone --branch 2.7.15 https://github.com/meshtastic/firmware.git firmware
   cd firmware && git submodule update --init --recursive && cd ..
   ```

2. Clone this repo alongside both:
   ```bash
   cd ~/Documents
   git clone https://github.com/Kels316/tbeam-tracker buoy-tracker
   ```

3. Create the symlink that lets PlatformIO find the Heltec V4 custom modules:
   ```bash
   ln -s ~/Documents/buoy-tracker/nodes/heltec-v4/src \
         ~/Documents/firmware-2.7.15/src/heltec_v4_tracker
   ```

### Patch `firmware-2.7.15/platformio.ini` (trackers + repeater)

Add the following sections to `~/Documents/firmware-2.7.15/platformio.ini`:

```ini
[heltec_v4_base]
extends = esp32s3_base
board = heltec_v4
board_check = true
board_build.partitions = default_16MB.csv
build_flags =
    ${esp32s3_base.build_flags}
    -D HELTEC_V4
    -I variants/esp32s3/heltec_v4
lib_deps =
    ${esp32s3_base.lib_deps}

[env:heltec-v4-tracker]
extends = heltec_v4_base
upload_speed = 921600
build_type = release
build_src_filter =
    ${esp32s3_base.build_src_filter}
    -<modules/TrackerModule.cpp>
    -<modules/StrobeModule.cpp>
    +<heltec_v4_tracker/**>
build_flags =
    ${heltec_v4_base.build_flags}
    -D HELTEC_V4_OLED
    -D USE_SSD1306
    -D HAS_SCREEN=0
    -D LED_PIN=35
    -D RESET_OLED=21
    -D I2C_SDA=17
    -D I2C_SCL=18
    -D I2C_SDA1=4
    -D I2C_SCL1=3
    -I../buoy-tracker/nodes/heltec-v4
    -D USE_TRACKER_MODULE=1
    -D GPS_POWER_TOGGLE=0
    -D POSITION_BROADCAST_SECS=3600
    -D MESHTASTIC_EXCLUDE_WEBSERVER=1

[env:heltec-v4-repeater]
extends = heltec_v4_base
upload_speed = 921600
build_type = release
build_src_filter =
    ${esp32s3_base.build_src_filter}
    -<modules/TrackerModule.cpp>
    -<modules/StrobeModule.cpp>
    -<heltec_v4_tracker/**>
    +<heltec_v4_tracker/modules/RepeaterDisplayModule.cpp>
build_flags =
    ${heltec_v4_base.build_flags}
    -D HELTEC_V4_OLED
    -D USE_SSD1306
    -D HAS_SCREEN=1
    -D LED_PIN=35
    -D RESET_OLED=21
    -D I2C_SDA=17
    -D I2C_SCL=18
    -I../buoy-tracker/nodes/heltec-v4
    -D USE_REPEATER_MODULE=1
    -D MESHTASTIC_EXCLUDE_WEBSERVER=1
```

### Patch `firmware-2.7.15/src/main.cpp` (trackers + repeater)

Add near the top of `~/Documents/firmware-2.7.15/src/main.cpp` (after the standard includes):

```cpp
#ifdef USE_TRACKER_MODULE
#  ifdef HELTEC_V4
#    include "src/modules/TrackerModule.h"
#    include "src/modules/StrobeModule.h"
#  else
#    include "modules/TrackerModule.h"
#    include "modules/StrobeModule.h"
#  endif
#include "tracker_channel.h"
#endif
#ifdef USE_REPEATER_MODULE
#include "repeater_channel.h"
#include "src/modules/RepeaterDisplayModule.h"
#endif
```

Inside `void setup()`, after `service = new MeshService()` and before `service->init()`:

```cpp
#ifdef USE_TRACKER_MODULE
    setupTrackerChannel();
#endif
#ifdef USE_REPEATER_MODULE
    setupRepeaterChannel();
#endif
    service->init();
```

After `setupModules()`:

```cpp
#ifdef USE_TRACKER_MODULE
    trackerModule = new TrackerModule();
    strobeModule  = new StrobeModule();
#endif
#ifdef USE_REPEATER_MODULE
    repeaterDisplayModule = new RepeaterDisplayModule();
#endif
```

### Patch `firmware/platformio.ini` (T-Deck only)

Add to `~/Documents/firmware/platformio.ini`:

```ini
[env:t-deck-tracker-display]
extends = env:t-deck-tft
build_src_filter = ${env:t-deck-tft.build_src_filter} -<modules/TrackerModule.cpp> +<../../buoy-tracker/receiver/tdeck/src/modules/TrackerDisplayModule.cpp>
build_flags =
    ${env:t-deck-tft.build_flags}
    -DUSE_TRACKER_DISPLAY_MODULE=1
    -I${PROJECT_DIR}/../buoy-tracker/receiver/tdeck/src
```

### Patch `firmware/src/graphics/tftSetup.cpp` (T-Deck only)

After the existing `#include` block near the top of the file, add:

```cpp
#ifdef USE_TRACKER_DISPLAY_MODULE
extern void trackerRunSetup();
#endif
```

At the very start of `tft_task_handler()`, before the `while (true)` loop:

```cpp
#ifdef USE_TRACKER_DISPLAY_MODULE
    trackerRunSetup();
#endif
```

The function should look like this after patching:

```cpp
void tft_task_handler(void *param = nullptr)
{
#ifdef USE_TRACKER_DISPLAY_MODULE
    trackerRunSetup();
#endif
    while (true) {
        spiLock->lock();
        deviceScreen->task_handler();
        spiLock->unlock();
        deviceScreen->sleep();
    }
}
```

### Patch `firmware/src/main.cpp` (T-Deck only)

Add near the top of `~/Documents/firmware/src/main.cpp` (after the standard includes, e.g. after the `NodeDB.h` include):

```cpp
#ifdef USE_TRACKER_DISPLAY_MODULE
#include "modules/TrackerDisplayModule.h"
#endif
```

Inside `void setup()`, after `nodeDB = new NodeDB` and before the `#if HAS_TFT` block:

```cpp
#ifdef USE_TRACKER_DISPLAY_MODULE
    // Force COLOR display mode so device-ui doesn't reboot to fix a mismatch.
    config.display.displaymode = meshtastic_Config_DisplayConfig_DisplayMode_COLOR;
#endif
```

Inside `void setup()`, after `setupModules()`:

```cpp
#ifdef USE_TRACKER_DISPLAY_MODULE
    trackerRadarModule = new TrackerRadarModule();
    TrackerScreens::init();
#endif
```

---

## Building and flashing

PlatformIO CLI is at `~/.local/bin/pio`. Replace the port with your actual device from `ls /dev/cu.*`.

### T-Beam tracker

```bash
cd ~/Documents/firmware-2.7.15
~/.local/bin/pio run -e tbeam-tracker --target upload --upload-port /dev/cu.usbserial-XXXXXXXX
```

### Heltec V4 tracker

```bash
cd ~/Documents/firmware-2.7.15
~/.local/bin/pio run -e heltec-v4-tracker --target upload --upload-port /dev/cu.usbmodemXXXXXX
```

### Heltec V4 repeater

```bash
cd ~/Documents/firmware-2.7.15
~/.local/bin/pio run -e heltec-v4-repeater --target upload --upload-port /dev/cu.usbmodemXXXXXX
```

### T-Deck (receiver)

**Use `firmware/`, not `firmware-2.7.15`** — the T-Deck env only exists there.

```bash
cd ~/Documents/firmware
~/.local/bin/pio run -e t-deck-tracker-display --target upload --upload-port /dev/cu.usbmodemXXXXXX
```

---

## Channel configuration

All nodes use channel **slot 1** named `TRACKER` with AES-256 encryption. Channel slot 0 must remain the default primary.

The PSK is baked into the firmware and applied on first boot. If you need to set it manually via CLI (e.g. for the T-Deck):

```bash
meshtastic --port /dev/cu.<port> \
  --ch-index 1 \
  --ch-set name TRACKER \
  --ch-set psk 0x4a3f8c21d755b209e17a44fc308e6bd3921c5fa87703e64dbb291058c49d6ef1
```

> **Important:** Use the hex format (`0x...`). Base64 is silently rejected on these devices.
> Do **not** use `--ch-set-enabled true` — it breaks channel config.

PSK bytes (for reference / changing the key):
```
0x4a,0x3f,0x8c,0x21,0xd7,0x55,0xb2,0x09,0xe1,0x7a,0x44,0xfc,0x30,0x8e,0x6b,0xd3,
0x92,0x1c,0x5f,0xa8,0x77,0x03,0xe6,0x4d,0xbb,0x29,0x10,0x58,0xc4,0x9d,0x6e,0xf1
```

To change the PSK: generate a new 32-byte key (`openssl rand -hex 32`), update the `TRACKER_PSK` / `REPEATER_PSK` arrays in all three `*_channel.h` files, then rebuild and reflash all nodes.

---

## Data screen — speed and COG filtering

Speed (knots) and course over ground (COG) are calculated using an anchor model:

- The first position packet received is stored as the anchor.
- If displacement from the anchor is less than **15 metres**, speed and COG report as `0 kt / 0°` to filter GPS jitter.
- If displacement exceeds 15 metres, speed and COG are calculated from anchor to current position — the net set and drift over the deployment period, which is the relevant quantity for SAR operations.

`0 kt / 0°` = stationary (within GPS noise). `--- / ---` = no data yet.

---

## Repeater OLED display

The repeater's 0.96" OLED shows a single status page:

| Field | Description |
|---|---|
| Header | "REPEATER" title + battery icon (auto-drawn by firmware) |
| Batt | Battery % and voltage, or "USB" if powered via USB |
| Up | Uptime (hours, minutes, seconds) |
| Ch / TX | Channel utilization % and TX air utilization % |
| ID | Node ID in short hex (`!xxxxxxxx`) |

The GPIO 35 LED blinks at the standard Meshtastic heartbeat rate as a secondary alive indicator.

---

## Updating

```bash
cd ~/Documents/buoy-tracker && git pull
```

Then rebuild and reflash. No file copying needed — `firmware-2.7.15` pulls from `buoy-tracker/` via the symlink, and `firmware` pulls from `buoy-tracker/receiver/tdeck/` via the relative path in `platformio.ini`.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Build hangs at dependency scan | Wait 15–20 min on first run — normal for full dep scan |
| SCons `.dblite` crash | `rm -f .pio/build/<env>/.sconsign314.dblite` and retry |
| Port busy | `lsof /dev/cu.<port>` to find holder, then `kill <pid>` |
| Upload fails | Hold BOOT button on device while upload starts |
| No GPS lock | Normal indoors — take outside, cold fix takes 2–5 min |
| T-Deck shows zero position | Confirm TRACKER channel is slot 1 on both nodes with matching PSK |
| Packets not received | Confirm both nodes use identical PSK (hex format, not base64) |
| Channel 1 not appearing after flash | Set via CLI using hex PSK format above |
| Strobe always on at boot | Check 10kΩ gate pulldown on MOSFET circuit |
| Heltec V4 strobe not working | GPIO 13 is LORA_BUSY — do not use it |
| Heltec V4 GPS not locking | Check GNSS connector orientation; `rx_gpio=39`, `tx_gpio=38` |
| Battery shows `--` on T-Deck | Normal for first 5 min — telemetry interval is 300 s |
| `Unknown environment names 't-deck-tracker-display'` | You're in `firmware-2.7.15` — that env lives in `firmware/`. Run `cd ~/Documents/firmware` first. |
| `lv_conf.h not found` in tracker/repeater build | TrackerDisplayModule accidentally included — only belongs in `firmware/`, not `firmware-2.7.15` |
| Repeater probing for GPS on first boot | Expected — GPS disable saves on that boot, gone after one reboot |
