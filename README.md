# Meshtastic GPS Tracker — Build & Flash Guide
## Target: TTGO T-Beam (ESP32)

---

## What this firmware does

| Feature | Detail |
|---|---|
| GPS broadcast | Every **30 seconds** via LoRa |
| Compass heading | QMC5883L via I2C, packed into `ground_track` field |
| Channel | Private, PSK baked into firmware |
| Display | Disabled (no OLED driver loaded) |
| Bluetooth | Disabled (saves ~60 KB flash) |
| Power LED | GPIO 2 — on solid when running |
| TX LED | GPIO 4 — flashes 200 ms on each packet |
| Recovery strobe | GPIO 13 — 3 blinks/30s (lock), 3 blinks/5s (no lock) |

---

## Hardware — Wiring

### Wiring diagrams

**Diagram 1 — system overview**

<details>
<summary>Click to view system overview diagram</summary>

```svg
<svg width="100%" viewBox="0 0 680 560" role="img">
  <title>T-Beam system overview wiring diagram</title>
  <desc>Overview showing connections between M8Q-5883 GPS module, T-Beam, LEDs and strobe</desc>
  <defs>
    <marker id="a" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
      <path d="M2 1L8 5L2 9" fill="none" stroke="context-stroke" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
  </defs>
  <rect x="265" y="80" width="150" height="400" rx="12" fill="#f1efe8" stroke="#5f5e5a" stroke-width="1"/>
  <text font-family="sans-serif" font-size="13" font-weight="500" x="340" y="106" text-anchor="middle" fill="#2c2c2a">TTGO T-Beam</text>
  <text font-family="sans-serif" font-size="11" x="340" y="122" text-anchor="middle" fill="#5f5e5a">ESP32 + LoRa</text>
  <text font-family="sans-serif" font-size="11" x="275" y="175" fill="#2c2c2a">GPIO 12</text>
  <text font-family="sans-serif" font-size="11" x="275" y="215" fill="#2c2c2a">GPIO 15</text>
  <text font-family="sans-serif" font-size="11" x="275" y="255" fill="#2c2c2a">GPIO 21</text>
  <text font-family="sans-serif" font-size="11" x="275" y="295" fill="#2c2c2a">GPIO 22</text>
  <text font-family="sans-serif" font-size="11" x="275" y="335" fill="#2c2c2a">3.3V</text>
  <text font-family="sans-serif" font-size="11" x="275" y="395" fill="#2c2c2a">GND</text>
  <text font-family="sans-serif" font-size="11" x="405" y="175" text-anchor="end" fill="#2c2c2a">GPIO 2</text>
  <text font-family="sans-serif" font-size="11" x="405" y="255" text-anchor="end" fill="#2c2c2a">GPIO 4</text>
  <text font-family="sans-serif" font-size="11" x="405" y="335" text-anchor="end" fill="#2c2c2a">GPIO 13</text>
  <text font-family="sans-serif" font-size="11" x="405" y="360" text-anchor="end" fill="#2c2c2a">5V</text>
  <text font-family="sans-serif" font-size="11" x="405" y="395" text-anchor="end" fill="#2c2c2a">GND</text>
  <circle cx="265" cy="175" r="5" fill="#378add"/>
  <circle cx="265" cy="215" r="5" fill="#378add"/>
  <circle cx="265" cy="255" r="5" fill="#1d9e75"/>
  <circle cx="265" cy="295" r="5" fill="#1d9e75"/>
  <circle cx="265" cy="335" r="5" fill="#e24b4a"/>
  <circle cx="265" cy="395" r="5" fill="#5f5e5a"/>
  <circle cx="415" cy="175" r="5" fill="#e24b4a"/>
  <circle cx="415" cy="255" r="5" fill="#ba7517"/>
  <circle cx="415" cy="335" r="5" fill="#ba7517"/>
  <circle cx="415" cy="360" r="5" fill="#e24b4a"/>
  <circle cx="415" cy="395" r="5" fill="#5f5e5a"/>
  <rect x="30" y="155" width="150" height="220" rx="10" fill="#e1f5ee" stroke="#0f6e56" stroke-width="1"/>
  <text font-family="sans-serif" font-size="13" font-weight="500" x="105" y="181" text-anchor="middle" fill="#085041">M8Q-5883</text>
  <text font-family="sans-serif" font-size="11" x="105" y="197" text-anchor="middle" fill="#0f6e56">GPS + compass</text>
  <circle cx="180" cy="218" r="4" fill="#e24b4a"/>
  <text font-family="sans-serif" font-size="11" x="174" y="222" text-anchor="end" fill="#2c2c2a">3.3V</text>
  <circle cx="180" cy="248" r="4" fill="#378add"/>
  <text font-family="sans-serif" font-size="11" x="174" y="252" text-anchor="end" fill="#2c2c2a">TX</text>
  <circle cx="180" cy="278" r="4" fill="#378add"/>
  <text font-family="sans-serif" font-size="11" x="174" y="282" text-anchor="end" fill="#2c2c2a">RX</text>
  <circle cx="180" cy="308" r="4" fill="#1d9e75"/>
  <text font-family="sans-serif" font-size="11" x="174" y="312" text-anchor="end" fill="#2c2c2a">SDA</text>
  <circle cx="180" cy="338" r="4" fill="#1d9e75"/>
  <text font-family="sans-serif" font-size="11" x="174" y="342" text-anchor="end" fill="#2c2c2a">SCL</text>
  <circle cx="180" cy="368" r="4" fill="#5f5e5a"/>
  <text font-family="sans-serif" font-size="11" x="174" y="372" text-anchor="end" fill="#2c2c2a">GND</text>
  <polyline fill="none" stroke="#e24b4a" stroke-width="1.5" points="180,218 193,218 193,110 248,110 248,335 265,335"/>
  <polyline fill="none" stroke="#378add" stroke-width="1.5" points="180,248 205,248 205,175 265,175"/>
  <polyline fill="none" stroke="#378add" stroke-width="1.5" stroke-dasharray="6 3" points="180,278 210,278 210,215 265,215"/>
  <polyline fill="none" stroke="#1d9e75" stroke-width="1.5" points="180,308 215,308 215,255 265,255"/>
  <polyline fill="none" stroke="#1d9e75" stroke-width="1.5" stroke-dasharray="6 3" points="180,338 220,338 220,295 265,295"/>
  <polyline fill="none" stroke="#5f5e5a" stroke-width="1.5" points="180,368 193,368 193,450 248,450 248,395 265,395"/>
  <polyline fill="none" stroke="#e24b4a" stroke-width="1.5" points="415,175 490,175"/>
  <polyline fill="none" stroke="#ba7517" stroke-width="1.5" points="415,255 490,255"/>
  <polyline fill="none" stroke="#ba7517" stroke-width="1.5" points="415,335 450,335 450,390 490,390"/>
  <polyline fill="none" stroke="#e24b4a" stroke-width="1.5" stroke-dasharray="4 3" points="415,360 455,360 455,410 490,410"/>
  <polyline fill="none" stroke="#5f5e5a" stroke-width="1.5" stroke-dasharray="4 3" points="415,395 460,395 460,430 490,430"/>
  <rect x="490" y="148" width="160" height="65" rx="10" fill="#faece7" stroke="#993c1d" stroke-width="1"/>
  <text font-family="sans-serif" font-size="13" font-weight="500" x="570" y="171" text-anchor="middle" fill="#4a1b0c">Power LED</text>
  <text font-family="sans-serif" font-size="11" x="570" y="187" text-anchor="middle" fill="#712b13">Always on — GPIO 2</text>
  <text font-family="sans-serif" font-size="11" x="570" y="203" text-anchor="middle" fill="#712b13">See circuit A</text>
  <rect x="490" y="228" width="160" height="65" rx="10" fill="#faeeda" stroke="#854f0b" stroke-width="1"/>
  <text font-family="sans-serif" font-size="13" font-weight="500" x="570" y="251" text-anchor="middle" fill="#412402">TX LED</text>
  <text font-family="sans-serif" font-size="11" x="570" y="267" text-anchor="middle" fill="#633806">Flashes on send — GPIO 4</text>
  <text font-family="sans-serif" font-size="11" x="570" y="283" text-anchor="middle" fill="#633806">See circuit A</text>
  <rect x="490" y="368" width="160" height="100" rx="10" fill="#eeedfe" stroke="#534ab7" stroke-width="1"/>
  <text font-family="sans-serif" font-size="13" font-weight="500" x="570" y="393" text-anchor="middle" fill="#26215c">Recovery strobe</text>
  <text font-family="sans-serif" font-size="11" x="570" y="411" text-anchor="middle" fill="#3c3489">GPIO 13 gate</text>
  <text font-family="sans-serif" font-size="11" x="570" y="429" text-anchor="middle" fill="#3c3489">5V LED + MOSFET</text>
  <text font-family="sans-serif" font-size="11" x="570" y="447" text-anchor="middle" fill="#3c3489">See circuit B</text>
  <rect x="30" y="500" width="620" height="50" rx="8" fill="none" stroke="#b4b2a9" stroke-width="0.5"/>
  <line x1="50" y1="516" x2="86" y2="516" stroke="#378add" stroke-width="2"/>
  <text font-family="sans-serif" font-size="11" x="92" y="520" fill="#2c2c2a">UART TX</text>
  <line x1="50" y1="536" x2="86" y2="536" stroke="#378add" stroke-width="2" stroke-dasharray="6 3"/>
  <text font-family="sans-serif" font-size="11" x="92" y="540" fill="#2c2c2a">UART RX</text>
  <line x1="185" y1="516" x2="221" y2="516" stroke="#1d9e75" stroke-width="2"/>
  <text font-family="sans-serif" font-size="11" x="227" y="520" fill="#2c2c2a">I2C SDA</text>
  <line x1="185" y1="536" x2="221" y2="536" stroke="#1d9e75" stroke-width="2" stroke-dasharray="6 3"/>
  <text font-family="sans-serif" font-size="11" x="227" y="540" fill="#2c2c2a">I2C SCL</text>
  <line x1="330" y1="516" x2="366" y2="516" stroke="#e24b4a" stroke-width="2"/>
  <text font-family="sans-serif" font-size="11" x="372" y="520" fill="#2c2c2a">Power</text>
  <line x1="330" y1="536" x2="366" y2="536" stroke="#5f5e5a" stroke-width="2"/>
  <text font-family="sans-serif" font-size="11" x="372" y="540" fill="#2c2c2a">Ground</text>
  <line x1="450" y1="516" x2="486" y2="516" stroke="#ba7517" stroke-width="2"/>
  <text font-family="sans-serif" font-size="11" x="492" y="520" fill="#2c2c2a">GPIO signal</text>
  <line x1="450" y1="536" x2="486" y2="536" stroke="#ba7517" stroke-width="2" stroke-dasharray="4 3"/>
  <text font-family="sans-serif" font-size="11" x="492" y="540" fill="#2c2c2a">GPIO shared run</text>
</svg>
```

</details>

**Circuit A — power LED and TX LED (same wiring for both)**

```
GPIO pin ──[220Ω resistor]──── LED anode  (longer leg +)
                               LED cathode (shorter leg -) ──── GND

Power LED: GPIO 2  (always on)
TX LED:    GPIO 4  (flashes 200ms per packet sent)
```

**Circuit B — recovery strobe (5V via 2N7000 MOSFET)**

```
5V ──── LED anode (+)
        LED cathode (-) ──[56Ω]──── 2N7000 Drain
                                    2N7000 Source ──── GND

GPIO 13 ──[100Ω]──┬──── 2N7000 Gate
                  │
               [10kΩ]
                  │
                 GND   (pulldown — do not skip)
```


### M8Q-5883 module connections

| Module pin | T-Beam pin | Purpose |
|---|---|---|
| TX | GPIO 12 | GPS UART RX into T-Beam |
| RX | GPIO 15 | GPS UART TX from T-Beam |
| SDA | GPIO 21 | QMC5883L I2C data |
| SCL | GPIO 22 | QMC5883L I2C clock |
| VCC | 3.3V | Power |
| GND | GND | Ground |

### Power and TX LEDs

> GPIO 21 is used for I2C SDA — power LED is on GPIO 2, not GPIO 21.

**Power LED (always on) — GPIO 2:**
```
GPIO 2 ──[220Ω]── LED anode (+)
                  LED cathode (–) ── GND
```

**TX flash LED — GPIO 4:**
```
GPIO 4 ──[220Ω]── LED anode (+)
                  LED cathode (–) ── GND
```

### Recovery strobe circuit (GPIO 13)

LED: Jaycar ZD0290 White 5mm Cree 45000mcd (Vf=3.2V, If=100mA)

```
T-Beam 5V ── LED(+) ── LED(–) ──[56Ω]── 2N7000 Drain
2N7000 Source ─────────────────────────── GND
GPIO 13 ──[100Ω]───────────────────────── 2N7000 Gate
2N7000 Gate ──[10kΩ]───────────────────── GND
```

The 10kΩ gate pulldown is important — without it the GPIO floats at boot and the strobe may fire unexpectedly.

### Magnetic declination

Open `src/modules/TrackerModule.h` and set `MAG_DECLINATION` for your deployment location.
Brisbane is pre-set to +11.5 degrees. Find your value at https://www.magnetic-declination.com

---

## Step 1 — Install prerequisites

Open Terminal on your Mac and run:

```bash
# Install Homebrew if you don't have it
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install Python and PlatformIO
brew install python
pip3 install platformio

# Install USB driver for the T-Beam's CP2102 chip
brew install --cask silicon-labs-vcp-driver
```

Reboot your Mac after installing the driver.

---

## Step 2 — Clone the Meshtastic firmware

```bash
cd ~/Documents
git clone https://github.com/meshtastic/firmware.git
cd firmware
git submodule update --init
```

This downloads the official Meshtastic source code. The `git submodule update --init` pulls in all the libraries it depends on — it can take a few minutes.

---

## Step 3 — Clone your tracker repo

```bash
git clone https://github.com/Kels316/tbeam-tracker.git ~/Documents/tbeam-tracker
```

---

## Step 4 — Copy your custom files into the firmware

```bash
cd ~/Documents/firmware

cp ~/Documents/tbeam-tracker/src/modules/TrackerModule.h   src/modules/
cp ~/Documents/tbeam-tracker/src/modules/TrackerModule.cpp src/modules/
cp ~/Documents/tbeam-tracker/src/modules/StrobeModule.h    src/modules/
cp ~/Documents/tbeam-tracker/src/modules/StrobeModule.cpp  src/modules/
cp ~/Documents/tbeam-tracker/src/tracker_channel.h         src/
cp ~/Documents/tbeam-tracker/platformio.ini                .
```

---

## Step 5 — Set your private PSK

Open `src/tracker_channel.h` in a text editor:

```bash
open -e src/tracker_channel.h
```

Generate a random key:

```bash
openssl rand -hex 32
```

Convert the output into `0x??` byte pairs and replace the `TRACKER_PSK` array in the file. Every node that receives these packets must use the same key.

---

## Step 6 — Patch main.cpp

Open the file:

```bash
open -e src/main.cpp
```

Find the block of `#include` lines near the top and add these three lines with them:

```cpp
#include "modules/TrackerModule.h"
#include "modules/StrobeModule.h"
#include "tracker_channel.h"
```

Then find `nodeDB.init();` inside `void setup()` and add these lines immediately after it:

```cpp
setupTrackerChannel();
trackerModule = new TrackerModule();
strobeModule  = new StrobeModule();
```

Save and close the file.

---

## Step 7 — Connect the T-Beam

Plug the T-Beam into your Mac via USB, then check it is detected:

```bash
ls /dev/cu.*
```

You should see something like `/dev/cu.usbserial-0001` or `/dev/cu.SLAB_USBtoUART`. If nothing shows up, check your USB cable supports data (not charge-only) and that you rebooted after installing the driver in Step 1.

---

## Step 8 — Build and flash

```bash
cd ~/Documents/firmware
pio run -e tbeam-tracker --target upload
```

PlatformIO will download all dependencies on the first run — this takes several minutes. Subsequent builds are much faster. When it finishes you should see:

```
SUCCESS
```

If the upload fails or times out, hold the small **BOOT** button on the T-Beam while the upload starts, then release it.

---

## Step 9 — Verify it is running

Open the serial monitor:

```bash
pio device monitor --baud 115200
```

You should see output like:

```
TrackerModule: power LED GPIO 2, TX LED GPIO 4
StrobeModule: recovery strobe on GPIO 13
TrackerModule: QMC5883L compass online
TrackerModule: no GPS lock, skipping send
```

The "no GPS lock" message is normal indoors. Take the T-Beam outside and within a few minutes you should see:

```
TrackerModule: heading 157.3 deg
TrackerModule: packet sent (lat=..., lon=..., track=15730)
```

The strobe will also begin its 3-blink pattern once it has a lock.

Press `Ctrl+C` to exit the serial monitor.

---

## Step 10 — Updating the firmware later

When this repo is updated, pull the latest files and reflash:

```bash
# Pull latest from GitHub
cd ~/Documents/tbeam-tracker
git pull

# Re-copy changed files into firmware
cd ~/Documents/firmware
cp ~/Documents/tbeam-tracker/src/modules/TrackerModule.h   src/modules/
cp ~/Documents/tbeam-tracker/src/modules/TrackerModule.cpp src/modules/
cp ~/Documents/tbeam-tracker/src/modules/StrobeModule.h    src/modules/
cp ~/Documents/tbeam-tracker/src/modules/StrobeModule.cpp  src/modules/
cp ~/Documents/tbeam-tracker/src/tracker_channel.h         src/
cp ~/Documents/tbeam-tracker/platformio.ini                .

# Rebuild and flash
pio run -e tbeam-tracker --target upload
```

---

## Receiver node setup

The receiver runs stock Meshtastic firmware — no custom build needed. It just needs the matching channel config.

### What must match exactly

| Setting | Value |
|---|---|
| Channel name | `TRACKER` |
| PSK | The 32 bytes set in `tracker_channel.h` |
| Channel slot | 0 (primary) |

### Convert your PSK to base64

Run this on your Mac, substituting your actual key bytes:

```bash
python3 -c "
import base64
key = bytes([
    0x4a, 0x3f, 0x8c, 0x21, 0xd7, 0x55, 0xb2, 0x09,
    0xe1, 0x7a, 0x44, 0xfc, 0x30, 0x8e, 0x6b, 0xd3,
    0x92, 0x1c, 0x5f, 0xa8, 0x77, 0x03, 0xe6, 0x4d,
    0xbb, 0x29, 0x10, 0x58, 0xc4, 0x9d, 0x6e, 0xf1
])
print(base64.b64encode(key).decode())
"
```

### Apply via Meshtastic CLI (receiver connected by USB)

```bash
meshtastic --ch-index 0 --ch-set name TRACKER --ch-set psk base64:<your-base64-key> --ch-set-enabled true
```

### Apply via phone app

1. Open Meshtastic app and connect to the receiver node
2. Go to Channel Config → Channel 0
3. Set name to `TRACKER`
4. Set PSK to the same key
5. Save

### What you will see

Position packets from the tracker appear as a node on the map in the Meshtastic app, including GPS coordinates, altitude, satellite count, and compass heading. Any node or app on the same channel decodes them automatically.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Port not found in Step 7 | Check USB cable supports data; reboot after driver install |
| Upload fails / times out | Hold BOOT button on T-Beam while upload starts |
| Power LED not on | Check wiring on GPIO 2; change `POWER_LED_PIN` in TrackerModule.h if needed |
| TX LED not flashing | Check wiring on GPIO 4 |
| Compass heading missing | Verify SDA/SCL wiring; check serial monitor for 'QMC5883L not found' |
| No GPS lock indoors | Normal — take it outside, cold fix takes 2–5 min |
| Packets not received | Confirm both nodes use identical PSK bytes |
| Build fails | Ensure all four .h/.cpp files are in `src/modules/` |
| Strobe not flashing | Check MOSFET wiring on GPIO 13; verify gate pulldown resistor |
| Strobe always on | Missing 10kΩ gate pulldown — GPIO floats high at boot without it |
