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
