# Meshtastic Tracker Display — T-Deck Module

Adds two extra screen pages to the LilyGo T-Deck for recovering
the tbeam-tracker GPS buoy at sea. All standard T-Deck Meshtastic
functionality is preserved.

---

## What it adds

### Page 1 — Radar view
- Track-up compass rose (your heading always at top)
- Your vessel at centre
- Tracker buoy plotted at correct relative bearing on the rose
- Small arrow on buoy showing which way it is pointing
- Distance in metres (under 1nm) or nautical miles
- Bearing to buoy in degrees true
- Buoy heading in degrees true
- Time since last packet
- Falls back to north-up if T-Deck has no GPS fix

### Page 2 — Data view
Full numeric readout: lat, lon, altitude, heading, satellites, PDOP, RSSI, time since last packet.

---

## Requirements

- LilyGo T-Deck running Meshtastic firmware 2.7.x
- tbeam-tracker node transmitting on channel 0 with your private PSK
- Both devices must use identical channel name (`TRACKER`) and PSK

---

## Step 1 — Clone the Meshtastic firmware

```bash
cd ~/Documents
git clone https://github.com/meshtastic/firmware.git
cd firmware
git submodule update --init
```

---

## Step 2 — Clone your repo

```bash
git clone https://github.com/Kels316/tbeam-tracker.git ~/Documents/tbeam-tracker
```

---

## Step 3 — Copy custom files into firmware

```bash
cd ~/Documents/firmware

cp ~/Documents/tbeam-tracker/tdeck-display/src/modules/TrackerDisplayModule.h   src/modules/
cp ~/Documents/tbeam-tracker/tdeck-display/src/modules/TrackerDisplayModule.cpp src/modules/
cp ~/Documents/tbeam-tracker/tdeck-display/platformio.ini .
```

---

## Step 4 — Patch main.cpp

```bash
open -e src/main.cpp
```

Add near the top with the other includes:

```cpp
#include "modules/TrackerDisplayModule.h"
```

Add inside `void setup()` after `nodeDB.init()`:

```cpp
trackerDisplayModule = new TrackerDisplayModule();
```

Save and close.

---

## Step 5 — Set your PSK

Connect the T-Deck via USB. Generate your base64 key:

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

Apply to the T-Deck:

```bash
meshtastic --ch-index 0 --ch-set name TRACKER --ch-set psk base64:<your-key> --ch-set-enabled true
```

> Use your actual key bytes if you changed the defaults in tracker_channel.h

---

## Step 6 — Build and flash

```bash
cd ~/Documents/firmware
pio run -e t-deck-tracker-display --target upload
```

---

## Step 7 — Verify

Open the serial monitor:

```bash
pio device monitor --baud 115200
```

When a packet arrives you should see:

```
TrackerDisplayModule: buoy fix lat=... lon=... track=...
```

The two new pages will appear in the T-Deck screen cycle.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Pages don't appear | Check main.cpp edits match main_patch.cpp exactly |
| "No buoy signal" on radar | Verify both devices use identical PSK and channel name |
| Bearing/distance not shown | T-Deck needs a GPS fix — take it outside |
| Build fails | Ensure both .h and .cpp are in `src/modules/` |
| Stale data on screen | Check tbeam-tracker is powered and has GPS lock (TX LED flashing) |
