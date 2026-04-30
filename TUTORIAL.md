# How the Buoy Tracker Works — A Code Walkthrough

This project uses custom Meshtastic firmware to track a GPS buoy at sea in real time.
A **tracker node** sits in the buoy and transmits its GPS position over LoRa radio every 30 seconds.
A **T-Deck handheld receiver** picks up those transmissions and shows where the buoy is,
how far away it is, how fast it's drifting, and its compass heading.
An optional **repeater node** — flown on a drone — acts as a transparent relay to extend range
when the tracker and T-Deck can't reach each other directly.

All devices run a customised version of [Meshtastic](https://meshtastic.org/), open-source
firmware for LoRa mesh radio devices. We add our own code on top.

Two tracker hardware options are supported: the **LilyGo T-Beam** (ESP32, external M8Q GPS),
and the **Heltec WiFi LoRa 32 V4** (ESP32-S3, L76K GPS via dedicated GNSS connector).
The repeater also runs on a Heltec V4.

---

## Contents

1. [Concepts you'll need](#concepts-youll-need)
2. [The build system — how the code gets compiled](#the-build-system)
3. [The secret channel — tracker_channel.h (T-Beam)](#the-secret-channel-t-beam)
4. [The secret channel — tracker_channel.h (Heltec V4)](#the-secret-channel-heltec-v4)
5. [The repeater — repeater_channel.h](#the-repeater)
6. [T-Beam: The recovery strobe — StrobeModule](#t-beam-the-recovery-strobe)
7. [T-Beam / Heltec V4: GPS + compass + radio — TrackerModule](#t-beam--heltec-v4-trackermodule)
8. [Heltec V4 repeater: The status screen — RepeaterDisplayModule](#heltec-v4-repeater-status-screen)
9. [T-Deck: Receiving packets — TrackerRadarModule](#t-deck-receiving-packets)
10. [T-Deck: Calculating speed and drift](#t-deck-calculating-speed-and-drift)
11. [T-Deck: Building the screens — TrackerScreens](#t-deck-building-the-screens)
12. [T-Deck: Making the trackball work](#t-deck-making-the-trackball-work)
13. [T-Deck: Drawing the radar and data views](#t-deck-drawing-the-radar-and-data-views)
14. [How it all fits together](#how-it-all-fits-together)
15. [Where to go from here](#where-to-go-from-here)

---

## Concepts you'll need

Before diving into the code, here are the ideas that come up constantly throughout this project.

**LoRa radio** is a low-power, long-range radio technology. It can send short messages
kilometres over open water using very little power. It's not fast — no video or audio —
but it's ideal for sending a GPS position once every 30 seconds. The SX1262 chip on both
the T-Beam and Heltec V4 is the LoRa transceiver.

**Meshtastic** is open-source firmware for LoRa devices. It handles the hard parts for us:
sending packets, receiving, decrypting, mesh routing (relaying packets through other nodes).
We build on top of it by writing *modules*.

**A module** in Meshtastic is a C++ class that gets called automatically by the firmware
framework when something happens — a timer fires, or a radio packet arrives. You don't write
a `main()` loop; you register a module and Meshtastic calls it at the right time.
Modules inherit from base classes like `OSThread` (for timer-driven work) or `SinglePortModule`
(for packet handling).

**OSThread** is a Meshtastic base class for things that run on a repeating timer.
You override one method — `runOnce()` — which returns a delay in milliseconds.
Meshtastic calls `runOnce()` again after that delay. This is how both the tracker's
30-second broadcast and the strobe's blink pattern work.

**Protobuf** (Protocol Buffers) is a data format from Google. Meshtastic uses it to pack
structured data (like a GPS position with latitude, longitude, altitude, and heading)
into a very compact binary message for transmission over radio. On the other end, the binary
is decoded back into the same structured data.

**PSK** (Pre-Shared Key) is the encryption key baked into the firmware. Both the tracker
and receiver must have the exact same 32-byte key. If they don't match, the receiver
cannot decrypt the packets and will never see the buoy data.

**LVGL** is a graphics library for small embedded screens. On the T-Deck, it manages
the UI: creating screen objects, drawing labels, handling touch and trackball input.
Think of it as the "UI framework" for embedded devices.

**I2C** is a two-wire communication standard for chips on the same circuit board.
The compass sensor (QMC5883L) uses I2C — two wires (SDA and SCL) let the ESP32 send
commands to the compass and read back magnetic field data.

**Flash memory** is where ESP32 devices store configuration that survives power cycling.
When we save channel or device settings using `nodeDB->saveToDisk()`, we're writing to flash.

**UART serial** is how the GPS module communicates with the ESP32. The GPS chip sends
NMEA sentences (text strings like `$GNGGA,123456.00,2734.5,...`) over two wires:
TX (transmit) and RX (receive). The direction labels are from the perspective of each device —
the GPS's TX wire connects to the ESP32's RX pin, and vice versa.

---

## The build system

Before understanding the code, it helps to understand how the code gets compiled.
The tracker and repeater modules live in this `buoy-tracker` repo, but the compiler
needs them alongside the Meshtastic firmware source in `firmware-2.7.15`.
This creates a problem: how do you tell the build system about files that live outside
the firmware tree?

### PlatformIO and build_src_filter

PlatformIO (the build system used by Meshtastic) uses a setting called `build_src_filter`
to decide which `.cpp` files to compile. By default it includes everything under `src/`.
You can add (`+`) or remove (`-`) files or directories with glob patterns.

### The symlink approach

The cleanest solution is a Unix symlink. Instead of copying files or fighting with
PlatformIO's path handling, we create a symlink inside the firmware's `src/` directory
that points to our module source:

```
firmware-2.7.15/src/heltec_v4_tracker  →  buoy-tracker/nodes/heltec-v4/src/
```

To PlatformIO, `src/heltec_v4_tracker/` looks like a normal subdirectory of the firmware.
In reality it's just a pointer to the buoy-tracker repo. When we change a file in
`buoy-tracker/nodes/heltec-v4/src/`, the firmware build automatically picks up the change.

> **Why not use external paths directly?** PlatformIO's build system (based on SCons)
> stores a build state file (`.sconsign314.dblite`) that maps source file paths to compiled
> objects. When source paths contain `..` (going up a directory), SCons sometimes crashes
> writing this state file. The symlink sidesteps this entirely by making all paths appear
> to be inside the firmware tree.

### Excluding the tbeam modules

The Meshtastic firmware already contains `src/modules/TrackerModule.cpp` and
`src/modules/StrobeModule.cpp` (the T-Beam versions). If we also include our Heltec V4
versions via the symlink, we'll get duplicate symbol errors at link time.

We prevent this with `build_src_filter` exclusions:

```ini
build_src_filter =
    ${esp32s3_base.build_src_filter}
    -<modules/TrackerModule.cpp>   ; exclude tbeam version
    -<modules/StrobeModule.cpp>    ; exclude tbeam version
    +<heltec_v4_tracker/**>        ; include all our Heltec V4 modules via symlink
```

The repeater build goes further — it excludes the entire `heltec_v4_tracker/` directory
(which contains tracker-specific modules like `TrackerModule` and `StrobeModule`)
but then re-includes just the one file it needs:

```ini
build_src_filter =
    ${esp32s3_base.build_src_filter}
    -<modules/TrackerModule.cpp>
    -<modules/StrobeModule.cpp>
    -<heltec_v4_tracker/**>                                 ; exclude everything...
    +<heltec_v4_tracker/modules/RepeaterDisplayModule.cpp>  ; ...except this one
```

### The include path trick

When `main.cpp` includes our custom headers, it uses paths like:

```cpp
#include "src/modules/TrackerModule.h"
```

With the flag `-I../buoy-tracker/nodes/heltec-v4` in `build_flags`, the compiler searches
that directory when resolving includes. So `"src/modules/TrackerModule.h"` resolves to
`buoy-tracker/nodes/heltec-v4/src/modules/TrackerModule.h` — our Heltec V4 version.

The T-Beam version is at `firmware-2.7.15/src/modules/TrackerModule.h`. If we just wrote
`#include "modules/TrackerModule.h"`, both versions would be visible and the wrong one
might be used. The `src/modules/` prefix makes the path unique, because there's no
`src/src/modules/` inside the firmware tree — it can only resolve via our `-I` path.

This is called an **ODR (One Definition Rule) fix** — ensuring only one definition of
each class exists in the compiled binary.

---

## The secret channel (T-Beam)

**File:** `nodes/tbeam/tracker_channel.h`

All three node types share a private channel called `TRACKER` encrypted with a 256-bit PSK.
Anyone with an off-the-shelf Meshtastic device cannot read our packets — they'd need the same key.

This header file is included in `main.cpp` and runs once from `setup()`, before the
mesh service initialises. It writes channel and device configuration to flash.

### The idempotent guard

```cpp
void setupTrackerChannel()
{
    if (strcmp(channels.getByIndex(1).settings.name, TRACKER_CHANNEL_NAME) == 0) {
        LOG_INFO("setupTrackerChannel: already configured, skipping\n");
        return;
    }
```

`channels.getByIndex(1)` reads channel slot 1 from flash (slot 0 is always the public
default channel). `strcmp` compares two C strings — if the name already reads `"TRACKER"`,
we've been here before and can return immediately. This means the setup code only runs
on the very first boot after flashing. Every subsequent boot skips straight past it.

### The PSK

```cpp
static const uint8_t TRACKER_PSK[32] = {
    0x4a, 0x3f, 0x8c, 0x21, 0xd7, 0x55, 0xb2, 0x09,
    ...
};
```

This is the 32-byte (256-bit) AES encryption key. The `0x` prefix means hexadecimal —
each pair of hex digits is one byte. Every node in the system must have this exact key,
or they can't communicate. To change the key, generate a new one with
`openssl rand -hex 32`, convert each byte pair to `0xXX` format, and update this array
in all three `*_channel.h` files.

### Writing the channel

```cpp
meshtastic_ChannelSettings cs = meshtastic_ChannelSettings_init_default;
strncpy(cs.name, TRACKER_CHANNEL_NAME, sizeof(cs.name) - 1);
cs.psk.size = sizeof(TRACKER_PSK);
memcpy(cs.psk.bytes, TRACKER_PSK, sizeof(TRACKER_PSK));

meshtastic_Channel ch = meshtastic_Channel_init_default;
ch.settings = cs;
ch.role     = meshtastic_Channel_Role_SECONDARY;
ch.index    = 1;
channels.setChannel(ch);
channels.onConfigChanged();
```

A `meshtastic_ChannelSettings` struct holds the name and PSK. A `meshtastic_Channel`
wraps it with a role and slot index. `SECONDARY` means this is a secondary channel —
slot 0 is always PRIMARY and must remain the public default channel.
`channels.onConfigChanged()` triggers the channel manager to persist the new config.

### GPS and device settings

```cpp
config.position.rx_gpio  = 36;  // GPS TX line connects here (bits coming FROM GPS)
config.position.tx_gpio  = 25;  // GPS RX line connects here (bits going TO GPS)
config.device.role = meshtastic_Config_DeviceConfig_Role_TRACKER;
```

`rx_gpio` and `tx_gpio` are named from the **ESP32's perspective**: `rx_gpio` is the pin
where the ESP32 receives data (which connects to the GPS module's TX pin). This naming
can be confusing because the wire's direction label flips at each end.

`Role_TRACKER` is a Meshtastic device role that configures the node to broadcast position
frequently and use power optimisations suited to a GPS tracker.

```cpp
nodeDB->saveToDisk(SEGMENT_CONFIG);
nodeDB->saveToDisk(SEGMENT_MODULECONFIG);
```

`SEGMENT_CONFIG` saves the main config (LoRa, GPS, device role, Bluetooth).
`SEGMENT_MODULECONFIG` saves module-specific settings (telemetry intervals).
Without explicit `saveToDisk` calls, settings live only in RAM and are lost on reboot.

---

## The secret channel (Heltec V4)

**File:** `nodes/heltec-v4/tracker_channel.h`

The Heltec V4 tracker uses the same PSK and channel name but a different implementation.
The reason: `channels.setChannel()` silently fails when `channelFile.channels_count < 2`.

### Why setChannel() fails silently

Meshtastic stores channels in a protobuf array called `channelFile`. After a fresh flash,
this array may have `channels_count = 1` (only the primary channel exists).
`channels.getByIndex(1)` is a method that returns a reference to slot 1, but when the
array is too small it returns a reference to a *static throwaway buffer* — an empty struct
that's overwritten every call. Any writes to it are silently discarded.

The fix is to write directly to the underlying array and ensure it's large enough:

```cpp
// Expand the array if needed
if (channelFile.channels_count < 2)
    channelFile.channels_count = 2;

// Write directly — no setChannel() involved
memset(&channelFile.channels[1], 0, sizeof(channelFile.channels[1]));
channelFile.channels[1].index = 1;
channelFile.channels[1].role  = meshtastic_Channel_Role_SECONDARY;
strncpy(channelFile.channels[1].settings.name, TRACKER_CHANNEL_NAME,
        sizeof(channelFile.channels[1].settings.name) - 1);
channelFile.channels[1].settings.psk.size = sizeof(TRACKER_PSK);
memcpy(channelFile.channels[1].settings.psk.bytes, TRACKER_PSK, sizeof(TRACKER_PSK));
```

`memset` zeroes out the slot first — critical because leftover data from a previous boot
could corrupt the channel config. Then we fill in every field explicitly.

### Saving channels separately

```cpp
channels.onConfigChanged();
nodeDB->saveToDisk(SEGMENT_CHANNELS);
```

`channels.onConfigChanged()` rebuilds the channel manager's internal state from the
`channelFile` array. `SEGMENT_CHANNELS` is a separate flash segment for channel data.
The T-Beam version omits this explicit save (a bug we later fixed) — `onConfigChanged()`
alone doesn't write to flash.

### GPS pin assignment on the Heltec V4

```cpp
config.position.rx_gpio  = 39;  // GPS_RX_PIN: bits going TO the GPS (CPU transmits)
config.position.tx_gpio  = 38;  // GPS_TX_PIN: bits going FROM the GPS (CPU receives)
```

Wait — this looks backwards from what you'd expect. Here's why:

Meshtastic names these fields from the ESP32's perspective:
- `rx_gpio` = the pin where the ESP32 *receives* data = where the GPS *transmits*
- `tx_gpio` = the pin where the ESP32 *transmits* data = where the GPS *receives*

The Heltec V4 variant file (`variant.h`) defines:
- `GPS_RX_PIN = 39` — this is the GPS's RX (goes TO the GPS, so the CPU transmits here)
- `GPS_TX_PIN = 38` — this is the GPS's TX (comes FROM the GPS, so the CPU receives here)

So `config.position.rx_gpio` takes `GPS_RX_PIN` (39) — confusingly, that's the pin the
firmware calls "RX" but the GPS calls "TX." The labels refer to the GPS module's perspective
in the variant file, and the CPU's perspective in the config struct. They're opposite.

The practical rule: connect GPS-TX to ESP32-RX (pin 38), and GPS-RX to ESP32-TX (pin 39).
The L76K's dedicated GNSS connector on the Heltec V4 handles this automatically.

---

## The repeater

**File:** `nodes/heltec-v4/repeater_channel.h`

The repeater node is a Heltec V4 that acts as a dumb relay — it forwards any packet it
receives on the TRACKER channel to anyone who can hear it. Mount it on a drone hovering
between the buoy and the T-Deck to extend range when line-of-sight is blocked.

### Always enforce GPS off

```cpp
void setupRepeaterChannel()
{
    // Always apply GPS disable — even on subsequent boots.
    // The GPS hardware driver initialises before setup() runs on the first boot,
    // so GPS probing happens once regardless. After this saves to flash, the
    // next boot will start with GPS already disabled.
    if (config.position.gps_mode != meshtastic_Config_PositionConfig_GpsMode_DISABLED) {
        config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_DISABLED;
        nodeDB->saveToDisk(SEGMENT_CONFIG);
        LOG_INFO("setupRepeaterChannel: GPS disabled and saved\n");
    }
```

This block comes *before* the idempotent guard. The reason: on first boot, the Heltec V4's
GPS driver initialises early in hardware setup — before our `setup()` function runs —
and will probe for the GPS module regardless of config. This is unavoidable on first boot.

But by saving `gps_mode = DISABLED` to flash on that first boot, the *next* boot reads the
disabled flag before the GPS driver initialises, and skips the probing entirely.
So GPS noise appears only once, on the very first boot after flashing.

### The idempotent guard

```cpp
    if (config.device.role == meshtastic_Config_DeviceConfig_Role_REPEATER &&
        strcmp(channelFile.channels[1].settings.name, REPEATER_CHANNEL_NAME) == 0) {
        LOG_INFO("setupRepeaterChannel: already configured, skipping\n");
        return;
    }
```

Two conditions must both be true to skip setup: the device role must already be `REPEATER`,
and channel 1 must already be named `TRACKER`. This is stricter than the T-Beam guard,
which only checks the channel name. The dual check prevents a partial configuration
(e.g. channel set but role not saved) from being silently skipped.

### The REPEATER role

```cpp
    config.device.role = meshtastic_Config_DeviceConfig_Role_REPEATER;
```

`Role_REPEATER` is a built-in Meshtastic role. A node in this role:

- Rebroadcasts every received packet at full power
- Does not generate its own position packets
- Does not show up in the node list on other devices
- Optimises radio duty cycle for relaying rather than originating traffic

No custom modules are needed — Meshtastic's core mesh routing code handles all the
forwarding automatically once the role is set.

### Same PSK, same channel

```cpp
static const uint8_t REPEATER_PSK[32] = {
    0x4a, 0x3f, 0x8c, 0x21, ... // identical to TRACKER_PSK
};
```

The repeater uses the exact same PSK as the tracker and T-Deck. This is essential:
to relay a packet, the repeater must be able to receive and re-encrypt it using the
same key. If the keys didn't match, the repeater would receive unreadable ciphertext
and have nothing valid to relay.

---

## T-Beam: The recovery strobe

**Files:** `nodes/tbeam/src/modules/StrobeModule.h` / `StrobeModule.cpp`

The buoy has a bright white LED strobe driven through a MOSFET transistor on GPIO 13.
A MOSFET acts like an electrically-controlled switch — the ESP32 can't supply enough
current to drive a high-power LED directly (GPIO pins are limited to ~12 mA), so we use
a tiny signal from GPIO 13 to switch a transistor that handles the real current from the
5V supply rail.

### The header defines all the timing constants

```cpp
static constexpr uint8_t  STROBE_PIN        = 13;
static constexpr uint32_t BLINK_ON_MS       = 80;    // duration of each flash
static constexpr uint32_t BLINK_GAP_MS      = 150;   // gap between flashes in a burst
static constexpr uint32_t INTERVAL_LOCK_MS  = 30000; // 30s between bursts when GPS locked
static constexpr uint32_t INTERVAL_NOLOCK_MS = 5000; //  5s between bursts when searching
```

`constexpr` means these values are evaluated at compile time and baked into the binary —
they're not variables that can accidentally change at runtime. Grouping them in the header
means you can adjust the timing without hunting through the implementation file.

### Inheriting from OSThread

```cpp
StrobeModule::StrobeModule()
    : concurrency::OSThread("StrobeModule")
{
    pinMode(STROBE_PIN, OUTPUT);
    digitalWrite(STROBE_PIN, LOW);
}
```

`concurrency::OSThread` is the Meshtastic timer base class. Passing `"StrobeModule"` as
the thread name lets you identify it in debug logs. `pinMode(STROBE_PIN, OUTPUT)` configures
the GPIO as a digital output. `digitalWrite(STROBE_PIN, LOW)` ensures the LED is off
immediately at boot — important because GPIO pins can float to an indeterminate state
during the brief moment before the constructor runs.

### runOnce() — the main loop

```cpp
int32_t StrobeModule::runOnce()
{
    bool hasLock = (gps && gps->hasLock());
    doBlinks(3);
    return hasLock ? INTERVAL_LOCK_MS : INTERVAL_NOLOCK_MS;
}
```

`runOnce()` returns a delay in milliseconds. Meshtastic waits that long, then calls
`runOnce()` again. The `gps` pointer is a global set by the Meshtastic GPS driver —
we check it's non-null before calling `hasLock()` to avoid a null pointer crash during
the brief period between boot and GPS driver initialisation.

If the GPS has a 3D fix (`hasLock() == true`), we flash every 30 seconds — matching the
position broadcast interval so both the strobe and the radio fire together.
Without a fix, we flash every 5 seconds so a SAR team can see the buoy is powered on
and searching, even if it's not yet transmitting a valid position.

### doBlinks() — producing the flash pattern

```cpp
void StrobeModule::doBlinks(uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        digitalWrite(STROBE_PIN, HIGH);
        delay(BLINK_ON_MS);
        digitalWrite(STROBE_PIN, LOW);
        if (i < count - 1)
            delay(BLINK_GAP_MS);
    }
}
```

`delay()` is a blocking pause — the ESP32 halts completely for that many milliseconds.
This is acceptable here because the strobe module is a low-priority background thread;
it doesn't matter if it blocks for a few hundred milliseconds.

The `if (i < count - 1)` check avoids adding a trailing gap after the last flash —
without it, the module would wait 150 ms unnecessarily at the end of each burst before
returning to sleep.

---

## T-Beam / Heltec V4: TrackerModule

**Files:** `nodes/tbeam/src/modules/TrackerModule.h` / `TrackerModule.cpp`
(and the Heltec V4 equivalents in `nodes/heltec-v4/src/modules/`)

This is the heart of the tracker node. Every 30 seconds it reads the GPS position,
reads a compass heading, packs everything into a protobuf position packet, and
broadcasts it on the TRACKER channel.

### Inheriting from two classes

```cpp
class TrackerModule : public SinglePortModule, private concurrency::OSThread
```

C++ allows inheriting from multiple base classes. `SinglePortModule` provides the ability
to build and send Meshtastic packets on a specific protocol port. `OSThread` provides the
30-second timer. By combining them, `TrackerModule` is both a periodic task and a
radio transmitter.

`private` inheritance from `OSThread` means the timer methods are not visible to outside
code — they're an implementation detail. `public` inheritance from `SinglePortModule`
means packet-handling methods are part of the public interface.

### The boot delay

```cpp
static constexpr uint32_t BOOT_DELAY_MS = 15000;  // 15 seconds
static constexpr uint32_t BOOT_POLL_MS  = 500;    // check every 500ms during delay

int32_t TrackerModule::runOnce()
{
    if (firstRun) {
        if (millis() < BOOT_DELAY_MS)
            return BOOT_POLL_MS;
        firstRun = false;
        sendHello();
    }
    sendPosition();
    return INTERVAL_MS;
}
```

`millis()` returns the number of milliseconds since the ESP32 booted. On first run, we
check whether 15 seconds have elapsed. If not, we return early and try again in 500 ms.

Why 15 seconds? Because `setupTrackerChannel()` runs before `service->init()` in `setup()`,
and the mesh service takes a few seconds after `init()` to fully settle — build its channel
table, register encryption keys, and prepare the radio stack. If we try to send a packet
immediately after boot, the service returns `Error=6 (NO_CHANNEL)` — it doesn't yet know
which channel to use. The 15-second delay ensures the service is fully ready.

### sendHello() — the boot diagnostic

```cpp
void TrackerModule::sendHello()
{
    static const char msg[] = "TRACKER ONLINE";
    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p) return;
    p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p->decoded.portnum       = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->decoded.payload.size  = strlen(msg);
    memcpy(p->decoded.payload.bytes, msg, p->decoded.payload.size);
    p->to      = NODENUM_BROADCAST;
    p->channel = 1;
    p->want_ack = false;
    service->sendToMesh(p, RX_SRC_LOCAL, true);
}
```

`router->allocForSending()` allocates a packet from a fixed pool of preallocated buffers.
If the pool is empty (all buffers in use), it returns `nullptr` — we check for this before
using the pointer. This "pool allocation" pattern avoids dynamic memory allocation (`new`/`delete`)
which would cause heap fragmentation on a microcontroller running for weeks.

We use `TEXT_MESSAGE_APP` rather than a position packet because we just want a visible
message on the T-Deck screen confirming the tracker is alive. The T-Deck displays text
messages directly in its UI.

`p->channel = 1` is critical — sending on channel 0 (the public default) would expose
our traffic. Channel 1 is our encrypted TRACKER channel.

`RX_SRC_LOCAL` tells the service this packet originated here (as opposed to being received
from the mesh and being forwarded).

### Setting up the compass

```cpp
Wire.beginTransmission(QMC_ADDR);  // QMC_ADDR = 0x0D
Wire.write(QMC_REG_RESET);
Wire.write(0x01);
Wire.endTransmission();
delay(10);
```

`Wire` is the Arduino I2C library. `beginTransmission(addr)` starts an I2C transaction
to device address `0x0D` (the QMC5883L compass). `write(reg)` then `write(value)` sends
a register address followed by the value to write. `endTransmission()` commits the
transaction and returns 0 on success, non-zero on failure.

The reset command (`0x01` to the reset register) forces the chip into a known state —
like pressing a reset button. We wait 10 ms for it to restart.

```cpp
Wire.write(QMC_CTRL1_VAL);  // continuous mode, 200 Hz, 8 gauss range, 512 oversampling
if (Wire.endTransmission() == 0) {
    compassOk = true;
}
```

`QMC_CTRL1_VAL` is a bitmask that configures the measurement mode. Bits in a control
register each mean something — this particular value enables continuous measurement
(rather than single-shot), sets the output data rate to 200 Hz (samples per second),
the full-scale range to ±8 gauss (enough for Earth's field), and 512 oversampling
(averages 512 readings per output to reduce noise).

### Reading the compass heading

```cpp
Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)6);
int16_t x = (int16_t)(Wire.read() | (Wire.read() << 8));
int16_t y = (int16_t)(Wire.read() | (Wire.read() << 8));
// Z is read but discarded for a horizontal compass
Wire.read(); Wire.read();
```

`requestFrom(addr, count)` asks the chip to send 6 bytes. The chip sends two bytes each
for X, Y, and Z magnetic field strength. Each pair arrives low byte first (little-endian).
`Wire.read() << 8` shifts the second byte into the upper 8 bits of a 16-bit integer,
then `|` (bitwise OR) combines it with the low byte to form a signed 16-bit value.

`int16_t` is a signed 16-bit integer (-32768 to +32767). The magnetic field can be
positive or negative depending on direction.

```cpp
float heading = atan2f((float)y, (float)x) * (180.0f / M_PI);
heading += MAG_DECLINATION;   // add local magnetic declination (~11.5° for SE Queensland)
if (heading < 0.0f)    heading += 360.0f;
if (heading >= 360.0f) heading -= 360.0f;
```

`atan2f(y, x)` is the two-argument arctangent. It takes a Y component and an X component
and returns the angle of that vector in radians (range: -π to +π).
Multiplying by `(180 / π)` converts to degrees.

Magnetic north differs from true north by the *magnetic declination*, which varies by
location. In south-east Queensland it's approximately +11.5° (magnetic north is 11.5°
east of true north). Adding the declination converts magnetic heading to true heading.

The final two lines wrap the result into 0–359°.

### Building and transmitting the position packet

```cpp
meshtastic_Position pos = meshtastic_Position_init_default;
pos.has_latitude_i  = true;   pos.latitude_i  = localPosition.latitude_i;
pos.has_longitude_i = true;   pos.longitude_i = localPosition.longitude_i;
pos.has_altitude    = true;   pos.altitude    = localPosition.altitude;
pos.has_ground_track = true;  pos.ground_track = (uint32_t)(heading * 100.0f);
pos.has_sats_in_view = true;  pos.sats_in_view = localPosition.sats_in_view;
```

`meshtastic_Position_init_default` creates a zeroed position struct. Every field in a
protobuf message has a matching `has_X` boolean — you must set it to `true` or the field
is treated as absent and the receiver ignores it. This explicit "has" flag allows protobuf
to distinguish "field is zero" from "field is not present."

Latitude and longitude are stored as signed integers: degrees multiplied by 10,000,000.
So 27.5° S = -275,000,000. Integer arithmetic avoids floating-point precision loss that
would accumulate over radio encoding and decoding.

We reuse `ground_track` (normally GPS course-over-ground) to carry the compass heading.
Multiplying by 100 preserves one decimal place as an integer — 157.3° becomes 15730.
The T-Deck knows to divide by 100 when displaying it.

```cpp
meshtastic_MeshPacket *p = router->allocForSending();
p->decoded.payload.size = pb_encode_to_bytes(
    p->decoded.payload.bytes,
    sizeof(p->decoded.payload.bytes),
    &meshtastic_Position_msg, &pos);
p->to      = NODENUM_BROADCAST;
p->channel = 1;
service->sendToMesh(p, RX_SRC_LOCAL, true);
```

`pb_encode_to_bytes` serialises the `pos` struct into compact binary protobuf format
and writes it into the packet's payload buffer, returning the number of bytes written.
`NODENUM_BROADCAST` means "everyone" — no specific destination node.
`service->sendToMesh` hands the packet to the radio stack: it encrypts it with the TRACKER
channel PSK and queues it for transmission.

### flashTxLed() — visual TX confirmation

```cpp
void TrackerModule::flashTxLed()
{
    digitalWrite(TX_LED_PIN, HIGH);
    delay(50);
    digitalWrite(TX_LED_PIN, LOW);
}
```

A brief 50 ms flash on the TX indicator LED gives visual confirmation that a packet was
sent. On the Heltec V4, this uses the built-in GPIO 35 LED. On the T-Beam, a dedicated
TX LED pin is wired to an external indicator.

---

## Heltec V4 repeater: Status screen

**Files:** `nodes/heltec-v4/src/modules/RepeaterDisplayModule.h` / `RepeaterDisplayModule.cpp`

The repeater has no GPS and runs no custom packet modules. But it does have an OLED screen.
`RepeaterDisplayModule` adds a custom status page to the Meshtastic screen rotation showing
battery level, uptime, channel utilisation, and node ID.

### Plugging into the Meshtastic UI system

Meshtastic builds its screen from a list of "frames" — callbacks that draw one page each.
The standard frames are things like the node info page, the GPS page, and the LoRa info page.
Custom modules can add their own frames by implementing two methods from `MeshModule`:

```cpp
class RepeaterDisplayModule : public MeshModule
{
  public:
    virtual bool wantUIFrame() override { return true; }
    virtual void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state,
                           int16_t x, int16_t y) override;
```

Returning `true` from `wantUIFrame()` tells the screen system: "this module wants to draw
a frame." The screen system then calls `drawFrame()` whenever it's this module's turn in
the rotation. The user scrolls between frames using the built-in button.

`MeshModule` also requires `wantPacket()` and `handleReceived()` to be implemented.
Since we're only drawing — not processing radio packets — both return "not interested":

```cpp
    virtual bool wantPacket(const meshtastic_MeshPacket *p) override { return false; }
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override
        { return ProcessMessage::CONTINUE; }
```

`ProcessMessage::CONTINUE` tells the Meshtastic packet dispatcher: "I didn't consume this
packet — pass it on to the next module." Returning `STOP` would prevent other modules
from seeing the packet, which we don't want.

### Instantiation

The module must be instantiated in `main.cpp` after `setupModules()`:

```cpp
#ifdef USE_REPEATER_MODULE
    repeaterDisplayModule = new RepeaterDisplayModule();
#endif
```

The `new` call runs the constructor, which calls the `MeshModule` base constructor with
`"RepeaterDisplay"` as the module name. The `MeshModule` constructor automatically
registers the module with the global module list. The screen system queries that list
when building its frame set.

### Drawing the status page

```cpp
void RepeaterDisplayModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state,
                                      int16_t x, int16_t y)
{
    display->clear();
    display->setFont(FONT_SMALL);
    display->setTextAlignment(TEXT_ALIGN_LEFT);

    graphics::drawCommonHeader(display, x, y, "REPEATER");
```

`display->clear()` wipes the framebuffer. `drawCommonHeader` is a shared Meshtastic
helper that draws an inverted header bar with a centred title and an automatic battery
icon in the corner — the icon fills with bars based on charge level and animates a
lightning bolt while charging. We get all of this for free by calling one function.

```cpp
    uint8_t batPct = powerStatus ? powerStatus->getBatteryChargePercent() : 0;
    float   batV   = powerStatus ? powerStatus->getBatteryVoltageMv() / 1000.0f : 0.0f;
    bool    usb    = powerStatus && powerStatus->getHasUSB();

    if (usb)
        snprintf(buf, sizeof(buf), "Batt: USB  %.2fV", batV);
    else
        snprintf(buf, sizeof(buf), "Batt: %d%%  %.2fV", batPct, batV);
    display->drawString(leftX, y + textFirstLine, buf);
```

`powerStatus` is a global Meshtastic pointer to the current power state. We guard against
it being null (before the power subsystem initialises). `getBatteryVoltageMv()` returns
millivolts, so we divide by 1000 to get volts. `%.2fV` formats to two decimal places.

When powered via USB with no battery, `getBatteryChargePercent()` returns 101
(a sentinel meaning "USB power, no battery measurement"). We show "USB" in that case
rather than a meaningless 101%.

```cpp
    uint32_t secs = millis() / 1000;
    uint32_t h = secs / 3600; secs %= 3600;
    uint32_t m = secs / 60;   secs %= 60;
    snprintf(buf, sizeof(buf), "Up: %uh %02um %02us", h, m, secs);
    display->drawString(leftX, y + textSecondLine, buf);
```

`millis() / 1000` converts milliseconds to seconds. Integer division and the modulo
operator (`%`) peel off hours, then minutes, then the remaining seconds. `%02u` formats
an unsigned integer padded with leading zeros to at least two digits — so 5 seconds
shows as `05`, not `5`.

```cpp
    float chUtil = airTime ? airTime->channelUtilizationPercent() : 0.0f;
    float txUtil = airTime ? airTime->utilizationTXPercent()      : 0.0f;
    snprintf(buf, sizeof(buf), "Ch: %.1f%%  TX: %.1f%%", chUtil, txUtil);
    display->drawString(leftX, y + textThirdLine, buf);
```

`airTime` is a global Meshtastic object that tracks radio time-on-air statistics.
`channelUtilizationPercent()` is the percentage of time the channel has been busy
(RX + TX combined) over the last sliding window. `utilizationTXPercent()` is just TX.
For a repeater, high channel utilisation means it's doing its job — relaying traffic.
Regulatory duty cycle limits (typically 1% in many regions) can be monitored here.

```cpp
    uint32_t nodeNum = nodeDB->getNodeNum();
    snprintf(buf, sizeof(buf), "ID: !%08x", nodeNum);
    display->drawString(leftX, y + textFourthLine, buf);

    graphics::drawCommonFooter(display, x, y);
```

`nodeDB->getNodeNum()` returns the 32-bit node number — Meshtastic's unique identifier
for this device (derived from the WiFi/BT MAC address). The `!` prefix and 8-digit hex
format is the standard Meshtastic node ID notation (e.g. `!a1b2c3d4`).

`drawCommonFooter` draws a thin line at the bottom of the display, separating the content
from any footer info.

The `textFirstLine`, `textSecondLine` etc. macros come from `SharedUIDisplay.h` and define
the Y pixel offsets for each line of text below the header bar, tightly spaced to fit
the maximum number of lines on the 64-pixel-tall OLED.

---

## T-Deck: Receiving packets

**Files:** `receiver/tdeck/src/modules/TrackerDisplayModule.h` / `TrackerDisplayModule.cpp`

On the T-Deck, `TrackerRadarModule` inherits from `SinglePortModule` and listens for
packets on the `POSITION_APP` port. When one arrives, it decodes the protobuf and stores
the data in a shared global struct that the display code reads.

### Port registration

```cpp
TrackerRadarModule::TrackerRadarModule()
    : SinglePortModule("TrackerRadar", meshtastic_PortNum_POSITION_APP)
```

`SinglePortModule` is a Meshtastic base class for modules that handle one specific packet
port. By passing `POSITION_APP` to the parent constructor, we register with the Meshtastic
packet dispatcher: "call my `handleReceived()` whenever a `POSITION_APP` packet arrives."
We don't need to poll or write any routing code — the dispatcher handles all of that.

### Filtering packets to our channel

```cpp
ProcessMessage TrackerRadarModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (mp.channel > 1) return ProcessMessage::CONTINUE;
```

We only care about packets on channel 0 (public) or 1 (TRACKER). Packets from channels 2+
are ignored by returning `CONTINUE`. This is future-proofing: if additional channels are
added to a mesh later, they won't produce false buoy readings on our display.

### Decoding the protobuf

```cpp
meshtastic_Position pos = meshtastic_Position_init_default;
pb_istream_t stream = pb_istream_from_buffer(
    mp.decoded.payload.bytes, mp.decoded.payload.size);

if (!pb_decode(&stream, &meshtastic_Position_msg, &pos)) {
    return ProcessMessage::CONTINUE;
}
```

`pb_istream_from_buffer` wraps the raw byte array in a "stream" object that the protobuf
library can read from. `pb_decode` reads those bytes and fills in our `pos` struct.
The `meshtastic_Position_msg` descriptor tells the decoder the structure of the message —
which bytes map to which fields.

If decoding fails (corrupted packet, wrong format, or a different message type accidentally
arriving on the POSITION_APP port), we return `CONTINUE` and discard it silently.

### Storing the data

```cpp
g_tracker.latitude_i   = pos.latitude_i;
g_tracker.longitude_i  = pos.longitude_i;
g_tracker.ground_track = pos.ground_track;  // compass heading × 100
g_tracker.last_rx_ms   = millis();
g_tracker.rssi         = mp.rx_rssi;
g_tracker.snr          = mp.rx_snr;
g_tracker.valid        = true;
```

`g_tracker` is a global struct shared between the packet handler and the display code.
This is intentionally simple — a shared memory buffer is fine because the ESP32 is
single-threaded (LVGL and the Meshtastic event loop run on the same core in a cooperative
multitasking arrangement; they can't race each other).

`millis()` gives us a timestamp for "how old is this data?" — displayed on the data screen.

`mp.rx_rssi` is the received signal strength in dBm (e.g. -85 dBm). More negative = weaker
signal = greater distance or more obstacles. `mp.rx_snr` is the signal-to-noise ratio —
positive values mean the signal is above the noise floor, negative values mean the
radio is decoding a packet that's barely audible above background noise.

After storing the raw data, `handleReceived()` calls `updateHistory()` to compute the
derived motion data (speed and drift direction).

---

## T-Deck: Calculating speed and drift

### The anchor model

Instead of a rolling average over recent positions, we use an **anchor model**:
the very first packet received is stored permanently as the *anchor* and never changes.
Every subsequent packet is compared against the anchor to give:

- **Set** — the direction the buoy has drifted (course over ground, COG)
- **Drift** — the speed it's moving (in knots)

This is exactly what Search and Rescue needs: net displacement from a known start point
over the full deployment duration. A rolling average would hide long-term drift by
including recent back-and-forth motion due to wave action.

```cpp
if (!g_anchorSet) {
    g_anchor.latitude_i  = lat_i;
    g_anchor.longitude_i = lon_i;
    g_anchor.rx_ms       = rx_ms;
    g_anchor.valid       = true;
    g_anchorSet = true;
    g_tracker.motion_valid = false;
    return;
}
```

On the very first call, we save the position and return immediately. There's nothing to
compare against yet, so we mark motion as invalid. The display will show `---` for speed
and COG until the second packet arrives.

### The Haversine formula

```cpp
float TrackerRadarModule::distanceMetres(float lat1, float lon1, float lat2, float lon2)
{
    float dLat = (lat2 - lat1) * DEG2RAD;
    float dLon = (lon2 - lon1) * DEG2RAD;
    float a = sinf(dLat/2)*sinf(dLat/2) +
              cosf(lat1*DEG2RAD)*cosf(lat2*DEG2RAD)*sinf(dLon/2)*sinf(dLon/2);
    return 6371000.0f * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f-a));
}
```

The Earth is a sphere, so you can't just subtract latitudes and longitudes to get distance —
the result would be wrong for any distance more than a few hundred metres.
The **Haversine formula** computes the great-circle distance (shortest path along the
Earth's surface) between two GPS coordinates.

`DEG2RAD = π/180` converts degrees to radians, which is what the trigonometric functions
(`sinf`, `cosf`, `atan2f`) expect. The constant `6371000.0f` is the Earth's mean radius
in metres. The result is the true surface distance in metres.

### The GPS jitter filter

GPS chips have a position noise of several metres, even when completely stationary.
Without filtering, this noise produces a fake speed reading of 0.1–0.5 knots.

```cpp
float distM = distanceMetres(ancLat, ancLon, nowLat, nowLon);

if (distM < MIN_MOVE_METRES) {   // MIN_MOVE_METRES = 15.0
    g_tracker.speed_kn     = 0.0f;
    g_tracker.cog_deg      = 0.0f;
    g_tracker.motion_valid = true;
    return;
}
```

If the buoy hasn't moved more than 15 metres from the anchor, it's considered stationary.
We report exactly zero rather than "data not available" — the display shows `0.0kn / 000°T`
instead of `---`. The distinction matters operationally: "stationary" and "no data" are
different situations.

15 metres is a conservative threshold. A real ocean buoy will almost certainly drift more
than 15 metres in any meaningful deployment, so false zeroes from wave bobbing are unlikely.

### Speed and bearing

```cpp
float dtSec = (float)(rx_ms - g_anchor.rx_ms) / 1000.0f;
g_tracker.cog_deg  = bearingTo(ancLat, ancLon, nowLat, nowLon);
g_tracker.speed_kn = (dtSec > 0.0f) ? (distM / dtSec) * MPS_TO_KN : 0.0f;
```

Speed = distance ÷ time. `distM` is in metres, `dtSec` is in seconds, so `distM / dtSec`
gives metres per second. `MPS_TO_KN = 1.94384` converts m/s to knots.

`bearingTo()` uses `atan2f` to compute the compass bearing from the anchor position
to the current position. This is the direction the buoy has drifted.

---

## T-Deck: Building the screens

The T-Deck uses LVGL, a full-featured graphics library for embedded devices.
LVGL manages a hierarchy of screen objects (`lv_obj_t`). Screens aren't displayed
until explicitly loaded — you can create all your screens at startup and switch between
them instantly later.

### Creating a screen

```cpp
lv_obj_t *rs = lv_obj_create(NULL);
lv_obj_set_style_bg_color(rs, lv_color_black(), 0);
radarScr = rs;
```

`lv_obj_create(NULL)` creates a screen object (passing `NULL` as parent means "this is
a root-level screen, not a child of anything"). We save the pointer in `radarScr`.
The `0` in style calls is a "selector" — it applies the style to the default state of
the object (as opposed to pressed, focused, disabled states).

### The canvas — a raw pixel buffer

```cpp
lv_obj_t *cv = lv_canvas_create(rs);
lv_canvas_set_buffer(cv, radarBuf, CANVAS_W, CANVAS_H, LV_COLOR_FORMAT_RGB565);
```

A canvas is a rectangular block of memory where we can draw pixels, lines, circles, and
text directly. `radarBuf` is allocated in PSRAM — the ESP32-S3's external RAM chip.
`LV_COLOR_FORMAT_RGB565` means each pixel takes 2 bytes: 5 bits red, 6 bits green,
5 bits blue. A 320×240 canvas takes 320 × 240 × 2 = 153,600 bytes (~150 KB).

We draw the radar rose, distance rings, and buoy dot on this canvas by calculating pixel
coordinates in code and writing them directly to the buffer.

### Labels for text display

```cpp
static lv_obj_t *makeLabel(lv_obj_t *parent, int x, int y, int w, int h, const char *txt)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_obj_set_pos(lbl, x, y);
    lv_obj_set_size(lbl, w, h);
    lv_label_set_text(lbl, txt);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    return lbl;
}
```

Labels are LVGL text objects. `lv_label_set_text` sets the initial content.
Later, the same function can update the text at any time — LVGL automatically marks
the label as needing a redraw and updates it on the next render cycle.
We store pointers to the labels we'll update (bearing, speed, distance, etc.) as
static variables so the update code can reach them.

### Invisible button for trackball press

```cpp
lv_obj_t *rb = lv_btn_create(rs);
lv_obj_set_size(rb, 320, 240);
lv_obj_set_style_bg_opa(rb, LV_OPA_TRANSP, 0);  // fully transparent
lv_obj_add_event_cb(rb,
    [](lv_event_t *e){ TrackerScreens::enterData(); },
    LV_EVENT_CLICKED, NULL);
```

The entire radar screen has a full-screen invisible button. When the user presses the
trackball (which generates an `LV_EVENT_CLICKED`), this button fires its callback and
navigates to the data screen. This is much simpler than handling raw input events
directly and integrates cleanly with LVGL's input system.

The lambda `[](lv_event_t *e){ ... }` is a C++ anonymous function — a quick way to
write a short callback without defining a named function separately.

---

## T-Deck: Making the trackball work

The T-Deck's trackball generates encoder input events in LVGL. We intercept these to
drive screen navigation.

### Hooking the encoder input device

```cpp
lv_indev_t *indev = lv_indev_get_next(NULL);
while (indev) {
    if (lv_indev_get_type(indev) == LV_INDEV_TYPE_ENCODER) {
        s_origEncReadCb = lv_indev_get_read_cb(indev);
        lv_indev_set_read_cb(indev, [](lv_indev_t *dev, lv_indev_data_t *data) {
            if (s_origEncReadCb) s_origEncReadCb(dev, data);
            if (data->key == LV_KEY_ENTER && data->state == LV_INDEV_STATE_PRESSED) {
                data->state = LV_INDEV_STATE_RELEASED;
                if (TrackerScreens::currentPage == TrackerScreens::Page::None)
                    lv_async_call([](void*){ TrackerScreens::enterRadar(); }, NULL);
                // ... other page transitions ...
            }
        });
        break;
    }
    indev = lv_indev_get_next(indev);
}
```

`lv_indev_get_next(NULL)` starts a walk through all registered LVGL input devices.
We loop until we find the one with type `ENCODER` (the trackball). We save the original
read callback, then replace it with our own.

Our callback first calls the original callback (`s_origEncReadCb`) — this ensures the
trackball continues to work normally for LVGL's own focus navigation. Then we check if
the trackball was pressed (`LV_KEY_ENTER`). If so, we override the state to `RELEASED`
(to prevent the press from also firing any focused LVGL widget) and trigger a screen
transition.

### Why lv_async_call?

```cpp
lv_async_call([](void*){ TrackerScreens::enterRadar(); }, NULL);
```

`lv_async_call` schedules a function to run at the end of the current LVGL processing
cycle, rather than immediately. We can't switch screens *inside* an input device callback —
LVGL is mid-way through processing the event, and swapping the active screen from within
that stack can cause crashes or corrupted UI state. `lv_async_call` defers the switch
until LVGL is idle.

### Capturing the main screen lazily

```cpp
void TrackerScreens::enterRadar()
{
    if (!mainScr) mainScr = lv_screen_active();
    lv_scr_load(radarScr);
    currentPage = Page::Radar;
}
```

When our setup code runs during Meshtastic's boot sequence, the device UI's main screen
hasn't been created yet. Instead of trying to capture it at init time (which would give
us a null pointer), we capture it lazily — the first time the user navigates away from it.
At that moment, the main screen is guaranteed to be the active one.

---

## T-Deck: Drawing the radar and data views

### The compass rose

```cpp
void TrackerScreens::drawCompassRose(void *layer, int cx, int cy, int r, float rotDeg)
{
    canvasCircle(layer, cx, cy, r,     2, 0xFFFFFF);  // outer ring, white, 2px thick
    canvasCircle(layer, cx, cy, r - 8, 1, 0x888888);  // inner ring, grey, 1px thick

    const char *cardinals[] = { "N", "E", "S", "W" };
    for (int i = 0; i < 4; i++) {
        float rad = ((float)(i * 90) - rotDeg - 90.0f) * DEG2RAD;
        int lx = cx + (int)((r - 18) * cosf(rad));
        int ly = cy + (int)((r - 18) * sinf(rad));
        canvasText(layer, lx - 8, ly - 8, 16, 16, cardinals[i], 0xFFFFFF);
    }
}
```

`rotDeg` is the vessel's current heading. Subtracting it from each cardinal's angle
keeps the display **track-up**: our heading always points toward the top of the screen,
just like a ship's radar or chart plotter. North moves around the compass as we turn.

For each cardinal direction (N at 0°, E at 90°, S at 180°, W at 270°), we:
1. Calculate the angle adjusted for rotation and convert to radians
2. Use `cosf` and `sinf` to find the X and Y pixel offsets on the circle edge
   (the formula `cx + r × cos(angle)` is standard circle geometry)
3. Draw the letter at that pixel position

### Plotting the buoy

```cpp
float bearDeg  = bearingTo(ownLat, ownLon, buoyLat, buoyLon);
float relBear  = fmodf(bearDeg - rotation + 360.0f, 360.0f);
float plotRad  = (relBear - 90.0f) * DEG2RAD;
float inner    = r * (distM / maxRange);   // scale distance to pixel radius
int bx = ROSE_CX + (int)(inner * cosf(plotRad));
int by = ROSE_CY + (int)(inner * sinf(plotRad));
canvasFilledCircle(&layer, bx, by, 6, 0xFF4444);  // red dot
```

`bearDeg` is the absolute compass bearing from our position to the buoy.
`relBear` subtracts our own heading, making the bearing *relative* to our bow.
A buoy dead ahead gets `relBear = 0°` regardless of which direction we're sailing.

`- 90.0f` corrects for the difference between compass convention (0° = north = screen up)
and screen/maths convention (0° = right = east). Without this correction, "north" would
appear on the right of the screen instead of the top.

`inner` scales the distance to a pixel radius: if the buoy is at half the maximum range,
the dot appears halfway between the centre and the edge of the rose.

`fmodf` is the floating-point modulo operator — it wraps the bearing into the 0–360 range
after subtracting the rotation. The `+ 360.0f` before the modulo prevents negative values
from appearing when the bearing is smaller than the rotation.

### Updating text labels

```cpp
char buf[32];
snprintf(buf, sizeof(buf), "Brg %03.0f\xC2\xB0T", bearDeg);
lv_label_set_text((lv_obj_t*)radarBrg, buf);

snprintf(buf, sizeof(buf), "%.1fkn", g_tracker.speed_kn);
lv_label_set_text((lv_obj_t*)dataSpdLbl, buf);
```

`snprintf` writes a formatted string into `buf`, stopping at 31 characters to leave room
for the null terminator. `%03.0f` means: floating-point, 0 decimal places, padded to
3 digits with leading zeros — so 45° appears as `045`.

`\xC2\xB0` is the UTF-8 byte sequence for the degree symbol °. The T-Deck's LVGL font
supports UTF-8, so these two bytes render as a single ° character.

`lv_label_set_text` updates the label contents. LVGL marks the label dirty and redraws
it on the next render cycle — you don't need to manually trigger a redraw.

### The 1-second LVGL timer

```cpp
static lv_timer_t *uiTimer = lv_timer_create(
    [](lv_timer_t *) { TrackerScreens::onTimer(); }, 1000, NULL);
```

`lv_timer_create` creates an LVGL-managed repeating timer (not an OS timer). It calls
the callback every 1000 milliseconds. The callback redraws whichever screen is active —
clearing the canvas, recalculating positions, redrawing the rose, and updating all labels.

The 1-second interval means the display updates once per second, even though packets
arrive every 30 seconds. Between packets, the data stays the same but the "age" timer
on the data screen counts up: `Last: 45s ago`.

---

## How it all fits together

Here's the complete data flow from GPS satellite to the T-Deck screen,
including the optional drone repeater:

```
[GPS satellites]
      │
      ▼
[L76K / M8Q GPS on tracker node]
  UART serial → Meshtastic GPS driver → localPosition struct
      │
      ▼
[TrackerModule.runOnce() fires every 30s]
  wait 15s on first boot (boot delay)
  reads localPosition + QMC5883L compass heading
  packs into meshtastic_Position protobuf
  encrypts with TRACKER PSK (channel 1)
  sends via SX1262 LoRa radio
      │
      │  ~~~ LoRa radio signal ~~~
      │
      │  [Optional: Heltec V4 repeater on drone]
      │    receives packet on TRACKER channel
      │    REPEATER role rebroadcasts at full power
      │    no decoding, no custom module — pure relay
      │
      │  ~~~ LoRa radio signal (now with extended range) ~~~
      │
      ▼
[T-Deck SX1262 radio receives packet]
  Meshtastic mesh stack decrypts with TRACKER PSK
  identifies port as POSITION_APP
  calls TrackerRadarModule::handleReceived()
      │
      ▼
[TrackerRadarModule decodes protobuf]
  stores lat/lon/heading/RSSI/SNR into g_tracker global
  calls updateHistory():
    first packet → sets anchor, marks motion invalid
    subsequent  → Haversine distance from anchor
                  jitter filter (< 15m = stationary)
                  speed = distance / time × MPS_TO_KN
                  COG = bearing from anchor to current
      │
      ▼
[LVGL timer fires every 1s]
  TrackerScreens::onTimer() →
    redrawRadar(): clear canvas, draw rose + rings,
                   scale buoy distance to pixel radius,
                   rotate for track-up display,
                   update bearing/distance/heading labels
    or
    redrawData():  update speed/COG/age/RSSI/SNR labels
      │
      ▼
[T-Deck 320×240 TFT display]
  Compass rose, track-up oriented, with buoy as red dot
  Bearing (°T), distance (m/km), speed (kn), COG (°T/°M)
  Compass heading from tracker, RSSI, SNR, age of data
```

The key design insight: the tracker broadcasts to everyone. The T-Deck picks up whatever
it hears. Multiple T-Deck receivers can watch the same buoy simultaneously — there's no
pairing or handshake. The repeater is also completely transparent: neither the tracker
nor the T-Deck knows or cares whether a relay was involved.

---

## Where to go from here

If you want to extend this project:

**Add a second buoy** — give it a different PSK and display it as a second colour on the
radar. Add a second `g_tracker2` struct, a second packet handler, and a second dot on the canvas.

**Log data to SD card** — the T-Deck has a microSD slot. Open a file in `setup()` and
append a CSV line in `handleReceived()`: timestamp, lat, lon, speed, COG.

**Alert on drift** — in `updateHistory()`, if `speed_kn > 0.5` and motion was previously
zero, trigger the T-Deck's built-in buzzer. The buzzer pin is already driven by Meshtastic
for notifications.

**Add battery reporting to the radar screen** — the T-Beam sends battery telemetry packets
every 5 minutes on the same mesh. Register a second module listening on `TELEMETRY_APP`
and display the tracker's battery percentage alongside the GPS data.

**Improve the anchor model** — instead of anchoring forever, reset the anchor automatically
after a configurable time period (e.g. 1 hour) to track drift from the most recent reference
point rather than initial deployment.

All of these extend from the same pattern: add fields to `g_tracker`, update
`handleReceived()` to populate them, and add labels to the display screens.
