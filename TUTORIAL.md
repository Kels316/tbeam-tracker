# How the Buoy Tracker Works — A Code Walkthrough

This project uses two LilyGo devices to track a GPS buoy at sea in real time.
The **T-Beam** sits in the buoy and transmits its position over LoRa radio every 30 seconds.
The **T-Deck** is a handheld receiver with a screen — it picks up those transmissions and shows
you where the buoy is, how far away it is, and how fast it's drifting.

Both devices run a customised version of [Meshtastic](https://meshtastic.org/), which is open-source
firmware for LoRa mesh radio devices. We add our own code on top.

---

## Contents

1. [Concepts you'll need](#concepts-youll-need)
2. [The secret channel — tracker_channel.h](#the-secret-channel)
3. [T-Beam: The recovery strobe — StrobeModule](#t-beam-the-recovery-strobe)
4. [T-Beam: GPS + compass + radio — TrackerModule](#t-beam-gps--compass--radio)
5. [T-Deck: Receiving packets — TrackerRadarModule](#t-deck-receiving-packets)
6. [T-Deck: Calculating speed and drift](#t-deck-calculating-speed-and-drift)
7. [T-Deck: Building the screens — TrackerScreens](#t-deck-building-the-screens)
8. [T-Deck: Making the trackball work](#t-deck-making-the-trackball-work)
9. [T-Deck: Drawing the radar and data views](#t-deck-drawing-the-radar-and-data-views)
10. [How it all fits together](#how-it-all-fits-together)

---

## Concepts you'll need

Before diving in, here are a few ideas that come up constantly.

**LoRa radio** is a low-power radio that can send short messages kilometres over open water.
It's not fast (no video or audio), but it's perfect for sending a GPS position once every 30 seconds.

**Meshtastic** is firmware that handles all the hard radio work for us — sending, receiving,
retrying, mesh routing. We just add modules that plug into it.

**A module** in Meshtastic is a C++ class that gets called automatically when something happens —
like a timer firing, or a packet arriving. You don't write a `main()` loop; you register a module
and Meshtastic calls it.

**Protobuf** (Protocol Buffers) is how Meshtastic packages data into a radio packet. A GPS position
is wrapped up as a small binary message and unwrapped on the other end.

**LVGL** is a graphics library for small screens. On the T-Deck it draws labels, canvases,
and handles touch/trackball input. Think of it as the "UI framework" for embedded devices.

**I2C** is a wiring standard for connecting chips together with just two wires (SDA and SCL).
Our compass sensor uses I2C to talk to the ESP32.

---

## The secret channel

**File:** `nodes/tbeam/tracker_channel.h`

The buoy transmits on a private channel called TRACKER with a secret encryption key (PSK).
Anyone with an off-the-shelf Meshtastic device won't be able to read our packets —
they'd need the same key.

This header file sets up the channel and saves it to flash memory the first time the T-Beam boots.

```cpp
// Only run setup if we haven't already configured this channel
if (strcmp(channels.getByIndex(1).settings.name, TRACKER_CHANNEL_NAME) == 0) {
    LOG_INFO("setupTrackerChannel: already configured, skipping\n");
    return;
}
```

`channels.getByIndex(1)` means "look at channel slot 1" (slot 0 is the public default channel).
`strcmp` compares two strings — if the name is already "TRACKER", we've already set this up before
and can skip. This stops the T-Beam reconfiguring itself on every boot.

```cpp
static const uint8_t TRACKER_PSK[32] = {
    0x4a, 0x3f, 0x8c, 0x21, ...
};
```

This is the 256-bit encryption key. Every byte is written in hexadecimal (that's what `0x` means).
Both the T-Beam and T-Deck must have the exact same key, otherwise the T-Deck can't decrypt
the packets and will never see any buoy data.

```cpp
meshtastic_Channel ch = meshtastic_Channel_init_default;
ch.settings = cs;
ch.role     = meshtastic_Channel_Role_SECONDARY;
ch.index    = 1;
channels.setChannel(ch);
channels.onConfigChanged(); // save to flash
```

This creates a `Channel` object, sets it to role SECONDARY (slot 1, not the primary public channel),
and calls `onConfigChanged()` to write it to flash memory so it survives a power cycle.

---

## T-Beam: The recovery strobe

**Files:** `nodes/tbeam/src/modules/StrobeModule.h` and `StrobeModule.cpp`

The buoy has a bright white LED strobe driven through a MOSFET transistor on GPIO 13.
The MOSFET acts like a switch — the ESP32 can't supply enough current to drive a bright LED directly,
so we use a tiny signal from GPIO 13 to switch a transistor that handles the real current.

### The header file defines the timing

```cpp
static constexpr uint8_t  STROBE_PIN       = 13;
static constexpr uint32_t BLINK_ON_MS      = 80;   // flash duration
static constexpr uint32_t BLINK_GAP_MS     = 150;  // gap between blinks in a burst
static constexpr uint32_t INTERVAL_LOCK_MS   = 30000; // 30 s when GPS locked
static constexpr uint32_t INTERVAL_NOLOCK_MS =  5000; //  5 s when no fix
```

`constexpr` means these values are fixed at compile time — they're baked into the firmware.
There's no "magic number" scattered through the code; everything is named and in one place.

### The module runs on a timer

```cpp
StrobeModule::StrobeModule()
    : concurrency::OSThread("StrobeModule")
{
    pinMode(STROBE_PIN, OUTPUT);
    digitalWrite(STROBE_PIN, LOW);
}
```

`concurrency::OSThread` is a Meshtastic class that runs our code on a repeating timer.
`pinMode(STROBE_PIN, OUTPUT)` tells the ESP32 that GPIO 13 is an output pin.
`digitalWrite(STROBE_PIN, LOW)` makes sure the LED is off when the board first boots.

```cpp
int32_t StrobeModule::runOnce()
{
    bool hasLock = (gps && gps->hasLock());
    doBlinks(3);
    return hasLock ? INTERVAL_LOCK_MS : INTERVAL_NOLOCK_MS;
}
```

`runOnce()` is called by Meshtastic automatically. The return value tells Meshtastic how long
to wait before calling it again (in milliseconds). So if the GPS has a lock, we wait 30 seconds;
if not, we flash every 5 seconds so a SAR team on a nearby vessel can see it's searching for a fix.

```cpp
void StrobeModule::doBlinks(uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        digitalWrite(STROBE_PIN, HIGH);   // LED on
        delay(BLINK_ON_MS);               // wait 80 ms
        digitalWrite(STROBE_PIN, LOW);    // LED off
        if (i < count - 1)
            delay(BLINK_GAP_MS);          // gap between flashes
    }
}
```

A simple loop: turn the LED on, wait, turn it off, wait (except after the last blink).
Three calls to `digitalWrite` produce three flashes. The `if (i < count - 1)` check avoids
adding a trailing gap after the last blink in the burst.

---

## T-Beam: GPS + compass + radio

**Files:** `nodes/tbeam/src/modules/TrackerModule.h` and `TrackerModule.cpp`

This is the heart of the buoy. Every 30 seconds it:
1. Reads the GPS position from Meshtastic's internal `localPosition` object
2. Reads a heading from the compass chip
3. Packs everything into a position packet
4. Broadcasts it on the TRACKER channel

### Inheriting from two classes at once

```cpp
class TrackerModule : public SinglePortModule, private concurrency::OSThread
```

In C++ a class can inherit from more than one parent. `SinglePortModule` gives us the ability
to send Meshtastic packets. `OSThread` gives us the 30-second timer. We're combining both.

### Setting up the compass over I2C

The QMC5883L compass chip lives on the I2C bus at address `0x0D`.

```cpp
Wire.beginTransmission(QMC_ADDR);
Wire.write(QMC_REG_RESET);
Wire.write(0x01);
Wire.endTransmission();
delay(10);
```

`Wire` is the Arduino I2C library. This sends a reset command to the compass chip —
like pressing a reset button on it. We wait 10 ms for it to restart.

```cpp
Wire.beginTransmission(QMC_ADDR);
Wire.write(QMC_REG_CTRL1);
Wire.write(QMC_CTRL1_VAL);
if (Wire.endTransmission() == 0) {
    compassOk = true;
}
```

Now we configure the chip: continuous measurement mode, 200 measurements per second,
8 gauss range. If `endTransmission()` returns `0`, the chip acknowledged our command
and everything is working. We store `compassOk = true` so later code knows whether
to trust compass readings.

### Reading the heading

```cpp
Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)6);
int16_t x = (int16_t)((Wire.read()) | (Wire.read() << 8));
int16_t y = (int16_t)((Wire.read()) | (Wire.read() << 8));
```

The compass sends back 6 bytes: two bytes each for X, Y, and Z magnetic field strength.
Each pair of bytes forms a 16-bit signed integer. `Wire.read() << 8` shifts the high byte
into the upper half of the 16-bit value, then `|` (bitwise OR) combines it with the low byte.
This is called "little-endian byte order" — low byte first.

```cpp
float heading = atan2f((float)y, (float)x) * (180.0f / M_PI);
heading += MAG_DECLINATION;
if (heading < 0.0f)   heading += 360.0f;
if (heading >= 360.0f) heading -= 360.0f;
```

`atan2f(y, x)` converts the X/Y magnetic field components into an angle in radians.
Multiplying by `(180 / π)` converts radians to degrees. We then add magnetic declination —
in south-east Queensland, true north is about 11.5° east of magnetic north, so we add 11.5°.
Finally we wrap the result to the 0–359 range.

### Building and sending the position packet

```cpp
meshtastic_Position pos = meshtastic_Position_init_default;
pos.has_latitude_i  = true;
pos.latitude_i      = localPosition.latitude_i;
pos.has_longitude_i = true;
pos.longitude_i     = localPosition.longitude_i;
```

Meshtastic uses "protobuf" to structure data. Every field has a matching `has_X` boolean
that you must set to `true`, otherwise the receiver ignores the field. Latitude and longitude
are stored as integers (degrees × 10,000,000) to avoid floating point rounding errors over the radio.

```cpp
pos.ground_track     = (uint32_t)(heading * 100.0f);
pos.has_ground_track = true;
```

We reuse the `ground_track` field (normally used for GPS course-over-ground) to carry
the compass heading. We multiply by 100 to preserve one decimal place as an integer —
so 157.3° becomes 15730.

```cpp
p->decoded.payload.size =
    pb_encode_to_bytes(p->decoded.payload.bytes,
                       sizeof(p->decoded.payload.bytes),
                       &meshtastic_Position_msg, &pos);
p->channel  = 1;
p->to       = NODENUM_BROADCAST;
service->sendToMesh(p, RX_SRC_LOCAL, true);
```

`pb_encode_to_bytes` packs the position struct into compact binary protobuf format.
`p->channel = 1` sends it on our private TRACKER channel (not the public channel 0).
`NODENUM_BROADCAST` means "send to everyone" rather than a specific device.
`sendToMesh` hands the packet to Meshtastic's radio stack.

---

## T-Deck: Receiving packets

**Files:** `receiver/tdeck/src/modules/TrackerDisplayModule.h` and `TrackerDisplayModule.cpp`

On the T-Deck, `TrackerRadarModule` inherits from `SinglePortModule` and listens for packets
on the `POSITION_APP` port. When one arrives, it decodes it and stores the data.

```cpp
TrackerRadarModule::TrackerRadarModule()
    : SinglePortModule("TrackerRadar", meshtastic_PortNum_POSITION_APP)
```

By passing `POSITION_APP` to the parent constructor, we're telling Meshtastic:
"call my `handleReceived()` method whenever a position packet arrives."

### Filtering packets

```cpp
ProcessMessage TrackerRadarModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (mp.channel > 1) return ProcessMessage::CONTINUE;
```

We only care about packets on channel 0 or 1. Packets from other channels are ignored
by returning `CONTINUE`, which tells Meshtastic "I didn't handle this, pass it on."

### Decoding the protobuf

```cpp
meshtastic_Position pos = meshtastic_Position_init_default;
pb_istream_t stream = pb_istream_from_buffer(
    mp.decoded.payload.bytes, mp.decoded.payload.size);

if (!pb_decode(&stream, &meshtastic_Position_msg, &pos)) {
    return ProcessMessage::CONTINUE;
}
```

`pb_istream_from_buffer` creates a "stream" pointing at the raw bytes in the packet.
`pb_decode` reads those bytes and fills in our `pos` struct. If decoding fails
(corrupted packet, wrong format) we skip it.

### Storing the data

```cpp
g_tracker.latitude_i   = pos.latitude_i;
g_tracker.longitude_i  = pos.longitude_i;
g_tracker.ground_track = pos.ground_track;
g_tracker.last_rx_ms   = millis();
g_tracker.rssi         = mp.rx_rssi;
g_tracker.valid        = true;
```

`g_tracker` is a global struct shared between the packet handler and the display code.
`millis()` returns the number of milliseconds since boot — we store this so we can later
calculate "how old is this data?" `mp.rx_rssi` is the signal strength of the received packet
(RSSI — Received Signal Strength Indicator), useful for knowing if the buoy is near or far.

---

## T-Deck: Calculating speed and drift

This is the most interesting bit of maths in the project.

### The anchor model

Instead of averaging over a rolling window of recent packets, we use an **anchor model**:
the very first packet received is stored as the anchor — it never changes.
Every subsequent packet is compared against the anchor to give us:

- **Set** — the direction the buoy has drifted (COG, course over ground)
- **Drift** — the speed it's moving (in knots)

This is exactly what Search and Rescue teams want: net displacement from a known start point.

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

On the very first call, we save the position as the anchor and return early —
there's nothing to compare yet, so we mark motion as invalid (display will show `---`).

### Haversine distance

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

The Earth is a sphere, so you can't just subtract latitudes and longitudes.
The **Haversine formula** gives the straight-line distance between two GPS points on a sphere.
`6371000` is the Earth's radius in metres. The result comes back in metres.

### The jitter filter

GPS chips are never perfectly still — even a stationary receiver jitters by a few metres.
Without a filter, this jitter would produce a fake speed reading of 0.1–0.5 knots.

```cpp
float distM = distanceMetres(ancLat, ancLon, nowLat, nowLon);

if (distM < MIN_MOVE_METRES) {  // MIN_MOVE_METRES = 15.0
    g_tracker.speed_kn     = 0.0f;
    g_tracker.cog_deg      = 0.0f;
    g_tracker.motion_valid = true;
    return;
}
```

If the buoy hasn't moved more than 15 metres from its anchor, we call it stationary
and report exactly zero speed. We still set `motion_valid = true` so the display shows
`0.0kn` and `000°T` rather than `---`. This makes it obvious the buoy is stationary
rather than "data not available."

### Speed and bearing

```cpp
float dtSec = (float)(rx_ms - g_anchor.rx_ms) / 1000.0f;
g_tracker.cog_deg  = bearingTo(ancLat, ancLon, nowLat, nowLon);
g_tracker.speed_kn = (dtSec > 0.0f) ? (distM / dtSec) * MPS_TO_KN : 0.0f;
```

Speed = distance ÷ time. `distM` is metres, `dtSec` is seconds, so `distM / dtSec`
gives metres per second. Multiplying by `MPS_TO_KN` (1.94384) converts to knots.

`bearingTo` uses `atan2f` (the two-argument arctangent) to calculate the compass bearing
from the anchor to the current position — this is the direction the buoy has drifted.

---

## T-Deck: Building the screens

The T-Deck uses LVGL, a graphics library for embedded devices. Screens in LVGL are
plain objects (`lv_obj_t`) that aren't attached to the display until you load them.

### The radar screen

```cpp
lv_obj_t *rs = lv_obj_create(NULL);
lv_obj_set_style_bg_color(rs, lv_color_black(), 0);
radarScr = rs;
```

`lv_obj_create(NULL)` creates a new screen (passing `NULL` as parent means "no parent — this is a root screen").
We store the pointer in `radarScr` so we can switch to it later.

```cpp
lv_obj_t *cv = lv_canvas_create(rs);
lv_canvas_set_buffer(cv, radarBuf, CANVAS_W, CANVAS_H, LV_COLOR_FORMAT_RGB565);
```

A canvas is a raw pixel buffer we can draw on freely. `radarBuf` is a block of memory
(allocated in PSRAM — the T-Deck's extra RAM chip) that holds the pixel data.
`LV_COLOR_FORMAT_RGB565` means each pixel takes 2 bytes: 5 bits red, 6 bits green, 5 bits blue.

### Labels

```cpp
static lv_obj_t *makeLabel(lv_obj_t *parent, int x, int y, int w, int h,
                            const char *txt, bool large)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_obj_set_pos(lbl, x, y);
    lv_obj_set_size(lbl, w, h);
    lv_label_set_text(lbl, txt);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    return lbl;
}
```

A helper function that creates a text label at a specific pixel position.
`lv_label_set_text` sets the initial string. Later, `lv_label_set_text` can update it
at any time and LVGL will redraw automatically.

### Invisible button for trackball press

```cpp
lv_obj_t *rb = lv_btn_create(rs);
lv_obj_set_size(rb, 320, 240);   // full screen size
lv_obj_set_style_bg_opa(rb, LV_OPA_TRANSP, 0);  // invisible
lv_obj_add_event_cb(rb,
    [](lv_event_t *e){ TrackerScreens::enterData(); },
    LV_EVENT_CLICKED, NULL);
```

The entire radar screen has an invisible button the same size as the display.
When the trackball is pressed (which triggers a "click"), this button fires and calls `enterData()`.
This is simpler than handling raw input events and works reliably with LVGL's input system.

---

## T-Deck: Making the trackball work

The T-Deck's trackball is treated by LVGL as an "encoder" input device.
We hook into its read callback to intercept presses.

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
                ...
            }
        });
        break;
    }
    indev = lv_indev_get_next(indev);
}
```

We loop through all LVGL input devices until we find the encoder (trackball).
We save the original read callback, then replace it with our own. Our callback calls
the original first (so the trackball still works normally), then checks if the press key
was `LV_KEY_ENTER`. If so, we navigate to the next screen.

`lv_async_call` is important — it defers the screen change until LVGL is done processing
the current input event. Switching screens inside an input callback could crash LVGL.

### Why we capture the main screen lazily

```cpp
void TrackerScreens::enterRadar()
{
    if (!mainScr) mainScr = lv_screen_active();
    ...
}
```

When our setup code runs, device-ui's main screen hasn't been created yet.
So instead of capturing it at startup, we grab it the first time the user navigates away —
at that moment, the main screen is guaranteed to be the active one.

---

## T-Deck: Drawing the radar and data views

### The compass rose

```cpp
void TrackerScreens::drawCompassRose(void *layer, int cx, int cy, int r, float rotDeg)
{
    canvasCircle(layer, cx, cy, r,     2, 0xFFFFFF);
    canvasCircle(layer, cx, cy, r - 8, 1, 0x888888);

    const char *cardinals[] = { "N", "E", "S", "W" };
    for (int i = 0; i < 4; i++) {
        float rad = ((float)(i * 90) - rotDeg - 90.0f) * DEG2RAD;
        int lx = cx + (int)((r - 18) * cosf(rad));
        int ly = cy + (int)((r - 18) * sinf(rad));
        canvasText(layer, lx - 8, ly - 8, 16, 16, cardinals[i], 0xFFFFFF);
    }
}
```

`rotDeg` is our vessel's heading. By rotating all the labels by `-rotDeg`, the rose stays
"track up" — our heading always points to the top of the screen, just like a real nautical plotter.

For each cardinal direction (N, E, S, W) we use `cosf` and `sinf` to calculate where on
the circle edge to place the label. The formula `cx + r * cos(angle)` converts a circle angle
to an X pixel position — this is basic trigonometry.

### Plotting the buoy on the rose

```cpp
float bearDeg = bearingTo(ownLat, ownLon, buoyLat, buoyLon);
float relBear = fmodf(bearDeg - rotation + 360.0f, 360.0f);

float plotRad = (relBear - 90.0f) * DEG2RAD;
int bx = ROSE_CX + (int)(inner * cosf(plotRad));
int by = ROSE_CY + (int)(inner * sinf(plotRad));
canvasFilledCircle(&layer, bx, by, 6, 0xFF4444);
```

`bearDeg` is the absolute bearing to the buoy (e.g. 045° = northeast).
`relBear` subtracts our own heading to make it relative — so if the buoy is dead ahead,
`relBear` is 0° regardless of which direction we're facing.
The `- 90.0f` offset converts from compass convention (0° = north = up)
to screen/maths convention (0° = right). The red dot is then placed at the right pixel.

### Updating labels

```cpp
char buf[32];
snprintf(buf, sizeof(buf), "Brg %03.0f\xC2\xB0T", bearDeg);
lv_label_set_text((lv_obj_t*)radarBrg, buf);
```

`snprintf` formats a string safely into `buf`. `%03.0f` means: floating point, 0 decimal places,
pad to 3 digits with leading zeros. `\xC2\xB0` is the UTF-8 encoding of the degree symbol °.
`lv_label_set_text` then pushes the new string to the screen.

---

## How it all fits together

Here's the complete data flow from GPS satellite to your screen:

```
[GPS satellites]
      │
      ▼
[u-blox M8Q on T-Beam]
  UART serial → Meshtastic GPS driver → localPosition struct
      │
      ▼
[TrackerModule.runOnce() fires every 30s]
  reads localPosition + QMC5883L heading
  packs into meshtastic_Position protobuf
  encrypts with TRACKER PSK
  sends via LoRa radio
      │
      │  ~~~ LoRa radio ~~~
      │
      ▼
[T-Deck radio receives packet]
  Meshtastic decrypts with TRACKER PSK
  calls TrackerRadarModule::handleReceived()
      │
      ▼
[TrackerRadarModule decodes protobuf]
  stores into g_tracker global struct
  calls updateHistory() to compute speed + drift
      │
      ▼
[LVGL timer fires every 1s]
  TrackerScreens::onTimer() →
    redrawRadar() or redrawData()
    updates canvas pixels + label text
      │
      ▼
[T-Deck screen]
  Compass rose with buoy position
  Bearing, distance, speed, drift
```

The key insight is that the two devices never talk directly to each other — the T-Beam
just broadcasts to everyone, and the T-Deck picks it up. This means you could have
multiple T-Deck receivers watching the same buoy simultaneously, which is useful
in a large SAR operation.

---

## Where to go from here

If you want to extend this project, here are some ideas to try:

- **Add a second buoy** — give it a different node ID and a second colour on the radar
- **Log data to SD card** — the T-Deck has an SD slot; write a CSV of positions every packet
- **Add a battery voltage readout** — the T-Beam sends battery level in telemetry packets
- **Alert when the buoy moves** — buzz the T-Deck speaker when drift exceeds 0.5 knots

All of these can be done by adding more fields to `g_tracker`, updating `handleReceived()`,
and adding new labels to the data screen.
