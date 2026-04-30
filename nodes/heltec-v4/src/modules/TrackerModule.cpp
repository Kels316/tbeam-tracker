#include "src/modules/TrackerModule.h"
#include "GPS.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"
#include "configuration.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

TrackerModule *trackerModule;

// ── QMC5883L register map ────────────────────────────────────────────
static constexpr uint8_t QMC_REG_DATA   = 0x00; // X_LSB .. Z_MSB (6 bytes)
static constexpr uint8_t QMC_REG_STATUS = 0x06;
static constexpr uint8_t QMC_REG_CTRL1  = 0x09;
static constexpr uint8_t QMC_REG_RESET  = 0x0B;

// CTRL1: continuous mode, 200 Hz ODR, 8G range, 512 OSR
static constexpr uint8_t QMC_CTRL1_VAL  = 0b00001101;

TrackerModule::TrackerModule()
    : SinglePortModule("tracker", meshtastic_PortNum_POSITION_APP),
      concurrency::OSThread("TrackerModule")
{
    // ── Power LED: GPIO 2 — always on ───────────────────────────────
    pinMode(POWER_LED_PIN, OUTPUT);
    digitalWrite(POWER_LED_PIN, HIGH);

    // ── TX LED: GPIO 4 — off until packet sent ──────────────────────
    pinMode(TX_LED_PIN, OUTPUT);
    digitalWrite(TX_LED_PIN, LOW);

    // ── Initialise I2C for QMC5883L ─────────────────────────────────
    // Heltec V4: share the OLED I2C bus (SDA=17, SCL=18).
    // OLED is at 0x3C, QMC5883L is at 0x0D — no conflict.
    // GPIO 13 is LORA_BUSY — never use for I2C or any other purpose.
    Wire.begin(17, 18);

    // Soft-reset the QMC5883L
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(QMC_REG_RESET);
    Wire.write(0x01);
    Wire.endTransmission();
    delay(10);

    // Configure: continuous measurement, 200 Hz, 8G, OSR 512
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(QMC_REG_CTRL1);
    Wire.write(QMC_CTRL1_VAL);
    if (Wire.endTransmission() == 0) {
        compassOk = true;
        LOG_INFO("TrackerModule: QMC5883L compass online (SDA=17, SCL=18)\n");
    } else {
        LOG_WARN("TrackerModule: QMC5883L not found on I2C — heading will be omitted\n");
    }

    LOG_INFO("TrackerModule: power LED GPIO %d, TX LED GPIO %d\n",
             POWER_LED_PIN, TX_LED_PIN);
}

int32_t TrackerModule::runOnce()
{
    if (firstRun) {
        if (millis() < BOOT_DELAY_MS)
            return BOOT_POLL_MS; // come back in 500ms — service not ready yet
        firstRun = false;
        sendHello();
    }
    sendPosition();
    return INTERVAL_MS;
}

// ── Hello packet (boot announcement on channel 1) ────────────────────
void TrackerModule::sendHello()
{
    static const char msg[] = "TRACKER ONLINE";
    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p) return;
    p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p->decoded.portnum       = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->decoded.payload.size  = strlen(msg);
    memcpy(p->decoded.payload.bytes, msg, p->decoded.payload.size);
    p->to       = NODENUM_BROADCAST;
    p->channel  = 1;
    p->want_ack = false;
    service->sendToMesh(p, RX_SRC_LOCAL, true);
    LOG_INFO("TrackerModule: sent hello on channel 1\n");
    flashTxLed();
}

// ── QMC5883L heading read ────────────────────────────────────────────
float TrackerModule::readHeadingDeg()
{
    if (!compassOk) return -1.0f;

    // Wait for data-ready (DRDY bit 0 of status register)
    uint8_t status = 0;
    for (int i = 0; i < 20; i++) {
        Wire.beginTransmission(QMC_ADDR);
        Wire.write(QMC_REG_STATUS);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)1);
        if (Wire.available()) status = Wire.read();
        if (status & 0x01) break;
        delay(5);
    }
    if (!(status & 0x01)) {
        LOG_WARN("TrackerModule: QMC5883L data not ready\n");
        return -1.0f;
    }

    // Read 6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(QMC_REG_DATA);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)6);

    if (Wire.available() < 6) {
        LOG_WARN("TrackerModule: QMC5883L short read\n");
        return -1.0f;
    }

    int16_t x = (int16_t)((Wire.read()) | (Wire.read() << 8));
    int16_t y = (int16_t)((Wire.read()) | (Wire.read() << 8));
    /* int16_t z = */ (void)((Wire.read()) | (Wire.read() << 8)); // unused

    // atan2 gives heading in radians; convert to degrees
    float heading = atan2f((float)y, (float)x) * (180.0f / M_PI);

    // Apply magnetic declination correction
    heading += MAG_DECLINATION;

    // Normalise to 0–359
    if (heading < 0.0f)    heading += 360.0f;
    if (heading >= 360.0f) heading -= 360.0f;

    return heading;
}

// ── Position packet ──────────────────────────────────────────────────
void TrackerModule::sendPosition()
{
    if (localPosition.latitude_i == 0 && localPosition.longitude_i == 0) {
        LOG_WARN("TrackerModule: no GPS lock, skipping send\n");
        return;
    }

    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p) {
        LOG_ERROR("TrackerModule: failed to allocate packet\n");
        return;
    }

    // Build position payload
    meshtastic_Position pos = meshtastic_Position_init_default;
    pos.has_latitude_i      = true;
    pos.latitude_i          = localPosition.latitude_i;
    pos.has_longitude_i     = true;
    pos.longitude_i         = localPosition.longitude_i;
    pos.has_altitude        = true;
    pos.altitude            = localPosition.altitude;
    pos.time                = localPosition.time;
    pos.PDOP                = localPosition.PDOP;
    pos.sats_in_view        = localPosition.sats_in_view;

    // Compass heading → ground_track field (0–359 degrees * 100 for fixed-point)
    float heading = readHeadingDeg();
    if (heading >= 0.0f) {
        pos.ground_track = (uint32_t)(heading * 100.0f); // e.g. 157.3° → 15730
        pos.has_ground_track = true;
        LOG_INFO("TrackerModule: heading %.1f deg\n", heading);
    } else {
        LOG_WARN("TrackerModule: heading unavailable, field omitted\n");
    }

    // Encode protobuf into packet
    p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p->decoded.portnum       = meshtastic_PortNum_POSITION_APP;
    p->decoded.payload.size  =
        pb_encode_to_bytes(p->decoded.payload.bytes,
                           sizeof(p->decoded.payload.bytes),
                           &meshtastic_Position_msg, &pos);

    // Broadcast on private channel 1 (TRACKER)
    p->to       = NODENUM_BROADCAST;
    p->channel  = 1;
    p->want_ack = false;

    service->sendToMesh(p, RX_SRC_LOCAL, true);

    LOG_INFO("TrackerModule: packet sent (lat=%d, lon=%d, track=%u)\n",
             pos.latitude_i, pos.longitude_i, pos.ground_track);

    flashTxLed();
}

void TrackerModule::flashTxLed()
{
    digitalWrite(TX_LED_PIN, HIGH);
    delay(TX_LED_MS);
    digitalWrite(TX_LED_PIN, LOW);
}
