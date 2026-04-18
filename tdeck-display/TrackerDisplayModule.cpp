#include "TrackerDisplayModule.h"
#include "GPS.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "channels.h"
#include "configuration.h"
#include "graphics/Screen.h"
#include <Arduino.h>
#include <math.h>
#include <pb_decode.h>

TrackerDisplayModule *trackerDisplayModule;

static constexpr float DEG2RAD = M_PI / 180.0f;
static constexpr float RAD2DEG = 180.0f / M_PI;

// ── Constructor ───────────────────────────────────────────────
TrackerDisplayModule::TrackerDisplayModule()
    : SinglePortModule("TrackerDisplay", meshtastic_PortNum_POSITION_APP)
{
    LOG_INFO("TrackerDisplayModule: initialised\n");
}

// ── Packet handler ────────────────────────────────────────────
ProcessMessage TrackerDisplayModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (mp.channel != 0) return ProcessMessage::CONTINUE;

    meshtastic_Position pos = meshtastic_Position_init_default;
    pb_istream_t stream = pb_istream_from_buffer(
        mp.decoded.payload.bytes, mp.decoded.payload.size);

    if (!pb_decode(&stream, &meshtastic_Position_msg, &pos)) {
        LOG_WARN("TrackerDisplayModule: failed to decode position\n");
        return ProcessMessage::CONTINUE;
    }

    tracker.latitude_i   = pos.latitude_i;
    tracker.longitude_i  = pos.longitude_i;
    tracker.altitude     = pos.altitude;
    tracker.ground_track = pos.ground_track;
    tracker.sats_in_view = pos.sats_in_view;
    tracker.PDOP         = pos.PDOP;
    tracker.time         = pos.time;
    tracker.last_rx_ms   = millis();
    tracker.rssi         = mp.rx_rssi;
    tracker.node_id      = mp.from;
    tracker.valid        = true;

    LOG_INFO("TrackerDisplayModule: buoy fix lat=%d lon=%d track=%u\n",
             pos.latitude_i, pos.longitude_i, pos.ground_track);

    if (screen) screen->forceDisplay();

    return ProcessMessage::CONTINUE;
}

// ── Frame dispatcher ──────────────────────────────────────────
void TrackerDisplayModule::drawFrame(OLEDDisplay *display,
                                     OLEDDisplayUiState *state,
                                     int16_t x, int16_t y)
{
    int page = state->currentFrame % 2;
    if (page == 0)
        drawRadarPage(display, state, x, y);
    else
        drawDataPage(display, state, x, y);
}

// ── Radar page ────────────────────────────────────────────────
void TrackerDisplayModule::drawRadarPage(OLEDDisplay *display,
                                          OLEDDisplayUiState *state,
                                          int16_t x, int16_t y)
{
    display->clear();
    display->setFont(ArialMT_Plain_10);

    // ── Get our own position from T-Deck GPS ──────────────────
    bool ownFix = false;
    float ownLat = 0, ownLon = 0, ownHeading = 0;

    if (gps && gps->hasLock()) {
        ownLat     = gps->latitude  / 1e7f;
        ownLon     = gps->longitude / 1e7f;
        ownHeading = gps->heading   / 100.0f; // heading stored as deg*100
        ownFix     = true;
    }

    // ── Compass rose setup ────────────────────────────────────
    // Rose centred in left portion of screen, leaving right margin for text
    const int16_t cx = x + 52;
    const int16_t cy = y + 38;
    const int16_t rOuter = 32;
    const int16_t rInner = 28;

    // Rotation: track-up if we have a fix, north-up if not
    float rotation = ownFix ? ownHeading : 0.0f;

    drawCompassRose(display, cx, cy, rOuter, rotation);

    // ── Draw own vessel at centre ─────────────────────────────
    drawBoatIcon(display, cx, cy);

    // ── Plot tracker buoy if we have data ─────────────────────
    if (tracker.valid && ownFix) {
        float buoyLat = tracker.latitude_i  / 1e7f;
        float buoyLon = tracker.longitude_i / 1e7f;

        float distM   = distanceMetres(ownLat, ownLon, buoyLat, buoyLon);
        float bearDeg = bearingTo(ownLat, ownLon, buoyLat, buoyLon);

        // Relative bearing on the rose (subtract our heading for track-up)
        float relBear = bearDeg - rotation;

        // Plot buoy dot on the rose edge
        float plotRad = (relBear - 90.0f) * DEG2RAD;
        int16_t bx = cx + (int16_t)(rInner * cosf(plotRad));
        int16_t by = cy + (int16_t)(rInner * sinf(plotRad));

        // Buoy dot
        display->fillCircle(bx, by, 3);

        // Line from centre to buoy
        display->drawLine(cx, cy, bx, by);

        // Small heading arrow on the buoy dot showing buoy orientation
        float buoyHeading = tracker.ground_track / 100.0f;
        float buoyRelHead = buoyHeading - rotation;
        drawArrow(display, bx, by, 5, buoyRelHead);

        // ── Right panel: distance and bearing ─────────────────
        display->setTextAlignment(TEXT_ALIGN_LEFT);

        // Distance — large and prominent
        display->setFont(ArialMT_Plain_16);
        display->drawString(x + 90, y + 10, formatDistance(distM));

        display->setFont(ArialMT_Plain_10);

        // Bearing
        char bearBuf[16];
        snprintf(bearBuf, sizeof(bearBuf), "Brg %03.0f%c", bearDeg, (char)176);
        display->drawString(x + 90, y + 30, bearBuf);

        // Buoy heading
        char hdgBuf[16];
        float buoyHdgTrue = tracker.ground_track / 100.0f;
        snprintf(hdgBuf, sizeof(hdgBuf), "Hdg %03.0f%c", buoyHdgTrue, (char)176);
        display->drawString(x + 90, y + 42, hdgBuf);

        // Age
        display->drawString(x + 90, y + 54, formatAge(millis()));

    } else if (tracker.valid && !ownFix) {
        // Have buoy data but no own fix — show absolute position
        display->setTextAlignment(TEXT_ALIGN_LEFT);
        display->setFont(ArialMT_Plain_10);
        display->drawString(x + 90, y + 10, "No own GPS");
        char latBuf[20], lonBuf[20];
        snprintf(latBuf, sizeof(latBuf), "%.4f", tracker.latitude_i / 1e7f);
        snprintf(lonBuf, sizeof(lonBuf), "%.4f", tracker.longitude_i / 1e7f);
        display->drawString(x + 90, y + 24, latBuf);
        display->drawString(x + 90, y + 36, lonBuf);
        display->drawString(x + 90, y + 48, formatAge(millis()));

    } else {
        // No buoy data at all
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->drawString(x + 64, y + 52, "No buoy signal");
    }

    // ── Own heading indicator top right ───────────────────────
    if (ownFix) {
        display->setFont(ArialMT_Plain_10);
        display->setTextAlignment(TEXT_ALIGN_RIGHT);
        char ownHdgBuf[12];
        snprintf(ownHdgBuf, sizeof(ownHdgBuf), "%03.0f%c", ownHeading, (char)176);
        display->drawString(x + 126, y + 0, ownHdgBuf);
    }

    // ── Page label ────────────────────────────────────────────
    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->drawString(x + 2, y + 0, "TRACKER");
}

// ── Data page ─────────────────────────────────────────────────
void TrackerDisplayModule::drawDataPage(OLEDDisplay *display,
                                         OLEDDisplayUiState *state,
                                         int16_t x, int16_t y)
{
    display->clear();
    display->setFont(ArialMT_Plain_10);

    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(x + 64, y + 0, "TRACKER — Data");
    display->drawLine(x + 0, y + 12, x + 128, y + 12);

    if (!tracker.valid) {
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->drawString(x + 64, y + 30, "No data yet");
        return;
    }

    display->setTextAlignment(TEXT_ALIGN_LEFT);

    float lat     = tracker.latitude_i  / 1e7f;
    float lon     = tracker.longitude_i / 1e7f;
    float heading = tracker.ground_track / 100.0f;
    float pdop    = tracker.PDOP / 100.0f;

    char buf[48];
    int row = y + 15;
    const int rowH = 11;

    snprintf(buf, sizeof(buf), "Lat:  %.5f", lat);
    display->drawString(x + 2, row, buf); row += rowH;

    snprintf(buf, sizeof(buf), "Lon:  %.5f", lon);
    display->drawString(x + 2, row, buf); row += rowH;

    snprintf(buf, sizeof(buf), "Alt:  %dm", (int)tracker.altitude);
    display->drawString(x + 2, row, buf);

    // Right column
    row = y + 15;
    display->setTextAlignment(TEXT_ALIGN_RIGHT);

    snprintf(buf, sizeof(buf), "Hdg:%.0f", heading);
    display->drawString(x + 126, row, buf); row += rowH;

    snprintf(buf, sizeof(buf), "Sats:%u", (unsigned)tracker.sats_in_view);
    display->drawString(x + 126, row, buf); row += rowH;

    snprintf(buf, sizeof(buf), "PDOP:%.1f", pdop);
    display->drawString(x + 126, row, buf); row += rowH;

    display->setTextAlignment(TEXT_ALIGN_LEFT);
    snprintf(buf, sizeof(buf), "RSSI:%ddBm", (int)tracker.rssi);
    display->drawString(x + 2, row, buf);

    display->setTextAlignment(TEXT_ALIGN_RIGHT);
    display->drawString(x + 126, row, formatAge(millis()));
}

// ── Compass rose ──────────────────────────────────────────────
// Draws a compass rose centred at (cx,cy) with outer radius r.
// rotationDeg rotates the rose so that heading is at top (track-up).
void TrackerDisplayModule::drawCompassRose(OLEDDisplay *display,
                                            int16_t cx, int16_t cy,
                                            int16_t r, float rotationDeg)
{
    // Outer circle
    display->drawCircle(cx, cy, r);

    // Inner tick circle
    display->drawCircle(cx, cy, r - 4);

    // Cardinal point labels N S E W
    const char *cardinals[] = { "N", "E", "S", "W" };
    for (int i = 0; i < 4; i++) {
        float angleDeg = (float)(i * 90) - rotationDeg;
        float angleRad = (angleDeg - 90.0f) * DEG2RAD;
        int16_t lx = cx + (int16_t)((r - 9) * cosf(angleRad));
        int16_t ly = cy + (int16_t)((r - 9) * sinf(angleRad));
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->setFont(ArialMT_Plain_10);
        display->drawString(lx, ly - 4, cardinals[i]);
    }

    // Tick marks every 45 degrees
    for (int i = 0; i < 8; i++) {
        float angleDeg = (float)(i * 45) - rotationDeg;
        float angleRad = (angleDeg - 90.0f) * DEG2RAD;
        int16_t x1 = cx + (int16_t)((r - 4) * cosf(angleRad));
        int16_t y1 = cy + (int16_t)((r - 4) * sinf(angleRad));
        int16_t x2 = cx + (int16_t)(r * cosf(angleRad));
        int16_t y2 = cy + (int16_t)(r * sinf(angleRad));
        display->drawLine(x1, y1, x2, y2);
    }
}

// ── Boat icon ─────────────────────────────────────────────────
void TrackerDisplayModule::drawBoatIcon(OLEDDisplay *display,
                                         int16_t cx, int16_t cy)
{
    // Simple triangle pointing up (north/heading direction)
    display->drawLine(cx,     cy - 4, cx - 3, cy + 3);
    display->drawLine(cx,     cy - 4, cx + 3, cy + 3);
    display->drawLine(cx - 3, cy + 3, cx + 3, cy + 3);
}

// ── Heading arrow ─────────────────────────────────────────────
void TrackerDisplayModule::drawArrow(OLEDDisplay *display,
                                      int16_t cx, int16_t cy,
                                      int16_t r, float angleDeg)
{
    float rad = (angleDeg - 90.0f) * DEG2RAD;

    int16_t tx = cx + (int16_t)(r * cosf(rad));
    int16_t ty = cy + (int16_t)(r * sinf(rad));
    int16_t bx = cx - (int16_t)((r * 0.6f) * cosf(rad));
    int16_t by = cy - (int16_t)((r * 0.6f) * sinf(rad));

    float w1r = rad + M_PI * 0.75f;
    float w2r = rad - M_PI * 0.75f;
    int16_t w1x = tx + (int16_t)(3 * cosf(w1r));
    int16_t w1y = ty + (int16_t)(3 * sinf(w1r));
    int16_t w2x = tx + (int16_t)(3 * cosf(w2r));
    int16_t w2y = ty + (int16_t)(3 * sinf(w2r));

    display->drawLine(bx, by, tx, ty);
    display->drawLine(tx, ty, w1x, w1y);
    display->drawLine(tx, ty, w2x, w2y);
}

// ── Haversine distance (metres) ───────────────────────────────
float TrackerDisplayModule::distanceMetres(float lat1, float lon1,
                                            float lat2, float lon2)
{
    const float R = 6371000.0f; // Earth radius metres
    float dLat = (lat2 - lat1) * DEG2RAD;
    float dLon = (lon2 - lon1) * DEG2RAD;
    float a = sinf(dLat / 2) * sinf(dLat / 2) +
              cosf(lat1 * DEG2RAD) * cosf(lat2 * DEG2RAD) *
              sinf(dLon / 2) * sinf(dLon / 2);
    return R * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
}

// ── True bearing from point 1 to point 2 ─────────────────────
float TrackerDisplayModule::bearingTo(float lat1, float lon1,
                                       float lat2, float lon2)
{
    float dLon = (lon2 - lon1) * DEG2RAD;
    float y = sinf(dLon) * cosf(lat2 * DEG2RAD);
    float x = cosf(lat1 * DEG2RAD) * sinf(lat2 * DEG2RAD) -
               sinf(lat1 * DEG2RAD) * cosf(lat2 * DEG2RAD) * cosf(dLon);
    float bearing = atan2f(y, x) * RAD2DEG;
    return fmodf(bearing + 360.0f, 360.0f);
}

// ── Distance formatter ────────────────────────────────────────
String TrackerDisplayModule::formatDistance(float metres)
{
    char buf[16];
    if (metres < 1852.0f) {
        snprintf(buf, sizeof(buf), "%dm", (int)metres);
    } else {
        float nm = metres / 1852.0f;
        if (nm < 10.0f)
            snprintf(buf, sizeof(buf), "%.2fnm", nm);
        else
            snprintf(buf, sizeof(buf), "%.1fnm", nm);
    }
    return String(buf);
}

// ── Age formatter ─────────────────────────────────────────────
String TrackerDisplayModule::formatAge(uint32_t now_ms)
{
    if (!tracker.valid || tracker.last_rx_ms == 0) return "--";
    uint32_t age_s = (now_ms - tracker.last_rx_ms) / 1000;
    char buf[12];
    if (age_s < 60)
        snprintf(buf, sizeof(buf), "%us ago", age_s);
    else if (age_s < 3600)
        snprintf(buf, sizeof(buf), "%um ago", age_s / 60);
    else
        snprintf(buf, sizeof(buf), "%uh ago", age_s / 3600);
    return String(buf);
}
