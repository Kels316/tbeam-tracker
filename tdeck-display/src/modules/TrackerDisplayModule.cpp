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

static constexpr float DEG2RAD  = M_PI / 180.0f;
static constexpr float RAD2DEG  = 180.0f / M_PI;
static constexpr float MPS_TO_KN = 1.94384f; // metres/sec to knots

// ── Constructor ───────────────────────────────────────────────
TrackerDisplayModule::TrackerDisplayModule()
    : SinglePortModule("TrackerDisplay", meshtastic_PortNum_POSITION_APP)
{
    LOG_INFO("TrackerDisplayModule: initialised, 10min SAR window\n");
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

    updateHistory(pos.latitude_i, pos.longitude_i, millis());

    LOG_INFO("TrackerDisplayModule: fix lat=%d lon=%d spd=%.2fkn cog=%.0f win=%u/20\n",
             pos.latitude_i, pos.longitude_i,
             tracker.speed_kn, tracker.cog_deg, tracker.window_fills);

    if (screen) screen->forceDisplay();
    return ProcessMessage::CONTINUE;
}

// ── 10-minute history + SAR speed/COG ────────────────────────
//
// Mirrors the buoy_map.py approach:
//   - Discard intervals where movement < MIN_MOVE_METRES (GPS drift)
//   - Speed = total valid distance / total elapsed time (Bowditch)
//   - COG   = bearing from oldest fix to newest fix (net drift direction)
//
void TrackerDisplayModule::updateHistory(int32_t lat_i, int32_t lon_i,
                                          uint32_t rx_ms)
{
    // Write new fix into ring buffer
    history[histHead] = { lat_i, lon_i, rx_ms, true };
    histHead = (histHead + 1) % HISTORY_SIZE;
    if (histCount < HISTORY_SIZE) histCount++;

    tracker.window_fills = histCount;

    if (histCount < 2) {
        tracker.motion_valid = false;
        return;
    }

    // Walk buffer chronologically
    uint8_t oldest = (histHead - histCount + HISTORY_SIZE) % HISTORY_SIZE;
    uint8_t newest = (histHead - 1 + HISTORY_SIZE) % HISTORY_SIZE;

    // ── COG: bearing from oldest to newest fix (net drift) ───
    float latOld = history[oldest].latitude_i  / 1e7f;
    float lonOld = history[oldest].longitude_i / 1e7f;
    float latNew = history[newest].latitude_i  / 1e7f;
    float lonNew = history[newest].longitude_i / 1e7f;
    tracker.cog_deg = bearingTo(latOld, lonOld, latNew, lonNew);

    // ── Speed: total valid distance / total valid time ───────
    // Only count intervals where movement >= MIN_MOVE_METRES
    // to mirror the GPS drift filter in buoy_map.py
    float totalDistM = 0.0f;
    float totalTimeSec = 0.0f;

    for (uint8_t i = 0; i < histCount - 1; i++) {
        uint8_t idxA = (oldest + i)     % HISTORY_SIZE;
        uint8_t idxB = (oldest + i + 1) % HISTORY_SIZE;

        if (!history[idxA].valid || !history[idxB].valid) continue;

        float latA = history[idxA].latitude_i  / 1e7f;
        float lonA = history[idxA].longitude_i / 1e7f;
        float latB = history[idxB].latitude_i  / 1e7f;
        float lonB = history[idxB].longitude_i / 1e7f;

        float distM = distanceMetres(latA, lonA, latB, lonB);
        float dtSec = (float)(history[idxB].rx_ms - history[idxA].rx_ms) / 1000.0f;

        // Skip GPS drift (same filter as buoy_map.py MIN_MOVE_METRES)
        if (distM < MIN_MOVE_METRES) continue;
        if (dtSec < 1.0f) continue;

        totalDistM   += distM;
        totalTimeSec += dtSec;
    }

    if (totalTimeSec > 0.0f) {
        tracker.speed_kn     = (totalDistM / totalTimeSec) * MPS_TO_KN;
        tracker.motion_valid = true;
    } else {
        // All intervals were sub-threshold — buoy is stationary
        tracker.speed_kn     = 0.0f;
        tracker.motion_valid = true; // still valid, just not moving
    }
}

// ── Frame dispatcher ──────────────────────────────────────────
void TrackerDisplayModule::drawFrame(OLEDDisplay *display,
                                      OLEDDisplayUiState *state,
                                      int16_t x, int16_t y)
{
    int page = state->currentFrame % 2;
    if (page == 0) drawRadarPage(display, state, x, y);
    else           drawDataPage(display, state, x, y);
}

// ── Radar page ────────────────────────────────────────────────
void TrackerDisplayModule::drawRadarPage(OLEDDisplay *display,
                                          OLEDDisplayUiState *state,
                                          int16_t x, int16_t y)
{
    display->clear();
    display->setFont(ArialMT_Plain_10);

    bool ownFix = false;
    float ownLat = 0, ownLon = 0, ownHeading = 0;
    if (gps && gps->hasLock()) {
        ownLat     = gps->latitude  / 1e7f;
        ownLon     = gps->longitude / 1e7f;
        ownHeading = gps->heading   / 100.0f;
        ownFix     = true;
    }

    const int16_t cx = x + 52;
    const int16_t cy = y + 38;
    const int16_t rOuter = 32;
    const int16_t rInner = 28;

    float rotation = ownFix ? ownHeading : 0.0f;
    drawCompassRose(display, cx, cy, rOuter, rotation);
    drawBoatIcon(display, cx, cy);

    if (tracker.valid && ownFix) {
        float buoyLat = tracker.latitude_i  / 1e7f;
        float buoyLon = tracker.longitude_i / 1e7f;

        float distM   = distanceMetres(ownLat, ownLon, buoyLat, buoyLon);
        float bearDeg = bearingTo(ownLat, ownLon, buoyLat, buoyLon);
        float relBear = bearDeg - rotation;

        float plotRad = (relBear - 90.0f) * DEG2RAD;
        int16_t bx = cx + (int16_t)(rInner * cosf(plotRad));
        int16_t by = cy + (int16_t)(rInner * sinf(plotRad));

        display->fillCircle(bx, by, 3);
        display->drawLine(cx, cy, bx, by);

        float buoyRelHead = (tracker.ground_track / 100.0f) - rotation;
        drawArrow(display, bx, by, 5, buoyRelHead);

        display->setTextAlignment(TEXT_ALIGN_LEFT);
        display->setFont(ArialMT_Plain_16);
        display->drawString(x + 90, y + 8, formatDistance(distM));
        display->setFont(ArialMT_Plain_10);

        char buf[20];
        snprintf(buf, sizeof(buf), "Brg %03.0f%cTrue", bearDeg, (char)176);
        display->drawString(x + 90, y + 28, buf);

        snprintf(buf, sizeof(buf), "Hdg %03.0f%cMag",
                 tracker.ground_track / 100.0f, (char)176);
        display->drawString(x + 90, y + 40, buf);

        // Speed on radar page too
        if (tracker.motion_valid) {
            snprintf(buf, sizeof(buf), "Spd %.1fkn", tracker.speed_kn);
            display->drawString(x + 90, y + 52, buf);
        }

        display->setTextAlignment(TEXT_ALIGN_RIGHT);
        display->drawString(x + 126, y + 64, formatAge(millis()));

    } else if (tracker.valid && !ownFix) {
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
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->drawString(x + 64, y + 36, "No buoy signal");
    }

    if (ownFix) {
        display->setFont(ArialMT_Plain_10);
        display->setTextAlignment(TEXT_ALIGN_RIGHT);
        char hdgBuf[12];
        snprintf(hdgBuf, sizeof(hdgBuf), "%03.0f%cM", ownHeading, (char)176);
        display->drawString(x + 126, y + 0, hdgBuf);
    }

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

    float lat        = tracker.latitude_i  / 1e7f;
    float lon        = tracker.longitude_i / 1e7f;
    float compassHdg = tracker.ground_track / 100.0f;
    float pdop       = tracker.PDOP / 100.0f;

    char buf[48];
    const int rowH = 11;
    int rowL = y + 15; // left column y
    int rowR = y + 15; // right column y

    // ── Left column ───────────────────────────────────────────
    snprintf(buf, sizeof(buf), "Lat: %.5f", lat);
    display->drawString(x + 2, rowL, buf); rowL += rowH;

    snprintf(buf, sizeof(buf), "Lon: %.5f", lon);
    display->drawString(x + 2, rowL, buf); rowL += rowH;

    snprintf(buf, sizeof(buf), "Alt: %dm", (int)tracker.altitude);
    display->drawString(x + 2, rowL, buf); rowL += rowH;

    // Speed — show 0.0 if stationary, --- if no data yet
    if (tracker.motion_valid)
        snprintf(buf, sizeof(buf), "Spd: %.1fkn", tracker.speed_kn);
    else
        snprintf(buf, sizeof(buf), "Spd: ---");
    display->drawString(x + 2, rowL, buf); rowL += rowH;

    // COG — net drift bearing over 10-min window
    if (tracker.motion_valid)
        snprintf(buf, sizeof(buf), "COG: %.0f%cT", tracker.cog_deg, (char)176);
    else
        snprintf(buf, sizeof(buf), "COG: --- T");
    display->drawString(x + 2, rowL, buf);

    // ── Right column ──────────────────────────────────────────
    display->setTextAlignment(TEXT_ALIGN_RIGHT);

    snprintf(buf, sizeof(buf), "Hdg:%.0fM", compassHdg);
    display->drawString(x + 126, rowR, buf); rowR += rowH;

    snprintf(buf, sizeof(buf), "Sats:%u", (unsigned)tracker.sats_in_view);
    display->drawString(x + 126, rowR, buf); rowR += rowH;

    snprintf(buf, sizeof(buf), "PDOP:%.1f", pdop);
    display->drawString(x + 126, rowR, buf); rowR += rowH;

    snprintf(buf, sizeof(buf), "RSSI:%ddBm", (int)tracker.rssi);
    display->drawString(x + 126, rowR, buf); rowR += rowH;

    // Window fill indicator — shows SAR data maturity
    // e.g. "Win: 8/20" means 4 of 10 minutes accumulated
    snprintf(buf, sizeof(buf), "Win:%u/20", tracker.window_fills);
    display->drawString(x + 126, rowR, buf);

    // Age bottom left
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->drawString(x + 2, y + 64, formatAge(millis()));
}

// ── Compass rose ──────────────────────────────────────────────
void TrackerDisplayModule::drawCompassRose(OLEDDisplay *display,
                                            int16_t cx, int16_t cy,
                                            int16_t r, float rotationDeg)
{
    display->drawCircle(cx, cy, r);
    display->drawCircle(cx, cy, r - 4);

    const char *cardinals[] = { "N", "E", "S", "W" };
    for (int i = 0; i < 4; i++) {
        float angleRad = ((float)(i * 90) - rotationDeg - 90.0f) * DEG2RAD;
        int16_t lx = cx + (int16_t)((r - 9) * cosf(angleRad));
        int16_t ly = cy + (int16_t)((r - 9) * sinf(angleRad));
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->setFont(ArialMT_Plain_10);
        display->drawString(lx, ly - 4, cardinals[i]);
    }
    for (int i = 0; i < 8; i++) {
        float angleRad = ((float)(i * 45) - rotationDeg - 90.0f) * DEG2RAD;
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
    display->drawLine(bx, by, tx, ty);
    display->drawLine(tx, ty, tx + (int16_t)(3*cosf(w1r)), ty + (int16_t)(3*sinf(w1r)));
    display->drawLine(tx, ty, tx + (int16_t)(3*cosf(w2r)), ty + (int16_t)(3*sinf(w2r)));
}

// ── Haversine ─────────────────────────────────────────────────
float TrackerDisplayModule::distanceMetres(float lat1, float lon1,
                                            float lat2, float lon2)
{
    const float R = 6371000.0f;
    float dLat = (lat2 - lat1) * DEG2RAD;
    float dLon = (lon2 - lon1) * DEG2RAD;
    float a = sinf(dLat/2)*sinf(dLat/2) +
              cosf(lat1*DEG2RAD)*cosf(lat2*DEG2RAD)*
              sinf(dLon/2)*sinf(dLon/2);
    return R * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
}

// ── True bearing ──────────────────────────────────────────────
float TrackerDisplayModule::bearingTo(float lat1, float lon1,
                                       float lat2, float lon2)
{
    float dLon = (lon2 - lon1) * DEG2RAD;
    float y = sinf(dLon) * cosf(lat2 * DEG2RAD);
    float x = cosf(lat1*DEG2RAD)*sinf(lat2*DEG2RAD) -
               sinf(lat1*DEG2RAD)*cosf(lat2*DEG2RAD)*cosf(dLon);
    return fmodf(atan2f(y, x) * RAD2DEG + 360.0f, 360.0f);
}

// ── Distance formatter ────────────────────────────────────────
String TrackerDisplayModule::formatDistance(float metres)
{
    char buf[16];
    if (metres < 1852.0f)
        snprintf(buf, sizeof(buf), "%dm", (int)metres);
    else if (metres / 1852.0f < 10.0f)
        snprintf(buf, sizeof(buf), "%.2fnm", metres / 1852.0f);
    else
        snprintf(buf, sizeof(buf), "%.1fnm", metres / 1852.0f);
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
