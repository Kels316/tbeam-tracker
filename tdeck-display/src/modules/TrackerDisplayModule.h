#pragma once
#include "SinglePortModule.h"
#include "mesh/generated/meshtastic/mesh.pb.h"

/**
 * TrackerDisplayModule
 *
 * Meshtastic screen module for the LilyGo T-Deck.
 * Adds two extra pages to the existing T-Deck screen cycle:
 *
 *   Page 1 — Radar view
 *       Compass rose, track-up (your heading at top).
 *       Your vessel fixed at centre.
 *       Tracker buoy plotted at correct relative bearing and range.
 *       Buoy heading arrow shows which way the buoy is pointing.
 *       Distance in metres (<1852m) or nautical miles (>=1852m).
 *       Bearing to buoy in degrees true.
 *       Falls back to north-up if T-Deck has no GPS fix.
 *
 *   Page 2 — Data view
 *       Full numeric readout: lat, lon, alt, compass heading, sats,
 *       PDOP, RSSI, age, calculated speed (kn), COG, and window
 *       fill progress for SAR set/drift calculations.
 *
 * Speed and COG calculation:
 *   - Rolling 10-minute window (20 fixes at 30s interval)
 *   - GPS drift filter: intervals where movement < MIN_MOVE_METRES
 *     are discarded (same approach as buoy_map.py RPi tracker)
 *   - Speed = total distance / total elapsed time over window
 *   - COG  = bearing from oldest fix to newest fix (Bowditch method,
 *     gives net drift direction for SAR set/drift calculation)
 *   - Window fill shown as X/20 so operator knows data maturity
 *
 * Target : Meshtastic firmware 2.7.x
 * Hardware: LilyGo T-Deck (ESP32-S3, ILI9341 320x240, SX1262, GPS)
 */

// ── Position history ring buffer ──────────────────────────────
// 20 entries = 10 minutes at one fix per 30 seconds
static constexpr uint8_t  HISTORY_SIZE    = 20;
static constexpr float    MIN_MOVE_METRES = 3.0f;  // GPS drift filter

struct PositionFix {
    int32_t  latitude_i  = 0;
    int32_t  longitude_i = 0;
    uint32_t rx_ms       = 0;
    bool     valid       = false;
};

// ── Tracker buoy data ─────────────────────────────────────────
struct TrackerData {
    int32_t  latitude_i   = 0;
    int32_t  longitude_i  = 0;
    int32_t  altitude     = 0;
    uint32_t ground_track = 0;   // compass heading degrees * 100
    uint32_t sats_in_view = 0;
    uint32_t PDOP         = 0;
    uint32_t time         = 0;
    uint32_t last_rx_ms   = 0;
    int8_t   rssi         = 0;
    uint32_t node_id      = 0;
    bool     valid        = false;

    // Calculated from 10-minute position window
    float    speed_kn     = 0.0f;  // speed over ground, knots
    float    cog_deg      = 0.0f;  // course over ground (net drift bearing)
    uint8_t  window_fills = 0;     // how many of 20 slots are populated
    bool     motion_valid = false;
};

class TrackerDisplayModule : public SinglePortModule
{
  public:
    TrackerDisplayModule();

    bool wantUIFrame() override { return true; }
    void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state,
                   int16_t x, int16_t y) override;
    int getNumExtraFrames() override { return 2; }

  protected:
    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

  private:
    void drawRadarPage(OLEDDisplay *display, OLEDDisplayUiState *state,
                       int16_t x, int16_t y);
    void drawDataPage(OLEDDisplay *display, OLEDDisplayUiState *state,
                      int16_t x, int16_t y);
    void drawCompassRose(OLEDDisplay *display, int16_t cx, int16_t cy,
                         int16_t r, float rotationDeg);
    void drawArrow(OLEDDisplay *display, int16_t cx, int16_t cy,
                   int16_t r, float angleDeg);
    void drawBoatIcon(OLEDDisplay *display, int16_t cx, int16_t cy);

    float   bearingTo(float lat1, float lon1, float lat2, float lon2);
    float   distanceMetres(float lat1, float lon1, float lat2, float lon2);
    void    updateHistory(int32_t lat_i, int32_t lon_i, uint32_t rx_ms);
    String  formatDistance(float metres);
    String  formatAge(uint32_t now_ms);

    TrackerData tracker;
    PositionFix history[HISTORY_SIZE];
    uint8_t     histHead  = 0;
    uint8_t     histCount = 0;
};

extern TrackerDisplayModule *trackerDisplayModule;
