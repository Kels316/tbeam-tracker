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
 *       Full numeric readout: lat, lon, alt, heading, sats,
 *       PDOP, RSSI, time since last packet.
 *
 * Target : Meshtastic firmware 2.7.x
 * Hardware: LilyGo T-Deck (ESP32-S3, ILI9341 320x240, SX1262, GPS)
 */

// ── Tracker buoy data ─────────────────────────────────────────
struct TrackerData {
    int32_t  latitude_i   = 0;   // degrees * 1e7
    int32_t  longitude_i  = 0;   // degrees * 1e7
    int32_t  altitude     = 0;   // metres
    uint32_t ground_track = 0;   // degrees * 100  (buoy heading)
    uint32_t sats_in_view = 0;
    uint32_t PDOP         = 0;
    uint32_t time         = 0;   // unix timestamp of GPS fix
    uint32_t last_rx_ms   = 0;   // millis() when last packet received
    int8_t   rssi         = 0;
    uint32_t node_id      = 0;
    bool     valid        = false;
};

class TrackerDisplayModule : public SinglePortModule
{
  public:
    TrackerDisplayModule();

    // Meshtastic screen integration
    bool wantUIFrame() override { return true; }
    void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state,
                   int16_t x, int16_t y) override;
    int getNumExtraFrames() override { return 2; }

  protected:
    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

  private:
    // Pages
    void drawRadarPage(OLEDDisplay *display, OLEDDisplayUiState *state,
                       int16_t x, int16_t y);
    void drawDataPage(OLEDDisplay *display, OLEDDisplayUiState *state,
                      int16_t x, int16_t y);

    // Drawing helpers
    void drawCompassRose(OLEDDisplay *display, int16_t cx, int16_t cy,
                         int16_t r, float rotationDeg);
    void drawArrow(OLEDDisplay *display, int16_t cx, int16_t cy,
                   int16_t r, float angleDeg);
    void drawBoatIcon(OLEDDisplay *display, int16_t cx, int16_t cy);

    // Navigation math
    // Returns bearing in degrees true (0-360) from point 1 to point 2
    float bearingTo(float lat1, float lon1, float lat2, float lon2);

    // Returns distance in metres between two lat/lon points (Haversine)
    float distanceMetres(float lat1, float lon1, float lat2, float lon2);

    // Format distance: metres if < 1852, nautical miles if >=
    String formatDistance(float metres);

    // Format age since last_rx_ms
    String formatAge(uint32_t now_ms);

    TrackerData tracker;
};

extern TrackerDisplayModule *trackerDisplayModule;
