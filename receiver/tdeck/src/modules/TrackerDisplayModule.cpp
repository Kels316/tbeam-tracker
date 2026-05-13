#include "TrackerDisplayModule.h"
#include "NodeDB.h"
#include "configuration.h"
#include <Arduino.h>
#include <math.h>
#include <pb_decode.h>
#include "esp_heap_caps.h"
#include "esp_system.h"

// LVGL — picked up via device-ui include path
#include "lv_conf.h"
#include "lvgl.h"

#include <Wire.h>
#include <Preferences.h>

// ── Constants ─────────────────────────────────────────────────
static constexpr float DEG2RAD   = M_PI / 180.0f;
static constexpr float RAD2DEG   = 180.0f / M_PI;
static constexpr float MPS_TO_KN = 1.94384f;

// ── QMC5883P compass ──────────────────────────────────────────
static constexpr uint8_t QMC_ADDR     = 0x2C;
static constexpr uint8_t QMC_REG_DATA = 0x01;  // X LSB, 6 bytes follow
static constexpr uint8_t QMC_REG_CR1  = 0x0A;  // mode / ODR / OSR
static constexpr uint8_t QMC_REG_CR2  = 0x0B;  // range / set-reset
static constexpr uint8_t QMC_REG_SIGN = 0x29;  // axis sign (undocumented, required)

static float   s_compassHdg    = 0.0f;
static float   s_compassRawHdg = 0.0f;   // atan2 result before north offset
static float   s_northOffset   = 0.0f;   // set via trackball on Cal screen
static bool    s_compassValid  = false;
static bool    s_calReady      = false;   // true once range >= threshold on both axes
static bool    s_calActive     = false;   // used by explicit Cal screen UI only

// Hard-iron bounds — always reset to sentinels at boot so bias is captured fresh each session.
// (axis polarity can vary each boot due to SIGN register uncertainty)
static int16_t s_calXMin =  32767, s_calXMax = -32768;
static int16_t s_calYMin =  32767, s_calYMax = -32768;

static void *radarOrientBtn = nullptr;  // Orient button on radar screen
static void *radarOrientLbl = nullptr;

static void compassInit()
{
    // Wire already initialised by Meshtastic on SDA=18, SCL=8
    // Init sequence per QMC5883P datasheet section 7.2 (continuous mode)
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(QMC_REG_SIGN);
    Wire.write(0x06);  // define sign for X/Y/Z axes
    Wire.endTransmission();

    Wire.beginTransmission(QMC_ADDR);
    Wire.write(QMC_REG_CR2);
    Wire.write(0x08);  // Set/Reset ON, 8 Gauss range
    Wire.endTransmission();

    Wire.beginTransmission(QMC_ADDR);
    Wire.write(QMC_REG_CR1);
    Wire.write(0xE3);  // continuous mode, ODR=100Hz, OSR1=8, OSR2=1
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
        s_compassValid = true;
        LOG_INFO("TrackerScreens: QMC5883P ready at 0x2C\n");
    } else {
        LOG_WARN("TrackerScreens: QMC5883P not found (err=%u)\n", err);
    }
}

// Read one raw sample from the magnetometer. Returns false on I2C error.
static bool compassRawSample(int16_t &x, int16_t &y)
{
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(QMC_REG_DATA);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)6) != 6) return false;
    x = (int16_t)(Wire.read() | (Wire.read() << 8));
    y = (int16_t)(Wire.read() | (Wire.read() << 8));
    Wire.read(); Wire.read(); // discard Z
    return true;
}

// Always-on: accumulates min/max every timer tick to self-calibrate bias during use.
static void compassCalTick()
{
    if (!s_compassValid) return;
    int16_t x, y;
    if (!compassRawSample(x, y)) return;
    if (x < s_calXMin) s_calXMin = x;
    if (x > s_calXMax) s_calXMax = x;
    if (y < s_calYMin) s_calYMin = y;
    if (y > s_calYMax) s_calYMax = y;
    int16_t rangeX = s_calXMax - s_calXMin;
    int16_t rangeY = s_calYMax - s_calYMin;
    s_calReady = (rangeX >= 300 && rangeY >= 300);
}

// Read heading from magnetometer and update s_compassHdg. Returns true if read succeeded.
static bool compassCapture()
{
    if (!s_compassValid) {
        LOG_WARN("compassCapture: sensor not valid\n");
        return false;
    }
    int16_t x, y;
    if (!compassRawSample(x, y)) {
        LOG_WARN("compassCapture: I2C read failed\n");
        return false;
    }
    // Use bias-centred values only once we have enough rotation to trust the min/max.
    // Until then fall back to raw — heading will be wrong but will at least move.
    float cx, cy;
    if (s_calReady) {
        cx = (float)x - (s_calXMax + s_calXMin) * 0.5f;
        cy = (float)y - (s_calYMax + s_calYMin) * 0.5f;
    } else {
        cx = (float)x;
        cy = (float)y;
    }
    float raw = atan2f(-cy, cx) * RAD2DEG;
    while (raw < 0.0f)    raw += 360.0f;
    while (raw >= 360.0f) raw -= 360.0f;
    s_compassRawHdg = raw;
    s_compassHdg    = raw;
    LOG_INFO("compassCapture: rawX=%d rawY=%d cx=%.0f cy=%.0f hdg=%.1f calReady=%d rangeX=%d rangeY=%d\n",
             x, y, cx, cy, raw, (int)s_calReady,
             (int)(s_calXMax - s_calXMin), (int)(s_calYMax - s_calYMin));
    return true;
}

// Canvas dimensions for the radar rose (fits left side of 320×240)
static constexpr int CANVAS_W = 210;
static constexpr int CANVAS_H = 210;
// Rose geometry (within canvas)
static constexpr int ROSE_CX = 105;
static constexpr int ROSE_CY = 110;
static constexpr int ROSE_R  = 98;

// Right-side info panel start x on the full screen
static constexpr int INFO_X  = 215;

// ── Global shared state ────────────────────────────────────────
TrackerData g_tracker;
PositionFix g_anchor;
bool        g_anchorSet = false;

TrackerRadarModule *trackerRadarModule = nullptr;

// ── Static members ─────────────────────────────────────────────
TrackerScreens::Page  TrackerScreens::currentPage  = TrackerScreens::Page::None;
bool                  TrackerScreens::hookDone     = false;
void                 *TrackerScreens::mainScr      = nullptr;
static lv_indev_read_cb_t  s_origEncReadCb          = nullptr;
static lv_indev_t         *s_encIndev               = nullptr;
void                 *TrackerScreens::radarScr     = nullptr;
void                 *TrackerScreens::dataScr      = nullptr;
void                 *TrackerScreens::calScr       = nullptr;
static void          *calStatusLbl                 = nullptr;
static void          *calActionBtn                 = nullptr;  // Recalibrate / Save
static void          *calActionLbl                 = nullptr;  // label on that button
static void          *calExitBtn                   = nullptr;  // Exit / Cancel
void                 *TrackerScreens::radarCanvas  = nullptr;
uint8_t              *TrackerScreens::radarBuf     = nullptr;
void *TrackerScreens::lblLat   = nullptr;
void *TrackerScreens::lblLon   = nullptr;
void *TrackerScreens::lblAlt   = nullptr;
void *TrackerScreens::lblSpd   = nullptr;
void *TrackerScreens::lblCog   = nullptr;
void *TrackerScreens::lblHdg   = nullptr;
void *TrackerScreens::lblSats  = nullptr;
void *TrackerScreens::lblPdop  = nullptr;
void *TrackerScreens::lblRssi  = nullptr;
void *TrackerScreens::lblWin   = nullptr;
void *TrackerScreens::lblAge   = nullptr;
void *TrackerScreens::radarDist  = nullptr;
void *TrackerScreens::radarBrg   = nullptr;
void *TrackerScreens::radarHdg   = nullptr;
void *TrackerScreens::radarSpd   = nullptr;
void *TrackerScreens::radarAge   = nullptr;
void *TrackerScreens::radarTitle = nullptr;

// ═══════════════════════════════════════════════════════════════
// TrackerRadarModule — packet handler
// ═══════════════════════════════════════════════════════════════

TrackerRadarModule::TrackerRadarModule()
    : SinglePortModule("TrackerRadar", meshtastic_PortNum_POSITION_APP)
{
    LOG_INFO("TrackerRadarModule: init\n");
}

ProcessMessage TrackerRadarModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (mp.channel > 1) return ProcessMessage::CONTINUE;

    meshtastic_Position pos = meshtastic_Position_init_default;
    pb_istream_t stream = pb_istream_from_buffer(
        mp.decoded.payload.bytes, mp.decoded.payload.size);

    if (!pb_decode(&stream, &meshtastic_Position_msg, &pos)) {
        LOG_WARN("TrackerRadarModule: decode failed\n");
        return ProcessMessage::CONTINUE;
    }
    if (!pos.has_latitude_i || !pos.has_longitude_i) return ProcessMessage::CONTINUE;

    g_tracker.latitude_i   = pos.latitude_i;
    g_tracker.longitude_i  = pos.longitude_i;
    g_tracker.altitude     = pos.altitude;
    g_tracker.ground_track = pos.ground_track;
    g_tracker.sats_in_view = pos.sats_in_view;
    g_tracker.PDOP         = pos.PDOP;
    g_tracker.time         = pos.time;
    g_tracker.last_rx_ms   = millis();
    g_tracker.rssi         = mp.rx_rssi;
    g_tracker.node_id      = mp.from;
    g_tracker.valid        = true;

    // Pull battery level from NodeDB (populated by T-Beam's telemetry packets)
    meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(mp.from);
    if (node && node->has_device_metrics)
        g_tracker.battery_pct = node->device_metrics.battery_level;

    updateHistory(pos.latitude_i, pos.longitude_i, millis());

    LOG_INFO("TrackerRadarModule: lat=%d lon=%d spd=%.2fkn cog=%.0f win=%u/20\n",
             pos.latitude_i, pos.longitude_i,
             g_tracker.speed_kn, g_tracker.cog_deg, g_tracker.window_fills);

    return ProcessMessage::CONTINUE;
}

void TrackerRadarModule::updateHistory(int32_t lat_i, int32_t lon_i, uint32_t rx_ms)
{
    g_tracker.window_fills++;

    // First fix becomes the anchor — never overwritten.
    if (!g_anchorSet) {
        g_anchor.latitude_i  = lat_i;
        g_anchor.longitude_i = lon_i;
        g_anchor.rx_ms       = rx_ms;
        g_anchor.valid       = true;
        g_anchorSet = true;
        g_tracker.motion_valid = false;
        return;
    }

    float ancLat = g_anchor.latitude_i  / 1e7f;
    float ancLon = g_anchor.longitude_i / 1e7f;
    float nowLat = lat_i / 1e7f;
    float nowLon = lon_i / 1e7f;

    float distM = distanceMetres(ancLat, ancLon, nowLat, nowLon);

    // Below threshold: GPS jitter — report stationary with explicit zeroes.
    if (distM < MIN_MOVE_METRES) {
        g_tracker.speed_kn     = 0.0f;
        g_tracker.cog_deg      = 0.0f;
        g_tracker.motion_valid = true;
        return;
    }

    float dtSec = (float)(rx_ms - g_anchor.rx_ms) / 1000.0f;
    g_tracker.cog_deg      = bearingTo(ancLat, ancLon, nowLat, nowLon);
    g_tracker.speed_kn     = (dtSec > 0.0f) ? (distM / dtSec) * MPS_TO_KN : 0.0f;
    g_tracker.motion_valid = true;
}

float TrackerRadarModule::distanceMetres(float lat1, float lon1, float lat2, float lon2)
{
    float dLat = (lat2 - lat1) * DEG2RAD;
    float dLon = (lon2 - lon1) * DEG2RAD;
    float a = sinf(dLat/2)*sinf(dLat/2) +
              cosf(lat1*DEG2RAD)*cosf(lat2*DEG2RAD)*sinf(dLon/2)*sinf(dLon/2);
    return 6371000.0f * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f-a));
}

float TrackerRadarModule::bearingTo(float lat1, float lon1, float lat2, float lon2)
{
    float dLon = (lon2 - lon1) * DEG2RAD;
    float y = sinf(dLon) * cosf(lat2*DEG2RAD);
    float x = cosf(lat1*DEG2RAD)*sinf(lat2*DEG2RAD) -
               sinf(lat1*DEG2RAD)*cosf(lat2*DEG2RAD)*cosf(dLon);
    return fmodf(atan2f(y, x) * RAD2DEG + 360.0f, 360.0f);
}

// ═══════════════════════════════════════════════════════════════
// TrackerScreens — LVGL screen builder + update
// ═══════════════════════════════════════════════════════════════

// ── Canvas draw helpers ───────────────────────────────────────

void TrackerScreens::canvasLine(void *layerV, int x1, int y1, int x2, int y2,
                                 uint32_t colHex, int w)
{
    lv_layer_t *layer = (lv_layer_t*)layerV;
    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.color   = lv_color_hex(colHex);
    d.width   = w;
    d.opa     = LV_OPA_COVER;
    d.p1.x = x1; d.p1.y = y1;
    d.p2.x = x2; d.p2.y = y2;
    lv_draw_line(layer, &d);
}

void TrackerScreens::canvasCircle(void *layerV, int cx, int cy, int r,
                                   int strokeW, uint32_t colHex)
{
    lv_layer_t *layer = (lv_layer_t*)layerV;
    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.color       = lv_color_hex(colHex);
    d.width       = strokeW;
    d.opa         = LV_OPA_COVER;
    d.center.x    = cx;
    d.center.y    = cy;
    d.radius      = (uint16_t)r;
    d.start_angle = 0;
    d.end_angle   = 360;
    lv_draw_arc(layer, &d);
}

void TrackerScreens::canvasFilledCircle(void *layerV, int cx, int cy, int r,
                                         uint32_t colHex)
{
    lv_layer_t *layer = (lv_layer_t*)layerV;
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = lv_color_hex(colHex);
    d.bg_opa   = LV_OPA_COVER;
    d.radius   = LV_RADIUS_CIRCLE;
    lv_area_t a = { (int32_t)(cx-r), (int32_t)(cy-r),
                    (int32_t)(cx+r), (int32_t)(cy+r) };
    lv_draw_rect(layer, &d, &a);
}

void TrackerScreens::canvasText(void *layerV, int x, int y, int w, int h,
                                 const char *txt, uint32_t colHex)
{
    lv_layer_t *layer = (lv_layer_t*)layerV;
    lv_draw_label_dsc_t d;
    lv_draw_label_dsc_init(&d);
    d.color       = lv_color_hex(colHex);
    d.font        = LV_FONT_DEFAULT;
    d.opa         = LV_OPA_COVER;
    d.align       = LV_TEXT_ALIGN_CENTER;
    d.text        = txt;
    d.text_static = 1;
    lv_area_t a = { (int32_t)x, (int32_t)y,
                    (int32_t)(x+w), (int32_t)(y+h) };
    lv_draw_label(layer, &d, &a);
}

void TrackerScreens::canvasTriangle(void *layerV,
                                     int x0, int y0, int x1, int y1, int x2, int y2,
                                     uint32_t colHex)
{
    lv_layer_t *layer = (lv_layer_t*)layerV;
    lv_draw_triangle_dsc_t d;
    lv_draw_triangle_dsc_init(&d);
    d.color = lv_color_hex(colHex);
    d.opa   = LV_OPA_COVER;
    d.p[0].x = x0; d.p[0].y = y0;
    d.p[1].x = x1; d.p[1].y = y1;
    d.p[2].x = x2; d.p[2].y = y2;
    lv_draw_triangle(layer, &d);
}

// ── Radar drawing primitives ──────────────────────────────────

void TrackerScreens::drawCompassRose(void *layer, int cx, int cy, int r, float rotDeg)
{
    // Outer ring
    canvasCircle(layer, cx, cy, r,     2, 0xFFFFFF);
    canvasCircle(layer, cx, cy, r - 8, 1, 0x888888);

    // Cardinal N/E/S/W labels and tick marks
    const char *cardinals[] = { "N", "E", "S", "W" };
    for (int i = 0; i < 4; i++) {
        float rad = ((float)(i * 90) - rotDeg - 90.0f) * DEG2RAD;
        // Label at 80% radius
        int lx = cx + (int)((r - 18) * cosf(rad));
        int ly = cy + (int)((r - 18) * sinf(rad));
        canvasText(layer, lx - 8, ly - 8, 16, 16, cardinals[i], 0xFFFFFF);
    }

    // 8 tick marks at 45° intervals
    for (int i = 0; i < 8; i++) {
        float rad = ((float)(i * 45) - rotDeg - 90.0f) * DEG2RAD;
        int tickLen = (i % 2 == 0) ? 10 : 6;
        int x1 = cx + (int)((r - tickLen) * cosf(rad));
        int y1 = cy + (int)((r - tickLen) * sinf(rad));
        int x2 = cx + (int)(r * cosf(rad));
        int y2 = cy + (int)(r * sinf(rad));
        canvasLine(layer, x1, y1, x2, y2, 0xAAAAAA);
    }
}

void TrackerScreens::drawBoatIcon(void *layer, int cx, int cy, int sz)
{
    if (sz < 4) sz = 4;
    // Triangle pointing up (own vessel)
    canvasTriangle(layer,
        cx,          cy - sz,
        cx - sz/2,   cy + sz/2,
        cx + sz/2,   cy + sz/2,
        0x00FF88);
}

void TrackerScreens::drawArrow(void *layer, int cx, int cy, int r, float angleDeg)
{
    if (r < 3) r = 3;
    float rad = (angleDeg - 90.0f) * DEG2RAD;
    int tx = cx + (int)(r * cosf(rad));
    int ty = cy + (int)(r * sinf(rad));
    int bx = cx - (int)(r * 0.5f * cosf(rad));
    int by = cy - (int)(r * 0.5f * sinf(rad));
    canvasLine(layer, bx, by, tx, ty, 0xFFFF00, 2);
    // Arrowhead wings
    float w1 = rad + M_PI * 0.75f;
    float w2 = rad - M_PI * 0.75f;
    int hw = r / 3 + 1;
    canvasLine(layer, tx, ty, tx+(int)(hw*cosf(w1)), ty+(int)(hw*sinf(w1)), 0xFFFF00, 2);
    canvasLine(layer, tx, ty, tx+(int)(hw*cosf(w2)), ty+(int)(hw*sinf(w2)), 0xFFFF00, 2);
}

// ── Screen builder ─────────────────────────────────────────────

static lv_obj_t *makeLabel(lv_obj_t *parent, int x, int y, int w, int h,
                            const char *txt, bool large = false)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_obj_set_pos(lbl, x, y);
    lv_obj_set_size(lbl, w, h);
    lv_label_set_text(lbl, txt);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    if (large)
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    else
        lv_obj_set_style_text_font(lbl, LV_FONT_DEFAULT, 0);
    return lbl;
}

void TrackerScreens::buildScreens()
{
    LOG_INFO("TrackerScreens: buildScreens enter, free heap=%u\n", (unsigned)esp_get_free_heap_size());
    // ── Allocate canvas buffer in PSRAM ───────────────────────
    radarBuf = (uint8_t*)heap_caps_malloc(
        CANVAS_W * CANVAS_H * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!radarBuf) {
        LOG_WARN("TrackerScreens: PSRAM alloc failed, trying DRAM\n");
        radarBuf = (uint8_t*)malloc(CANVAS_W * CANVAS_H * 2);
    }

    // ── Radar screen ──────────────────────────────────────────
    lv_obj_t *rs = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(rs, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(rs, LV_OPA_COVER, 0);
    lv_obj_clear_flag(rs, LV_OBJ_FLAG_SCROLLABLE);  // prevent trackball scrolling screen and shifting button positions
    radarScr = rs;

    // Full-screen background tap zone — plain obj (not btn) so it never steals encoder focus
    lv_obj_t *rb = lv_obj_create(rs);
    lv_obj_set_pos(rb, 0, 0);
    lv_obj_set_size(rb, 320, 240);
    lv_obj_set_style_bg_opa(rb, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(rb, 0, 0);
    lv_obj_add_flag(rb, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(rb, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(rb,
        [](lv_event_t *e){ TrackerScreens::enterData(); },
        LV_EVENT_CLICKED, NULL);

    // Canvas on left side
    lv_obj_t *cv = lv_canvas_create(rs);
    lv_obj_set_pos(cv, 0, 15);
    lv_canvas_set_buffer(cv, radarBuf, CANVAS_W, CANVAS_H, LV_COLOR_FORMAT_RGB565);
    radarCanvas = cv;

    // Info labels on right side
    radarTitle = makeLabel(rs, INFO_X, 5,  100, 18, "TRACKER",  false);
    radarDist  = makeLabel(rs, INFO_X, 28, 100, 22, "--",       true);
    radarBrg   = makeLabel(rs, INFO_X, 56, 100, 18, "Brg ---",  false);
    radarHdg   = makeLabel(rs, INFO_X, 78, 100, 18, "Hdg ---",  false);
    radarSpd   = makeLabel(rs, INFO_X,100, 100, 18, "Spd ---",  false);
    radarAge   = makeLabel(rs, INFO_X,122, 100, 18, "--",       false);

    // Orient button — captures heading on tap, locks rose
    lv_obj_t *ob = lv_btn_create(rs);
    lv_obj_set_pos(ob, INFO_X, 148);
    lv_obj_set_size(ob, 105, 50);
    lv_obj_set_style_bg_color(ob, lv_color_hex(0x004488), 0);
    lv_obj_set_style_radius(ob, 4, 0);
    lv_obj_add_event_cb(ob, [](lv_event_t *) {
        bool ok = compassCapture();
        char buf[32];
        // Show heading + raw values for diagnosis; remove rawX/rawY once working
        snprintf(buf, sizeof(buf), ok ? "H%03.0f R%03.0f" : "FAIL %03.0f",
                 s_compassHdg, s_compassRawHdg);
        lv_label_set_text((lv_obj_t*)radarOrientLbl, buf);
        lv_obj_set_style_bg_color((lv_obj_t*)radarOrientBtn,
                                  ok ? lv_color_hex(0x005500) : lv_color_hex(0x880000), 0);
        TrackerScreens::redrawRadar();
    }, LV_EVENT_CLICKED, NULL);
    radarOrientBtn = ob;
    lv_obj_t *ol = lv_label_create(ob);
    lv_label_set_text(ol, "Orient");
    lv_obj_set_style_text_color(ol, lv_color_white(), 0);
    lv_obj_center(ol);
    radarOrientLbl = ol;

    // Hint label bottom-left
    lv_obj_t *hint = makeLabel(rs, 2, 225, 200, 14, "\xE2\x8C\x82 press trackball for Data", false);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);

    // ── Data screen ───────────────────────────────────────────
    lv_obj_t *ds = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(ds, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(ds, LV_OPA_COVER, 0);
    dataScr = ds;

    // Title + divider
    makeLabel(ds, 0, 4, 320, 20, "TRACKER — Data", false);
    lv_obj_t *div = lv_obj_create(ds);
    lv_obj_set_pos(div, 0, 26);
    lv_obj_set_size(div, 320, 2);
    lv_obj_set_style_bg_color(div, lv_color_hex(0x444444), 0);
    lv_obj_set_style_bg_opa(div, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(div, 0, 0);

    // Two-column label grid
    const int CL = 4, CR = 162, RY = 32, RH = 34;
    lblLat  = makeLabel(ds, CL, RY + 0*RH, 155, RH, "Lat --",    false);
    lblLon  = makeLabel(ds, CL, RY + 1*RH, 155, RH, "Lon --",    false);
    lblAlt  = makeLabel(ds, CL, RY + 2*RH, 155, RH, "Alt --",    false);
    lblSpd  = makeLabel(ds, CL, RY + 3*RH, 155, RH, "Spd --",    false);
    lblCog  = makeLabel(ds, CL, RY + 4*RH, 155, RH, "COG --",    false);
    lblHdg  = makeLabel(ds, CR, RY + 0*RH, 155, RH, "Bat --",    false);
    lblSats = makeLabel(ds, CR, RY + 1*RH, 155, RH, "Sats --",   false);
    lblPdop = makeLabel(ds, CR, RY + 2*RH, 155, RH, "PDOP --",   false);
    lblRssi = makeLabel(ds, CR, RY + 3*RH, 155, RH, "RSSI --",   false);
    lblWin  = makeLabel(ds, CR, RY + 4*RH, 155, RH, "Pkts --",   false);
    lblAge  = makeLabel(ds, 240, 222,        80, 14, "--",         false);
    lv_obj_set_style_text_color((lv_obj_t*)lblAge, lv_color_hex(0x888888), 0);

    // Hint label
    lv_obj_t *hint2 = makeLabel(ds, 2, 222, 230, 14, "\xE2\x8C\x82 press trackball for Cal", false);
    lv_obj_set_style_text_color(hint2, lv_color_hex(0x666666), 0);

    // Invisible full-screen button
    lv_obj_t *db = lv_btn_create(ds);
    lv_obj_set_pos(db, 0, 0);
    lv_obj_set_size(db, 320, 240);
    lv_obj_set_style_bg_opa(db, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(db, 0, 0);
    lv_obj_add_event_cb(db,
        [](lv_event_t *e){ TrackerScreens::enterCal(); },
        LV_EVENT_CLICKED, NULL);

    // ── Cal screen ────────────────────────────────────────────
    lv_obj_t *cs = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(cs, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(cs, LV_OPA_COVER, 0);
    calScr = cs;

    makeLabel(cs, 0, 4, 320, 20, "Compass Calibration", false);
    lv_obj_t *cdiv = lv_obj_create(cs);
    lv_obj_set_pos(cdiv, 0, 26);
    lv_obj_set_size(cdiv, 320, 2);
    lv_obj_set_style_bg_color(cdiv, lv_color_hex(0x444444), 0);
    lv_obj_set_style_bg_opa(cdiv, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(cdiv, 0, 0);

    // Status label — updated every second from onTimer
    calStatusLbl = makeLabel(cs, 10, 36, 300, 40, "Checking cal...", false);
    lv_obj_set_style_text_color((lv_obj_t*)calStatusLbl, lv_color_hex(0xFFAA00), 0);

    // Instruction label (shown during active recal)
    makeLabel(cs, 10, 82, 300, 50,
        "Tilt and rotate in all directions.\nTrace a figure-8 in the air.", false);

    // ── Buttons ───────────────────────────────────────────────
    // Exit / Cancel button (left)
    lv_obj_t *exitBtn = lv_btn_create(cs);
    lv_obj_set_pos(exitBtn, 10, 135);
    lv_obj_set_size(exitBtn, 135, 44);
    lv_obj_set_style_bg_color(exitBtn, lv_color_hex(0x333333), 0);
    lv_obj_set_style_border_width(exitBtn, 0, 0);
    lv_obj_set_style_radius(exitBtn, 6, 0);
    lv_obj_t *exitLbl = lv_label_create(exitBtn);
    lv_label_set_text(exitLbl, "Exit");
    lv_obj_center(exitLbl);
    lv_obj_add_event_cb(exitBtn, [](lv_event_t *) {
        s_calActive = false;
        lv_async_call([](void *){ TrackerScreens::exitToMain(); }, NULL);
    }, LV_EVENT_CLICKED, NULL);
    calExitBtn = exitBtn;

    // Recalibrate / Save button (right)
    lv_obj_t *actionBtn = lv_btn_create(cs);
    lv_obj_set_pos(actionBtn, 175, 135);
    lv_obj_set_size(actionBtn, 135, 44);
    lv_obj_set_style_bg_color(actionBtn, lv_color_hex(0x005500), 0);
    lv_obj_set_style_border_width(actionBtn, 0, 0);
    lv_obj_set_style_radius(actionBtn, 6, 0);
    lv_obj_t *actionLbl = lv_label_create(actionBtn);
    lv_label_set_text(actionLbl, "Recalibrate");
    lv_obj_center(actionLbl);
    lv_obj_add_event_cb(actionBtn, [](lv_event_t *) {
        if (!s_calActive) {
            // Start new calibration
            s_calXMin =  32767; s_calXMax = -32768;
            s_calYMin =  32767; s_calYMax = -32768;
            s_calReady  = false;
            s_calActive = true;
            LOG_INFO("TrackerScreens: starting new calibration\n");
        } else if (s_calReady) {
            // Save and exit
            s_calActive = false;
            Preferences prefs;
            prefs.begin("tracker-cal", false);
            prefs.putShort("calXMin",      s_calXMin);
            prefs.putShort("calXMax",      s_calXMax);
            prefs.putShort("calYMin",      s_calYMin);
            prefs.putShort("calYMax",      s_calYMax);
            prefs.putFloat("northOffset",  s_northOffset);
            prefs.end();
            LOG_INFO("TrackerScreens: cal saved xMin=%d xMax=%d yMin=%d yMax=%d\n",
                     s_calXMin, s_calXMax, s_calYMin, s_calYMax);
            lv_async_call([](void *){ TrackerScreens::exitToMain(); }, NULL);
        }
        // If active but not ready yet: ignore tap
    }, LV_EVENT_CLICKED, NULL);
    calActionBtn = actionBtn;
    calActionLbl = actionLbl;

    // Set North button — point device toward magnetic north, then press to lock offset
    lv_obj_t *northBtn = lv_btn_create(cs);
    lv_obj_set_pos(northBtn, 10, 185);
    lv_obj_set_size(northBtn, 300, 32);
    lv_obj_set_style_bg_color(northBtn, lv_color_hex(0x334400), 0);
    lv_obj_set_style_border_width(northBtn, 0, 0);
    lv_obj_set_style_radius(northBtn, 6, 0);
    lv_obj_t *northLbl = lv_label_create(northBtn);
    lv_label_set_text(northLbl, "Point to Magnetic North \xe2\x86\x92 Set North");
    lv_obj_set_style_text_color(northLbl, lv_color_white(), 0);
    lv_obj_center(northLbl);
    lv_obj_add_event_cb(northBtn, [](lv_event_t *) {
        if (!compassCapture()) return;          // get current raw heading
        s_northOffset = -s_compassRawHdg;       // offset so this direction reads 0° (magnetic north)
        while (s_northOffset < -180.0f) s_northOffset += 360.0f;
        while (s_northOffset >  180.0f) s_northOffset -= 360.0f;
        Preferences prefs;
        prefs.begin("tracker-cal", false);
        prefs.putFloat("northOffset", s_northOffset);
        prefs.end();
        LOG_INFO("TrackerScreens: north offset set to %.1f\n", s_northOffset);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t *hint3 = makeLabel(cs, 2, 222, 300, 14, "\xE2\x8C\x82 trackball: exit", false);
    lv_obj_set_style_text_color(hint3, lv_color_hex(0x666666), 0);

    LOG_INFO("TrackerScreens: screens built\n");
}

// ── Public navigation ─────────────────────────────────────────

void TrackerScreens::enterRadar()
{
    if (!mainScr) mainScr = lv_screen_active(); // capture real main screen on first entry
    currentPage = Page::Radar;
    redrawRadar();
    lv_screen_load_anim((lv_obj_t*)radarScr, LV_SCR_LOAD_ANIM_FADE_IN, 200, 0, false);
}

void TrackerScreens::enterData()
{
    currentPage = Page::Data;
    redrawData();
    lv_screen_load_anim((lv_obj_t*)dataScr, LV_SCR_LOAD_ANIM_FADE_IN, 200, 0, false);
}

void TrackerScreens::enterCal()
{
    // Just navigate to the Cal screen — do NOT reset cal data.
    // A new calibration is only started when the user explicitly presses the trackball.
    s_calActive = false;
    currentPage = Page::Cal;
    lv_screen_load_anim((lv_obj_t*)calScr, LV_SCR_LOAD_ANIM_FADE_IN, 200, 0, false);
}

void TrackerScreens::exitToMain()
{
    currentPage = Page::None;
    lv_screen_load_anim((lv_obj_t*)mainScr, LV_SCR_LOAD_ANIM_FADE_IN, 200, 0, false);
}

// ── Radar redraw ──────────────────────────────────────────────

void TrackerScreens::redrawRadar()
{
    if (!radarCanvas || !radarBuf) return;

    lv_obj_t *cv = (lv_obj_t*)radarCanvas;
    lv_canvas_fill_bg(cv, lv_color_black(), LV_OPA_COVER);

    lv_layer_t layer;
    lv_canvas_init_layer(cv, &layer);

    bool ownFix = (localPosition.latitude_i != 0 || localPosition.longitude_i != 0);
    float ownLat = 0, ownLon = 0, ownHdg = 0;
    if (ownFix) {
        ownLat = localPosition.latitude_i  / 1e7f;
        ownLon = localPosition.longitude_i / 1e7f;
        ownHdg = localPosition.ground_track / 100.0f;
    }

    float rotation = s_compassValid ? s_compassHdg : (ownFix ? ownHdg : 0.0f);
    drawCompassRose(&layer, ROSE_CX, ROSE_CY, ROSE_R, rotation);
    drawBoatIcon(&layer, ROSE_CX, ROSE_CY, 10);

    if (g_tracker.valid && ownFix) {
        float buoyLat = g_tracker.latitude_i  / 1e7f;
        float buoyLon = g_tracker.longitude_i / 1e7f;
        float distM   = distanceMetres(ownLat, ownLon, buoyLat, buoyLon);
        float bearDeg = bearingTo(ownLat, ownLon, buoyLat, buoyLon);
        float relBear = fmodf(bearDeg - rotation + 360.0f, 360.0f);

        // Buoy dot on rose
        float   plotRad = (relBear - 90.0f) * DEG2RAD;
        int     inner   = ROSE_R - 12;
        int     bx      = ROSE_CX + (int)(inner * cosf(plotRad));
        int     by      = ROSE_CY + (int)(inner * sinf(plotRad));
        canvasFilledCircle(&layer, bx, by, 6, 0xFF4444);
        canvasLine(&layer, ROSE_CX, ROSE_CY, bx, by, 0xFF4444, 1);

        // Heading arrow on buoy dot
        float buoyRelHead = fmodf(
            (g_tracker.ground_track / 100.0f) - rotation + 360.0f, 360.0f);
        drawArrow(&layer, bx, by, 14, buoyRelHead);

        lv_canvas_finish_layer(cv, &layer);

        // Update info labels
        char buf[32];
        lv_label_set_text((lv_obj_t*)radarDist, formatDistance(distM).c_str());
        snprintf(buf, sizeof(buf), "Brg %03.0f\xC2\xB0T", bearDeg);
        lv_label_set_text((lv_obj_t*)radarBrg, buf);
        snprintf(buf, sizeof(buf), "Hdg %03.0f\xC2\xB0M", g_tracker.ground_track / 100.0f);
        lv_label_set_text((lv_obj_t*)radarHdg, buf);
        if (g_tracker.motion_valid)
            snprintf(buf, sizeof(buf), "Spd %.1fkn", g_tracker.speed_kn);
        else
            snprintf(buf, sizeof(buf), "Spd ---");
        lv_label_set_text((lv_obj_t*)radarSpd, buf);
        lv_label_set_text((lv_obj_t*)radarAge, formatAge(millis()).c_str());

    } else if (!ownFix && g_tracker.valid) {
        lv_canvas_finish_layer(cv, &layer);
        lv_label_set_text((lv_obj_t*)radarDist, "No own GPS");
        lv_label_set_text((lv_obj_t*)radarBrg, "");
        lv_label_set_text((lv_obj_t*)radarHdg, "");
        lv_label_set_text((lv_obj_t*)radarSpd, "");
        lv_label_set_text((lv_obj_t*)radarAge, formatAge(millis()).c_str());
    } else {
        lv_canvas_finish_layer(cv, &layer);
        lv_label_set_text((lv_obj_t*)radarDist, "No buoy signal");
        lv_label_set_text((lv_obj_t*)radarBrg, "");
        lv_label_set_text((lv_obj_t*)radarHdg, "");
        lv_label_set_text((lv_obj_t*)radarSpd, "");
        lv_label_set_text((lv_obj_t*)radarAge, "");
    }
}

// ── Data redraw ───────────────────────────────────────────────

void TrackerScreens::redrawData()
{
    char buf[32];

    // Buoy battery (populated from tracker telemetry packets)
    if (g_tracker.valid && g_tracker.battery_pct > 0) {
        if (g_tracker.battery_pct > 100)
            snprintf(buf, sizeof(buf), "Bat USB/chg");
        else
            snprintf(buf, sizeof(buf), "Bat %u%%", (unsigned)g_tracker.battery_pct);
    } else {
        snprintf(buf, sizeof(buf), "Bat --");
    }
    lv_label_set_text((lv_obj_t*)lblHdg, buf);

    if (!g_tracker.valid) {
        lv_label_set_text((lv_obj_t*)lblLat, "No data yet");
        return;
    }

    snprintf(buf, sizeof(buf), "Lat %.5f", g_tracker.latitude_i  / 1e7f);
    lv_label_set_text((lv_obj_t*)lblLat, buf);
    snprintf(buf, sizeof(buf), "Lon %.5f", g_tracker.longitude_i / 1e7f);
    lv_label_set_text((lv_obj_t*)lblLon, buf);
    snprintf(buf, sizeof(buf), "Alt %dm", (int)g_tracker.altitude);
    lv_label_set_text((lv_obj_t*)lblAlt, buf);
    if (g_tracker.motion_valid) snprintf(buf, sizeof(buf), "Spd %.1fkn", g_tracker.speed_kn);
    else                        snprintf(buf, sizeof(buf), "Spd ---");
    lv_label_set_text((lv_obj_t*)lblSpd, buf);
    if (g_tracker.motion_valid) snprintf(buf, sizeof(buf), "COG %03.0f\xC2\xB0T", g_tracker.cog_deg);
    else                        snprintf(buf, sizeof(buf), "COG ---");
    lv_label_set_text((lv_obj_t*)lblCog, buf);
    // (lblHdg already updated above, before tracker data check)
    snprintf(buf, sizeof(buf), "Sats %u",  (unsigned)g_tracker.sats_in_view);
    lv_label_set_text((lv_obj_t*)lblSats, buf);
    snprintf(buf, sizeof(buf), "PDOP %.1f", g_tracker.PDOP / 100.0f);
    lv_label_set_text((lv_obj_t*)lblPdop, buf);
    snprintf(buf, sizeof(buf), "RSSI %ddBm", (int)g_tracker.rssi);
    lv_label_set_text((lv_obj_t*)lblRssi, buf);
    snprintf(buf, sizeof(buf), "Pkts %lu", (unsigned long)g_tracker.window_fills);
    lv_label_set_text((lv_obj_t*)lblWin, buf);
    lv_label_set_text((lv_obj_t*)lblAge, formatAge(millis()).c_str());
}

// ── LVGL timer ────────────────────────────────────────────────

void TrackerScreens::onTimer(void *timerV)
{
    // Lazily hook the encoder (trackball) indev on first run — we're inside
    // the tft task here, so lv_indev_set_read_cb is safe.
    if (!hookDone) {
        lv_indev_t *indev = lv_indev_get_next(NULL);
        while (indev) {
            if (lv_indev_get_type(indev) == LV_INDEV_TYPE_ENCODER) {
                s_encIndev      = indev;
                s_origEncReadCb = lv_indev_get_read_cb(indev);
                lv_indev_set_read_cb(indev, [](lv_indev_t *dev, lv_indev_data_t *data) {
                    if (s_origEncReadCb) s_origEncReadCb(dev, data);

                    if (data->key == LV_KEY_ENTER && data->state == LV_INDEV_STATE_PRESSED) {
                        data->state = LV_INDEV_STATE_RELEASED;
                        if (TrackerScreens::currentPage == TrackerScreens::Page::None)
                            lv_async_call([](void*){ TrackerScreens::enterRadar(); }, NULL);
                        else if (TrackerScreens::currentPage == TrackerScreens::Page::Radar)
                            lv_async_call([](void*){ TrackerScreens::enterData(); }, NULL);
                        else if (TrackerScreens::currentPage == TrackerScreens::Page::Data)
                            lv_async_call([](void*){ TrackerScreens::enterCal(); }, NULL);
                        else {
                            // Cal page: trackball always exits (buttons handle cal actions)
                            s_calActive = false;
                            lv_async_call([](void*){ TrackerScreens::exitToMain(); }, NULL);
                        }
                    }
                });
                LOG_INFO("TrackerScreens: hooked encoder indev\n");
                hookDone = true;
                break;
            }
            indev = lv_indev_get_next(indev);
        }
    }
    // Fast path: read compass and redraw radar at 10 Hz for smooth rose rotation.
    // Slow path (every 10 ticks = 1 s): update data/cal labels.
    static uint8_t slowTick = 0;
    compassCalTick(); // only does work when s_calActive is true
    if (currentPage == Page::Radar) {
        redrawRadar();
    } else {
        ++slowTick;
        if (slowTick >= 10) {
            slowTick = 0;
            if (currentPage == Page::Data) {
                redrawData();
            } else if (currentPage == Page::Cal && calStatusLbl) {
                char buf[64];
                int16_t rX = s_calXMax - s_calXMin;
                int16_t rY = s_calYMax - s_calYMin;
                if (!s_calActive) {
                    // Status view
                    if (s_calReady)
                        snprintf(buf, sizeof(buf), "Cal OK  rX=%d rY=%d", rX, rY);
                    else
                        snprintf(buf, sizeof(buf), "No calibration saved");
                    if (calActionLbl) lv_label_set_text((lv_obj_t*)calActionLbl, "Recalibrate");
                    if (calActionBtn) lv_obj_set_style_bg_color(
                        (lv_obj_t*)calActionBtn, lv_color_hex(0x005500), 0);
                } else if (!s_calReady) {
                    // Active, not ready
                    snprintf(buf, sizeof(buf), "Rotate...  rX=%d rY=%d", rX, rY);
                    if (calActionLbl) lv_label_set_text((lv_obj_t*)calActionLbl, "Save");
                    if (calActionBtn) lv_obj_set_style_bg_color(
                        (lv_obj_t*)calActionBtn, lv_color_hex(0x333333), 0);  // grayed
                } else {
                    // Active, ready
                    snprintf(buf, sizeof(buf), "READY  rX=%d rY=%d", rX, rY);
                    if (calActionLbl) lv_label_set_text((lv_obj_t*)calActionLbl, "Save");
                    if (calActionBtn) lv_obj_set_style_bg_color(
                        (lv_obj_t*)calActionBtn, lv_color_hex(0x007700), 0);  // bright green
                }
                lv_label_set_text((lv_obj_t*)calStatusLbl, buf);
            }
        }
    }
}

// Called once at tft task startup — runs in correct LVGL context.
// tftSetup.cpp declares extern void trackerRunSetup().
void trackerRunSetup()
{
    LOG_INFO("TrackerScreens: trackerRunSetup\n");
    Preferences prefs;
    prefs.begin("tracker-cal", true);
    // Cal bounds intentionally NOT loaded from Prefs — old stored values were bad.
    // Using hardcoded seed (X centre ≈ -3000, Y centre ≈ +2400) from live measurements.
    // s_calXMin/Max/YMin/Max are already initialised by the static declarations above.
    s_northOffset = prefs.getFloat("northOffset", 0.0f);
    prefs.end();
    {
        int16_t rX = s_calXMax - s_calXMin;
        int16_t rY = s_calYMax - s_calYMin;
        s_calReady = (rX >= 500 && rY >= 500);
        LOG_INFO("TrackerScreens: loaded cal xMin=%d xMax=%d yMin=%d yMax=%d calReady=%d\n",
                 s_calXMin, s_calXMax, s_calYMin, s_calYMax, (int)s_calReady);
    }
    compassInit();
    TrackerScreens::buildScreens();
    lv_timer_create([](lv_timer_t *u){ TrackerScreens::onTimer(u); }, 100, NULL);
    LOG_INFO("TrackerScreens: screens ready\n");
}

// ── Init ──────────────────────────────────────────────────────

void TrackerScreens::init()
{
    // Nothing to do — trackerRunSetup() is called directly by tft_task_handler at startup.
}

// ── Utilities ─────────────────────────────────────────────────

float TrackerScreens::distanceMetres(float lat1, float lon1, float lat2, float lon2)
{
    float dLat = (lat2-lat1)*DEG2RAD, dLon = (lon2-lon1)*DEG2RAD;
    float a = sinf(dLat/2)*sinf(dLat/2) +
              cosf(lat1*DEG2RAD)*cosf(lat2*DEG2RAD)*sinf(dLon/2)*sinf(dLon/2);
    return 6371000.0f * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f-a));
}

float TrackerScreens::bearingTo(float lat1, float lon1, float lat2, float lon2)
{
    float dLon = (lon2-lon1)*DEG2RAD;
    float y = sinf(dLon)*cosf(lat2*DEG2RAD);
    float x = cosf(lat1*DEG2RAD)*sinf(lat2*DEG2RAD) -
               sinf(lat1*DEG2RAD)*cosf(lat2*DEG2RAD)*cosf(dLon);
    return fmodf(atan2f(y,x)*RAD2DEG + 360.0f, 360.0f);
}

String TrackerScreens::formatDistance(float metres)
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

String TrackerScreens::formatAge(uint32_t now_ms)
{
    if (!g_tracker.valid || g_tracker.last_rx_ms == 0) return "no signal";
    uint32_t age_s = (now_ms - g_tracker.last_rx_ms) / 1000;
    char buf[16];
    if      (age_s < 60)   snprintf(buf, sizeof(buf), "%us ago", age_s);
    else if (age_s < 3600) snprintf(buf, sizeof(buf), "%um ago", age_s / 60);
    else                   snprintf(buf, sizeof(buf), "%uh ago", age_s / 3600);
    return String(buf);
}
