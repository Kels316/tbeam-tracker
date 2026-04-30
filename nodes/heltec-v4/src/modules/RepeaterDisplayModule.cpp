#include "configuration.h"
#ifdef USE_REPEATER_MODULE

#include "src/modules/RepeaterDisplayModule.h"
#include "NodeDB.h"
#include "airtime.h"
#include "main.h"
#include "power.h"

RepeaterDisplayModule *repeaterDisplayModule;

RepeaterDisplayModule::RepeaterDisplayModule()
    : MeshModule("RepeaterDisplay")
{
}

#if HAS_SCREEN
#include "graphics/ScreenFonts.h"
#include "graphics/SharedUIDisplay.h"

void RepeaterDisplayModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state,
                                      int16_t x, int16_t y)
{
    display->clear();
    display->setFont(FONT_SMALL);
    display->setTextAlignment(TEXT_ALIGN_LEFT);

    // Header bar: "REPEATER" title + battery icon drawn by drawCommonHeader
    graphics::drawCommonHeader(display, x, y, "REPEATER");

    char buf[32];
    const int leftX = x + 2;

    // Line 1: Battery % and voltage
    uint8_t batPct = powerStatus ? powerStatus->getBatteryChargePercent() : 0;
    float   batV   = powerStatus ? powerStatus->getBatteryVoltageMv() / 1000.0f : 0.0f;
    bool    usb    = powerStatus && powerStatus->getHasUSB();
    if (usb)
        snprintf(buf, sizeof(buf), "Batt: USB  %.2fV", batV);
    else
        snprintf(buf, sizeof(buf), "Batt: %d%%  %.2fV", batPct, batV);
    display->drawString(leftX, y + textFirstLine, buf);

    // Line 2: Uptime (h m s)
    uint32_t secs = millis() / 1000;
    uint32_t h    = secs / 3600; secs %= 3600;
    uint32_t m    = secs / 60;   secs %= 60;
    snprintf(buf, sizeof(buf), "Up: %uh %02um %02us", h, m, secs);
    display->drawString(leftX, y + textSecondLine, buf);

    // Line 3: Channel util / TX air util
    float chUtil = airTime ? airTime->channelUtilizationPercent() : 0.0f;
    float txUtil = airTime ? airTime->utilizationTXPercent()      : 0.0f;
    snprintf(buf, sizeof(buf), "Ch: %.1f%%  TX: %.1f%%", chUtil, txUtil);
    display->drawString(leftX, y + textThirdLine, buf);

    // Line 4: Node ID (short hex)
    uint32_t nodeNum = nodeDB->getNodeNum();
    snprintf(buf, sizeof(buf), "ID: !%08x", nodeNum);
    display->drawString(leftX, y + textFourthLine, buf);

    graphics::drawCommonFooter(display, x, y);
}

#endif // HAS_SCREEN
#endif // USE_REPEATER_MODULE
