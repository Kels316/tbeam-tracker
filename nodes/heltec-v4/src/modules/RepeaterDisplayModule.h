#pragma once
/**
 * RepeaterDisplayModule.h — Heltec V4 drone repeater status screen
 *
 * Adds a single OLED page showing:
 *   header : "REPEATER" + battery icon (via drawCommonHeader)
 *   line 1 : Battery % and voltage
 *   line 2 : Uptime
 *   line 3 : Channel utilization / TX air util
 *   line 4 : Node ID
 *
 * Only compiled when USE_REPEATER_MODULE is defined.
 */

#ifdef USE_REPEATER_MODULE

#include "MeshModule.h"

class RepeaterDisplayModule : public MeshModule
{
  public:
    RepeaterDisplayModule();

    virtual bool wantUIFrame() override { return true; }

#if HAS_SCREEN
    virtual void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state,
                           int16_t x, int16_t y) override;
#endif

  protected:
    // We don't handle any packets — pure display module
    virtual bool wantPacket(const meshtastic_MeshPacket *p) override { return false; }
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override { return ProcessMessage::CONTINUE; }
};

extern RepeaterDisplayModule *repeaterDisplayModule;

#endif // USE_REPEATER_MODULE
