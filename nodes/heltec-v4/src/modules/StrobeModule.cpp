#include "src/modules/StrobeModule.h"
#include <Arduino.h>

StrobeModule *strobeModule;

StrobeModule::StrobeModule()
    : concurrency::OSThread("StrobeModule")
{
    pinMode(STROBE_PIN, OUTPUT);
    digitalWrite(STROBE_PIN, LOW); // ensure LED is off at boot
    LOG_INFO("StrobeModule: recovery strobe on GPIO %d\n", STROBE_PIN);
}

int32_t StrobeModule::runOnce()
{
    bool hasLock = (gps && gps->hasLock());

    // 3 blinks regardless of state; interval changes based on lock
    doBlinks(3);

    return hasLock ? INTERVAL_LOCK_MS : INTERVAL_NOLOCK_MS;
}

void StrobeModule::doBlinks(uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        digitalWrite(STROBE_PIN, HIGH);
        delay(BLINK_ON_MS);
        digitalWrite(STROBE_PIN, LOW);
        if (i < count - 1) {
            delay(BLINK_GAP_MS); // gap between blinks, not after the last one
        }
    }
}
