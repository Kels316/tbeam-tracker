#pragma once
#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include <Wire.h>

/**
 * TrackerModule  —  Heltec WiFi LoRa 32 V4 variant
 * Sends GPS position + compass heading over a private channel every 30 seconds.
 * Flashes TX LED on each packet send.
 *
 * Hardware (Heltec V4):
 *   GPS      : u-blox M8Q via SH1.25-8P GNSS connector (GPIO 38 RX / GPIO 39 TX)
 *   Compass  : QMC5883L via I2C shared with OLED (SDA=17, SCL=18, addr 0x0D)
 *              OLED is at 0x3C — no address conflict.
 *   Power LED: GPIO 2  (always on — external indicator)
 *   TX LED   : GPIO 4  (flashes 200 ms per packet — external indicator)
 *
 * Pin conflict notes:
 *   GPIO 13 = LORA_BUSY  — do not use (LoRa radio DIO2)
 *   GPIO 36 = VEXT_ENABLE — do not use (external power rail control)
 *   GPIO 38 = GNSS_RX    — dedicated GNSS connector (in use by GPS)
 *   GPIO 39 = GNSS_TX    — dedicated GNSS connector (in use by GPS)
 */
class TrackerModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    TrackerModule();

  protected:
    // OSThread: called on the 30s interval
    int32_t runOnce() override;

  private:
    void sendPosition();
    void flashTxLed();

    // Read heading from QMC5883L; returns degrees 0-359, or -1 on error
    float readHeadingDeg();

    // QMC5883L I2C address (fixed in hardware)
    static constexpr uint8_t QMC_ADDR      = 0x0D;

    // GPIO pins
    // GPIO 2: J3 pin 13 — free general-purpose I/O
    // GPIO 4: J3 pin 15 — free general-purpose I/O (I2C_SDA1 only used if Wire1 active)
    static constexpr uint8_t POWER_LED_PIN = 2;
    static constexpr uint8_t TX_LED_PIN    = 4;

    // Timing
    static constexpr uint32_t TX_LED_MS       = 200;
    static constexpr uint32_t INTERVAL_MS     = 30000;
    static constexpr uint32_t BOOT_DELAY_MS   = 15000; // wait for service->init() to settle
    static constexpr uint32_t BOOT_POLL_MS    = 500;

    // Magnetic declination for your area (degrees, + east / - west)
    // Brisbane, AU ≈ +11.5°  — adjust for your deployment location
    // Find yours at: https://www.magnetic-declination.com
    static constexpr float MAG_DECLINATION = 11.5f;

    bool compassOk = false;
    bool firstRun  = true;

    void sendHello();
};

extern TrackerModule *trackerModule;
