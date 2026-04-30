#pragma once
/**
 * tracker_channel.h  —  Heltec WiFi LoRa 32 V4 variant
 *
 * Bakes a private channel + PSK into the firmware at compile time.
 * Also sets LoRa region, GPS pins, and device role on first boot.
 * Call setupTrackerChannel() once from setup() BEFORE service->init().
 *
 * HOW TO CHANGE YOUR KEY:
 *   1. Generate a random 32-byte key (e.g. `openssl rand -base64 32`)
 *   2. Convert each byte to 0x?? hex and replace TRACKER_PSK below.
 *   3. Rebuild and reflash all nodes that need to communicate.
 *
 * PIN NOTES (Heltec V4 — ESP32-S3):
 *   GPS RX (board side) = GPIO 38  (GNSS_RX on the SH1.25-8P connector)
 *   GPS TX (board side) = GPIO 39  (GNSS_TX on the SH1.25-8P connector)
 *   DO NOT use GPIO 36 for GPS — it is VEXT_ENABLE (external power control).
 *   DO NOT use GPIO 13 for strobe — it is LORA_BUSY (SX1262 DIO2).
 */

#include "NodeDB.h"
#include "configuration.h"
#include "mesh/generated/meshtastic/channel.pb.h"
#include "mesh/generated/meshtastic/config.pb.h"
#include <string.h>

// -------------------------------------------------------------------
// YOUR PRIVATE CHANNEL SETTINGS — edit these before building
// -------------------------------------------------------------------

#define TRACKER_CHANNEL_NAME "TRACKER"

// 256-bit (32-byte) pre-shared key
// Replace with your own key — every node must share this exact key.
static const uint8_t TRACKER_PSK[32] = {
    0x4a, 0x3f, 0x8c, 0x21, 0xd7, 0x55, 0xb2, 0x09,
    0xe1, 0x7a, 0x44, 0xfc, 0x30, 0x8e, 0x6b, 0xd3,
    0x92, 0x1c, 0x5f, 0xa8, 0x77, 0x03, 0xe6, 0x4d,
    0xbb, 0x29, 0x10, 0x58, 0xc4, 0x9d, 0x6e, 0xf1
};

// -------------------------------------------------------------------

void setupTrackerChannel()
{
    if (strcmp(channels.getByIndex(1).settings.name, TRACKER_CHANNEL_NAME) == 0) {
        LOG_INFO("setupTrackerChannel: already configured, skipping\n");
        return;
    }
    LOG_INFO("setupTrackerChannel: configuring TRACKER node (Heltec V4)\n");

    // ── Channel ────────────────────────────────────────────────────
    // Bypass channels.setChannel() — it silently discards writes when
    // channels_count < 2 (getByIndex returns a throwaway static buffer).
    // Write directly to channelFile and save.

    // Ensure the array is large enough
    if (channelFile.channels_count < 2)
        channelFile.channels_count = 2;

    // Channel 0: keep existing settings, just force PRIMARY role
    channelFile.channels[0].role  = meshtastic_Channel_Role_PRIMARY;
    channelFile.channels[0].index = 0;

    // Channel 1: TRACKER with our custom PSK
    memset(&channelFile.channels[1], 0, sizeof(channelFile.channels[1]));
    channelFile.channels[1].index = 1;
    channelFile.channels[1].role  = meshtastic_Channel_Role_SECONDARY;
    strncpy(channelFile.channels[1].settings.name, TRACKER_CHANNEL_NAME,
            sizeof(channelFile.channels[1].settings.name) - 1);
    channelFile.channels[1].settings.psk.size = sizeof(TRACKER_PSK);
    memcpy(channelFile.channels[1].settings.psk.bytes, TRACKER_PSK, sizeof(TRACKER_PSK));

    channels.onConfigChanged();
    nodeDB->saveToDisk(SEGMENT_CHANNELS);

    // ── LoRa region + modem ────────────────────────────────────────
    config.lora.region       = meshtastic_Config_LoRaConfig_RegionCode_ANZ;
    config.lora.use_preset   = true;
    config.lora.modem_preset = meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
    config.lora.tx_enabled   = true;
    config.lora.tx_power     = 30;

    // ── GPS ────────────────────────────────────────────────────────
    // Heltec V4 dedicated GNSS connector (SH1.25-8P):
    //   GPS_RX_PIN = 39 — bits going TOWARDS the GPS (Meshtastic rx_gpio)
    //   GPS_TX_PIN = 38 — bits going TOWARDS the CPU  (Meshtastic tx_gpio)
    // Note: GPIO 36 is VEXT_ENABLE — do not use for GPS.
    config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_ENABLED;
    config.position.rx_gpio  = 39; // GPS_RX_PIN: towards GPS
    config.position.tx_gpio  = 38; // GPS_TX_PIN: towards CPU
    config.position.position_broadcast_secs = 3600; // our module handles 30s sends

    // ── Device ─────────────────────────────────────────────────────
    config.device.role = meshtastic_Config_DeviceConfig_Role_TRACKER;

    // ── Bluetooth — fixed PIN so no screen needed ──────────────────
    config.bluetooth.enabled   = true;
    config.bluetooth.mode      = meshtastic_Config_BluetoothConfig_PairingMode_FIXED_PIN;
    config.bluetooth.fixed_pin = 123456;

    // ── Telemetry — send battery level every 5 minutes ────────────
    moduleConfig.telemetry.device_update_interval   = 300;
    moduleConfig.telemetry.device_telemetry_enabled = true;
    nodeDB->saveToDisk(SEGMENT_MODULECONFIG);

    nodeDB->saveToDisk(SEGMENT_CONFIG);
    LOG_INFO("setupTrackerChannel: config saved\n");
}
