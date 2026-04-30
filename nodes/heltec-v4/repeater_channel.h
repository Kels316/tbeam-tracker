#pragma once
/**
 * repeater_channel.h  —  Heltec V4 drone repeater variant
 *
 * Configures the node as a dumb REPEATER on the same private channel
 * as the tracker and T-Deck.  No GPS, no telemetry, no custom modules.
 * Drop this on a drone to extend LoRa range between tracker and receiver.
 *
 * Call setupRepeaterChannel() from setup() BEFORE service->init().
 */

#include "NodeDB.h"
#include "configuration.h"
#include "mesh/generated/meshtastic/channel.pb.h"
#include "mesh/generated/meshtastic/config.pb.h"
#include <string.h>

// Shared channel settings — must match tracker and T-Deck exactly
#define REPEATER_CHANNEL_NAME "TRACKER"

static const uint8_t REPEATER_PSK[32] = {
    0x4a, 0x3f, 0x8c, 0x21, 0xd7, 0x55, 0xb2, 0x09,
    0xe1, 0x7a, 0x44, 0xfc, 0x30, 0x8e, 0x6b, 0xd3,
    0x92, 0x1c, 0x5f, 0xa8, 0x77, 0x03, 0xe6, 0x4d,
    0xbb, 0x29, 0x10, 0x58, 0xc4, 0x9d, 0x6e, 0xf1
};

void setupRepeaterChannel()
{
    // Always enforce GPS off — this must happen even on subsequent boots
    // because the GPS driver initialises before setup() on first boot.
    if (config.position.gps_mode != meshtastic_Config_PositionConfig_GpsMode_DISABLED) {
        config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_DISABLED;
        nodeDB->saveToDisk(SEGMENT_CONFIG);
        LOG_INFO("setupRepeaterChannel: GPS disabled and saved\n");
    }

    if (config.device.role == meshtastic_Config_DeviceConfig_Role_REPEATER &&
        strcmp(channelFile.channels[1].settings.name, REPEATER_CHANNEL_NAME) == 0) {
        LOG_INFO("setupRepeaterChannel: already configured, skipping\n");
        return;
    }
    LOG_INFO("setupRepeaterChannel: configuring drone repeater (Heltec V4)\n");

    // ── Channels ───────────────────────────────────────────────────
    if (channelFile.channels_count < 2)
        channelFile.channels_count = 2;

    channelFile.channels[0].role  = meshtastic_Channel_Role_PRIMARY;
    channelFile.channels[0].index = 0;

    memset(&channelFile.channels[1], 0, sizeof(channelFile.channels[1]));
    channelFile.channels[1].index = 1;
    channelFile.channels[1].role  = meshtastic_Channel_Role_SECONDARY;
    strncpy(channelFile.channels[1].settings.name, REPEATER_CHANNEL_NAME,
            sizeof(channelFile.channels[1].settings.name) - 1);
    channelFile.channels[1].settings.psk.size = sizeof(REPEATER_PSK);
    memcpy(channelFile.channels[1].settings.psk.bytes, REPEATER_PSK, sizeof(REPEATER_PSK));

    channels.onConfigChanged();
    nodeDB->saveToDisk(SEGMENT_CHANNELS);

    // ── LoRa ───────────────────────────────────────────────────────
    config.lora.region       = meshtastic_Config_LoRaConfig_RegionCode_ANZ;
    config.lora.use_preset   = true;
    config.lora.modem_preset = meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST;
    config.lora.tx_enabled   = true;
    config.lora.tx_power     = 30;

    // ── Device role ────────────────────────────────────────────────
    config.device.role = meshtastic_Config_DeviceConfig_Role_REPEATER;

    // ── GPS off ────────────────────────────────────────────────────
    config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_DISABLED;

    // ── Bluetooth — fixed PIN ──────────────────────────────────────
    config.bluetooth.enabled   = true;
    config.bluetooth.mode      = meshtastic_Config_BluetoothConfig_PairingMode_FIXED_PIN;
    config.bluetooth.fixed_pin = 123456;

    nodeDB->saveToDisk(SEGMENT_CONFIG);
    LOG_INFO("setupRepeaterChannel: config saved\n");
}
