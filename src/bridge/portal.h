// Setup portal: a Wi-Fi hotspot with a captive web page for pairing the bike,
// watching live stats and changing settings. Only runs in setup mode.
#pragma once
#include <Arduino.h>

void portalStart(const char* ssid, const char* password);
void portalStop();
void portalLoop();
bool portalActive();
int portalClientCount();
uint32_t portalLastRequestMs();

// Implemented by the bridge (main.cpp); called from portalLoop() on the main task.
String bridgeStatusJson();
String bridgeDevicesJson();
String bridgeSettingsJson();
void bridgeStartScan();
bool bridgePair(const String& address);
void bridgeForgetBike();
void bridgeForgetWatch();
bool bridgeSetSetting(const String& key, float value);
void bridgeResetTotalEnergy();
