// Status screen for the Waveshare ESP32-C6-LCD-1.47 (ST7789, 172x320).
#pragma once
#include <Arduino.h>

struct DisplayState {
  bool bikeUp;
  bool watchUp;
  bool hasData;        // bike connected and data received
  int power;           // W
  float rpm;
  int level;           // resistance level reported by the bike
  const char* mode;    // already formatted, e.g. "ERG 150W"
  double sessionKj;
  double totalKj;
};

void displayBegin();
void displayDraw(const DisplayState& s);
