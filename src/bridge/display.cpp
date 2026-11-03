#include "display.h"

#include <Arduino_GFX_Library.h>
#include <qrcode.h>

// Pins from the Waveshare ESP32-C6-LCD-1.47 wiki.
static const int PIN_MOSI = 6, PIN_SCLK = 7, PIN_CS = 14, PIN_DC = 15, PIN_RST = 21, PIN_BL = 22;

// The 172-pixel-wide panel sits in the middle of the ST7789's 240 columns.
static const int COL_OFFSET = 34;

static const uint16_t C_BLACK = 0x0000, C_WHITE = 0xFFFF, C_GREY = 0x8410, C_DARK = 0x18E3, C_GREEN = 0x07E0,
                      C_RED = 0xF800, C_AMBER = 0xFD20, C_CYAN = 0x07FF;

static Arduino_DataBus* bus;
static Arduino_GFX* panel;
static Arduino_Canvas* gfx;  // drawn off-screen, then flushed in one go: no flicker

// Full brightness mostly turns into heat in the board's regulator, right next
// to the chip, so the default (set from main) is well below 100 %.
void displaySetBrightness(uint8_t percent) { ledcWrite(PIN_BL, constrain(percent, 0, 100) * 255 / 100); }

void displayBegin() {
  ledcAttach(PIN_BL, 5000, 8);
  displaySetBrightness(60);
  bus = new Arduino_HWSPI(PIN_DC, PIN_CS, PIN_SCLK, PIN_MOSI, GFX_NOT_DEFINED);
  // Rotation 1 = landscape, 320x172. IPS panel, so colours are inverted.
  panel = new Arduino_ST7789(bus, PIN_RST, 1, true, 172, 320, COL_OFFSET, 0, COL_OFFSET, 0);
  gfx = new Arduino_Canvas(320, 172, panel);
  if (!gfx->begin(40000000)) Serial.println("Display init failed");
  gfx->fillScreen(C_BLACK);
  gfx->flush();
}

// --- Seven-segment digits ------------------------------------------------------

// Segment bits: a=top, b=top right, c=bottom right, d=bottom, e=bottom left,
// f=top left, g=middle.
static const uint8_t DIGIT_SEGMENTS[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
static const uint8_t SEG_DASH = 0x40, SEG_BLANK = 0x00;

static void drawSegments(int x, int y, int w, int h, int t, uint8_t on, uint16_t color) {
  const int gap = 2, r = t / 2;
  const int half = (h - 3 * t) / 2;
  struct Seg { int x, y, w, h; } segs[7] = {
      {x + t + gap, y, w - 2 * t - 2 * gap, t},                                 // a
      {x + w - t, y + t + gap, t, half - 2 * gap},                              // b
      {x + w - t, y + 2 * t + half + gap, t, half - 2 * gap},                   // c
      {x + t + gap, y + h - t, w - 2 * t - 2 * gap, t},                         // d
      {x, y + 2 * t + half + gap, t, half - 2 * gap},                           // e
      {x, y + t + gap, t, half - 2 * gap},                                      // f
      {x + t + gap, y + t + half, w - 2 * t - 2 * gap, t},                      // g
  };
  for (int i = 0; i < 7; i++)
    gfx->fillRoundRect(segs[i].x, segs[i].y, segs[i].w, segs[i].h, r, (on >> i) & 1 ? color : C_DARK);
}

// Right-aligned number in `digits` cells; dashes when value < 0.
static void drawBigNumber(int x, int y, int digits, int value, uint16_t color) {
  const int w = 54, h = 96, t = 11, pitch = 64;
  for (int i = digits - 1; i >= 0; i--) {
    uint8_t seg;
    if (value < 0) seg = SEG_DASH;
    else if (value == 0 && i < digits - 1) seg = SEG_BLANK;  // no leading zeros
    else seg = DIGIT_SEGMENTS[value % 10];
    drawSegments(x + i * pitch, y, w, h, t, seg, color);
    if (value > 0) value /= 10;
  }
}

// --- Screen --------------------------------------------------------------------

static void label(int x, int y, const char* text, uint8_t size, uint16_t color) {
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->setCursor(x, y);
  gfx->print(text);
}

static void statusLight(int x, const char* name, uint16_t color) {
  gfx->fillCircle(x + 6, 12, 6, color);
  label(x + 18, 5, name, 2, C_WHITE);
}

static void formatKj(char* buf, size_t n, double kj) {
  if (kj < 1000) snprintf(buf, n, "%.1f kJ", kj);
  else if (kj < 100000) snprintf(buf, n, "%.0f kJ", kj);
  else snprintf(buf, n, "%.1f MJ", kj / 1000);
}

// --- Setup screen ------------------------------------------------------------

// The Wi-Fi QR code ("WIFI:T:WPA;S:...;P:...;;") lets a phone camera join the
// hotspot directly. Generated once and cached.
static const int QR_MAX = 41;  // version 6, far more than the ~50 characters needed
static bool qrModules[QR_MAX][QR_MAX];
static int qrSize = 0;
static char qrText[96] = "";

static void storeQr(esp_qrcode_handle_t qr) {
  qrSize = min(esp_qrcode_get_size(qr), QR_MAX);
  for (int y = 0; y < qrSize; y++)
    for (int x = 0; x < qrSize; x++) qrModules[y][x] = esp_qrcode_get_module(qr, x, y);
}

static void drawWifiQr(int x, int y, int maxPx, const char* ssid, const char* password) {
  char text[96];
  snprintf(text, sizeof(text), "WIFI:T:WPA;S:%s;P:%s;;", ssid, password);
  if (strcmp(text, qrText) != 0) {
    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    cfg.display_func = storeQr;
    cfg.max_qrcode_version = 6;
    qrSize = 0;
    if (esp_qrcode_generate(&cfg, text) == ESP_OK) strcpy(qrText, text);
  }
  if (!qrSize) return;
  int scale = maxPx / (qrSize + 4);  // 2-module quiet zone on each side
  int side = (qrSize + 4) * scale;
  gfx->fillRect(x, y, side, side, C_WHITE);
  for (int my = 0; my < qrSize; my++)
    for (int mx = 0; mx < qrSize; mx++)
      if (qrModules[my][mx]) gfx->fillRect(x + (mx + 2) * scale, y + (my + 2) * scale, scale, scale, C_BLACK);
}

static void drawSetup(const DisplayState& s) {
  drawWifiQr(6, 14, 144, s.ssid, s.password);

  const int x = 160;
  label(x, 8, "SETUP", 2, C_CYAN);
  label(x, 32, "1. Scan the code, or", 1, C_GREY);
  label(x, 42, "   join Wi-Fi:", 1, C_GREY);
  label(x, 54, s.ssid, 1, C_WHITE);
  label(x, 68, "   password:", 1, C_GREY);
  label(x, 80, s.password, 2, C_WHITE);
  label(x, 102, "2. Open", 1, C_GREY);
  label(x, 114, s.url, 1, C_WHITE);

  gfx->fillCircle(x + 4, 141, 4, s.bikeUp ? C_GREEN : s.bikeName[0] ? C_RED : C_AMBER);
  label(x + 14, 138, s.bikeName[0] ? s.bikeName : "No bike paired", 1, C_WHITE);
  label(x, 156, "Hold BOOT to exit", 1, C_GREY);
}

void displayDraw(const DisplayState& s) {
  // Skip the redraw (and the 110 KB SPI transfer) when nothing visible changed.
  static char lastShown[160];
  char sessionText[16], totalText[16], shown[160];
  formatKj(sessionText, sizeof(sessionText), s.sessionKj);
  formatKj(totalText, sizeof(totalText), s.totalKj);
  if (s.setup)
    snprintf(shown, sizeof(shown), "setup|%d|%s|%s|%s|%s", s.bikeUp, s.ssid, s.password, s.url, s.bikeName);
  else
    snprintf(shown, sizeof(shown), "%d%d%d|%d|%.0f|%d|%s|%s|%s", s.bikeUp, s.watchUp, s.hasData, s.power, s.rpm,
             s.level, s.mode, sessionText, totalText);
  if (strcmp(shown, lastShown) == 0) return;
  strcpy(lastShown, shown);

  if (s.setup) {
    gfx->fillScreen(C_BLACK);
    drawSetup(s);
    gfx->flush();
    return;
  }

  gfx->fillScreen(C_BLACK);

  // Top bar: connections and mode.
  statusLight(4, "BIKE", s.bikeUp ? C_GREEN : C_RED);
  statusLight(84, "WATCH", s.watchUp ? C_GREEN : C_GREY);
  gfx->setTextSize(2);
  int16_t bx, by;
  uint16_t bw, bh;
  gfx->getTextBounds(s.mode, 0, 0, &bx, &by, &bw, &bh);
  label(316 - bw, 5, s.mode, 2, C_CYAN);
  gfx->drawFastHLine(0, 26, 320, C_DARK);

  // Watts.
  int watts = s.hasData ? constrain(s.power, 0, 999) : -1;
  drawBigNumber(6, 34, 3, watts, C_WHITE);
  label(204, 112, "W", 3, C_GREY);

  // Cadence and level.
  char buf[24];
  label(240, 34, "RPM", 1, C_GREY);
  if (s.hasData) snprintf(buf, sizeof(buf), "%.0f", s.rpm);
  else snprintf(buf, sizeof(buf), "--");
  label(240, 46, buf, 3, C_WHITE);
  label(240, 84, "LEVEL", 1, C_GREY);
  if (s.hasData) snprintf(buf, sizeof(buf), "%d", s.level);
  else snprintf(buf, sizeof(buf), "--");
  label(240, 96, buf, 3, C_WHITE);

  // Energy.
  gfx->drawFastHLine(0, 138, 320, C_DARK);
  label(6, 144, "SESSION", 1, C_GREY);
  formatKj(buf, sizeof(buf), s.sessionKj);
  label(6, 155, buf, 2, C_AMBER);
  label(168, 144, "TOTAL", 1, C_GREY);
  formatKj(buf, sizeof(buf), s.totalKj);
  label(168, 155, buf, 2, C_AMBER);

  gfx->flush();
}
