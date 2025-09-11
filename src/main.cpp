// ==================== main.cpp (ESP32 + ILI9341, ZOH + time axis) ====================
#include <Arduino.h>
#include <SPI.h>
#include <Ticker.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// --- your custom fonts ---
#include "Aurora4pt7b.h"   // small font: HUD, axis labels
#include "Aurora7pt7b.h"   // large font: title
#include "Aurora10pt7b.h"  // really large (if needed)

// -------------------- USER SETTINGS --------------------
#define TFT_CS   5
#define TFT_DC   21
#define TFT_RST  22
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_MISO 19

static inline uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// ----------- Color palette -----------
uint16_t COL_BG    = ILI9341_BLACK;
uint16_t COL_TEXT  = ILI9341_WHITE;          // HUD + axis labels
uint16_t COL_TITLE = RGB565(244, 206, 39);   // yellow title text (#f4ce27)
uint16_t COL_AXIS  = ILI9341_WHITE;
uint16_t COL_TICKS = ILI9341_WHITE;
uint16_t COL_GRID  = RGB565(90, 90, 110);
uint16_t COL_TRACE = ILI9341_WHITE;
// ------------------------------------------------------

#define MIC_PIN 36  // ADC1_CH0

// Screen geometry (rotation 1 => 320x240)
constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 240;

// Layout: title band, plot, x-axis band, bottom HUD
constexpr int TITLE_H          = 24;  // fits big title font
constexpr int XAXIS_H          = 18;  // x-axis ticks/labels band
constexpr int HUD_H            = 20;  // bottom banner
constexpr int PLOT_LMARGIN     = 38;  // space for Y-axis ticks/labels

constexpr int PLOT_X0 = PLOT_LMARGIN;
constexpr int PLOT_Y0 = TITLE_H;
constexpr int PLOT_W  = SCREEN_W - PLOT_X0;
constexpr int PLOT_H  = SCREEN_H - (TITLE_H + XAXIS_H + HUD_H);

// VU LED pins (12 outputs)
constexpr uint8_t Level1 = 2, Level2 = 4, Level3 = 12, Level4 = 13, Level5 = 14, Level6 = 15,
                  Level7 = 25, Level8 = 26, Level9 = 27, Level10 = 32, Level11 = 33, Level12 = 16;

// -------------------- GLOBALS --------------------
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);
// If your TFT has no SDO/MISO: Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

Ticker micTicker;
volatile bool micTickFlag = false;

// Effective sampling frequency (samples/sec) shown to user
volatile uint32_t gSampleFreqHz = 5000; // default 5 kHz
constexpr uint32_t FS_MIN = 1000;
constexpr uint32_t FS_MAX = 20000;

// ZOH = pixels-per-sample (hold horizontally for N pixels)
volatile uint8_t zoh = 2;             // default 2 px/sample
constexpr uint8_t ZOH_MIN = 1;
constexpr uint8_t ZOH_MAX = 10;

// Trace thickness (vertical pixels)
volatile uint8_t traceThick = 1;      // default 1 px
constexpr uint8_t THICK_MIN = 1;
constexpr uint8_t THICK_MAX = 4;

// Midline toggle
volatile bool gShowMidline = false;

volatile int readings[PLOT_W]; // plotted raw values for VU etc.
uint16_t DCOffset = 0;
int midY = 0;

// ZOH state for pixel-rate ticker
volatile int currentSampleRaw = 0;
volatile uint8_t holdCount = 0;   // counts down from zoh-1 to 0

// Dashed midline pattern
constexpr int DASH_ON = 6;
constexpr int DASH_OFF = 6;

// -------------------- HELPERS --------------------
void measureText(const GFXfont *font, const char *text, int *w, int *h) {
  int16_t x1, y1; uint16_t tw, th;
  tft.setFont(font);
  tft.getTextBounds(text, 0, 0, &x1, &y1, &tw, &th);
  *w = (int)tw; *h = (int)th;
}

static inline int adcToY_raw(int raw) {
  if (raw < 0) raw = 0;
  if (raw > 4095) raw = 4095;
  int y = PLOT_Y0 + PLOT_H - 1 - ((uint32_t)raw * PLOT_H >> 12);
  return y;
}

uint16_t backgroundAt(int plotX, int y) {
  if (y < PLOT_Y0 || y >= PLOT_Y0 + PLOT_H) return COL_BG;
  if (gShowMidline && y == midY) {
    int xInPlot = plotX - PLOT_X0;
    int phase = xInPlot % (DASH_ON + DASH_OFF);
    if (phase < DASH_ON) return COL_GRID;
  }
  return COL_BG;
}

void IRAM_ATTR onMicTick() { micTickFlag = true; }

// Pixel-rate timer = Fs(samples/s) * zoh(pixels/sample)
void applyTicker() {
  float pixelRate = (float)gSampleFreqHz * (float)zoh;
  if (pixelRate < 100.0f) pixelRate = 100.0f;           // avoid too slow
  if (pixelRate > 40000.0f) pixelRate = 40000.0f;       // avoid overdraw
  micTicker.detach();
  micTicker.attach(1.0f / pixelRate, onMicTick);
}

void setSampleFreq(uint32_t fs) {
  if (fs < FS_MIN) fs = FS_MIN;
  if (fs > FS_MAX) fs = FS_MAX;
  gSampleFreqHz = fs;
  applyTicker();
}

void setZOH(uint8_t n) {
  if (n < ZOH_MIN) n = ZOH_MIN;
  if (n > ZOH_MAX) n = ZOH_MAX;
  zoh = n;
  holdCount = 0; // force a fresh sample soon
  applyTicker();
}

void setTraceThick(uint8_t th) {
  if (th < THICK_MIN) th = THICK_MIN;
  if (th > THICK_MAX) th = THICK_MAX;
  if (th == traceThick) return;
  traceThick = th;
}

uint16_t estimateDCoffset(int numSamples) {
  uint32_t sum = 0;
  for (int i = 0; i < numSamples; ++i) { sum += analogRead(MIC_PIN); delay(2); }
  return (uint16_t)(sum / (uint32_t)numSamples);
}

// Choose a “nice” time tick step (1/2/5 ms multiples) for the x-axis
float chooseNiceMsStep(float total_ms, int target_ticks = 6) {
  float raw = total_ms / max(2, target_ticks);
  // base in ms
  float base[] = {1.0f, 2.0f, 5.0f};
  float pow10 = 1.0f;
  while (true) {
    for (int i = 0; i < 3; ++i) {
      float s = base[i] * pow10;
      if (s >= raw) return s;
    }
    pow10 *= 10.0f;
  }
}

// -------------------- DRAWING --------------------
void drawTitle() {
  const char *title = "Audio Signal Visualiser";
  tft.setFont(&aurora_247pt7b);
  int16_t x1, y1; uint16_t tw, th;
  tft.getTextBounds(title, 0, 0, &x1, &y1, &tw, &th);
  int x = PLOT_X0 + ((int)PLOT_W - (int)tw) / 2;
  int baselineY = TITLE_H - ((TITLE_H - (int)th) / 2);
  tft.setTextColor(COL_TITLE, COL_BG);
  tft.setCursor(x, baselineY);
  tft.print(title);
}

void drawBottomHUD() {
  // Clear HUD band (only plot width)
  tft.fillRect(PLOT_X0, SCREEN_H - HUD_H, PLOT_W, HUD_H, COL_BG);

  tft.setFont(&aurora_244pt7b);
  tft.setTextColor(COL_TEXT, COL_BG);

  char fsBuf[24]; snprintf(fsBuf, sizeof(fsBuf), "Fs: %.1fkHz", gSampleFreqHz / 1000.0f);
  char zBuf[24];  snprintf(zBuf,  sizeof(zBuf),  "ZOH: %u", (unsigned)zoh);
  char thBuf[24]; snprintf(thBuf, sizeof(thBuf), "Thk: %upx", (unsigned)traceThick);

  int16_t x1, y1; uint16_t wFs, hFs, wZ, hZ, wTh, hTh;
  tft.getTextBounds(fsBuf, 0, 0, &x1, &y1, &wFs, &hFs);
  tft.getTextBounds(zBuf,  0, 0, &x1, &y1, &wZ,  &hZ);
  tft.getTextBounds(thBuf, 0, 0, &x1, &y1, &wTh, &hTh);

  const int gap = 16;
  int totalW = (int)wFs + gap + (int)wZ + gap + (int)wTh;
  int startX = PLOT_X0 + (PLOT_W - totalW) / 2;
  int textH = max((int)hFs, max((int)hZ, (int)hTh));
  int baselineY = SCREEN_H - (HUD_H - textH) / 2;

  int x = startX;
  tft.setCursor(x, baselineY); tft.print(fsBuf);
  x += wFs + gap;
  tft.setCursor(x, baselineY); tft.print(zBuf);
  x += wZ + gap;
  tft.setCursor(x, baselineY); tft.print(thBuf);
}

void drawYAxis() {
  // left strip
  tft.fillRect(0, PLOT_Y0, PLOT_LMARGIN, PLOT_H, COL_BG);
  // axis line
  tft.drawFastVLine(PLOT_LMARGIN - 1, PLOT_Y0, PLOT_H, COL_AXIS);

  auto yForVolt = [](float v) -> int {
    int raw = (int)roundf((v / 3.3f) * 4095.0f);
    return adcToY_raw(raw);
  };

  tft.setFont(&aurora_244pt7b);
  tft.setTextColor(COL_TEXT, COL_BG);

  for (int i = 0; i <= 33; ++i) {
    float v = i * 0.1f;
    int y = yForVolt(v);
    if (y < PLOT_Y0 || y >= PLOT_Y0 + PLOT_H) continue;

    bool major = (i % 5 == 0);
    int tickLen = major ? 7 : 4;
    int xStart = PLOT_LMARGIN - 1 - tickLen;
    tft.drawFastHLine(xStart, y, tickLen, major ? COL_AXIS : COL_TICKS);

    if (major) {
      char buf[12];
      dtostrf(v, 3, 1, buf); strcat(buf, "V");
      int16_t x1, y1; uint16_t tw, th;
      tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);
      int tx = (PLOT_LMARGIN - 3) - (int)tw;
      int baselineY = y + (int)th / 2;
      int boxX = tx - 2;
      int boxW = (PLOT_LMARGIN - 2) - boxX;
      int boxY = baselineY - (int)th - 1;
      int boxH = (int)th + 2;
      tft.fillRect(boxX, boxY, boxW, boxH, COL_BG);
      tft.setCursor(tx, baselineY);
      tft.print(buf);
    }
  }

  // Ensure 3.3V label
  float v = 3.3f;
  int y = yForVolt(v);
  if (y >= PLOT_Y0 && y < PLOT_Y0 + PLOT_H) {
    char buf[12];
    dtostrf(v, 3, 1, buf); strcat(buf, "V");
    int16_t x1, y1; uint16_t tw, th;
    tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);
    int tx = (PLOT_LMARGIN - 3) - (int)tw;
    int baselineY = y + (int)th / 2;
    int boxX = tx - 2;
    int boxW = (PLOT_LMARGIN - 2) - boxX;
    int boxY = baselineY - (int)th - 1;
    int boxH = (int)th + 2;
    tft.fillRect(boxX, boxY, boxW, boxH, COL_BG);
    tft.setCursor(tx, baselineY);
    tft.print(buf);
  }
}

void drawPlotBackground() {
  tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, PLOT_H, COL_BG);
  midY = adcToY_raw((int)roundf((1.65f / 3.3f) * 4095.0f));
  if (gShowMidline) {
    int x = PLOT_X0;
    while (x < PLOT_X0 + PLOT_W) {
      int run = min(DASH_ON, PLOT_X0 + PLOT_W - x);
      tft.drawFastHLine(x, midY, run, COL_GRID);
      x += DASH_ON + DASH_OFF;
    }
  }
}

void drawXAxis() {
  // Clear axis band
  int y0 = PLOT_Y0 + PLOT_H;
  tft.fillRect(PLOT_X0, y0, PLOT_W, XAXIS_H, COL_BG);

  // Time window (seconds) across PLOT_W pixels with ZOH pixels/sample
  float time_window_s = (float)PLOT_W / ((float)zoh * (float)gSampleFreqHz);
  float total_ms = time_window_s * 1000.0f;

  // choose tick step (ms)
  float step_ms = chooseNiceMsStep(total_ms, 6);
  float px_per_ms = ((float)zoh * (float)gSampleFreqHz) / 1000.0f; // pixels per ms
  float step_px_f = step_ms * px_per_ms;

  // guard
  if (step_px_f < 10.0f) step_px_f = 10.0f;

  tft.setFont(&aurora_244pt7b);
  tft.setTextColor(COL_TEXT, COL_BG);

  for (float xpx = (float)PLOT_X0; xpx <= (float)(PLOT_X0 + PLOT_W - 1) + 0.5f; xpx += step_px_f) {
    int xi = (int)roundf(xpx);
    // tick
    tft.drawFastVLine(xi, y0, 4, COL_TICKS);

    // label (ms)
    float t_ms = (xpx - (float)PLOT_X0) / px_per_ms;
    char lab[16];
    if (total_ms >= 1000.0f) {
      // show seconds when large windows (one decimal)
      snprintf(lab, sizeof(lab), "%.1fs", t_ms / 1000.0f);
    } else {
      // show ms
      snprintf(lab, sizeof(lab), "%.0fms", t_ms);
    }
    int16_t x1, y1; uint16_t tw, th;
    tft.getTextBounds(lab, 0, 0, &x1, &y1, &tw, &th);
    int tx = xi - (int)tw / 2;
    int by = y0 + XAXIS_H - 2;  // baseline near bottom of axis band
    tft.setCursor(tx, by);
    tft.print(lab);
  }

  // x-axis baseline
  tft.drawFastHLine(PLOT_X0, y0, PLOT_W, COL_AXIS);
}

// -------------------- SERIAL --------------------
void handleSerial() {
  if (!Serial.available()) return;

  int peekc = Serial.peek();
  if (peekc == 'f' || peekc == 'F') {
    String line = Serial.readStringUntil('\n');
    long val = -1; String num = "";
    for (size_t i = 0; i < line.length(); ++i) {
      if (isDigit((unsigned char)line[i])) num += line[i];
      else if (num.length() && !isDigit((unsigned char)line[i])) break;
    }
    if (num.length()) val = num.toInt();
    if (val > 0) {
      setSampleFreq((uint32_t)val);
      Serial.print(F("Fs set to ")); Serial.println((uint32_t)val);
      drawBottomHUD(); drawXAxis();
    } else {
      Serial.println(F("Parse Fs failed. Use: f8000 or fs=12000"));
    }
    return;
  }

  int c = Serial.read();
  if (c == '[') { // ZOH down
    if (zoh > ZOH_MIN) setZOH(zoh - 1);
    drawBottomHUD(); drawXAxis();
  }
  if (c == ']') { // ZOH up
    if (zoh < ZOH_MAX) setZOH(zoh + 1);
    drawBottomHUD(); drawXAxis();
  }
  if (c == '{') { // thickness down
    setTraceThick(traceThick > THICK_MIN ? traceThick - 1 : THICK_MIN);
    drawBottomHUD();
  }
  if (c == '}') { // thickness up
    setTraceThick(traceThick < THICK_MAX ? traceThick + 1 : THICK_MAX);
    drawBottomHUD();
  }
  if (c == 'm' || c == 'M') {
    gShowMidline = !gShowMidline;
    drawPlotBackground();
  }
}

// -------------------- VU OUTPUTS --------------------
void initVU() {
  const uint8_t pins[12] = {Level1, Level2, Level3, Level4, Level5, Level6,
                            Level7, Level8, Level9, Level10, Level11, Level12};
  for (uint8_t i = 0; i < 12; ++i) pinMode(pins[i], OUTPUT);
}
void setVU(uint8_t level) {
  const uint8_t pins[12] = {Level1, Level2, Level3, Level4, Level5, Level6,
                            Level7, Level8, Level9, Level10, Level11, Level12};
  for (uint8_t i = 0; i < 12; ++i) digitalWrite(pins[i], (i < level) ? HIGH : LOW);
}
void updateVU() {
  int maxRead = 0;
  for (int i = 0; i < PLOT_W; ++i) if (readings[i] > maxRead) maxRead = readings[i];
  uint16_t amp10 = (uint16_t)abs(maxRead - (int)DCOffset) >> 2; // 0..1023

  uint8_t VUlevel = 0;
  if      (amp10 > 446) VUlevel = 12;
  else if (amp10 > 385) VUlevel = 11;
  else if (amp10 > 328) VUlevel = 10;
  else if (amp10 > 288) VUlevel = 9;
  else if (amp10 > 228) VUlevel = 8;
  else if (amp10 > 185) VUlevel = 7;
  else if (amp10 > 146) VUlevel = 6;
  else if (amp10 > 111) VUlevel = 5;
  else if (amp10 > 82)  VUlevel = 4;
  else if (amp10 > 57)  VUlevel = 3;
  else if (amp10 > 36)  VUlevel = 2;
  else if (amp10 > 20)  VUlevel = 1;
  else VUlevel = 0;
  setVU(VUlevel);
}

// -------------------- SETUP / LOOP --------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("Commands: f8000 | fs=12000 | [ ] ZOH | { } thick | m midline"));

  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);
  pinMode(MIC_PIN, INPUT);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(COL_BG);

  drawTitle();
  drawYAxis();
  drawPlotBackground();
  drawXAxis();
  drawBottomHUD();

  initVU();

  for (int i = 0; i < PLOT_W; ++i) readings[i] = 0;

  // Centered DC offset message
  {
    tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, PLOT_H, COL_BG);
    tft.setFont(&aurora_247pt7b);
    tft.setTextColor(COL_TEXT, COL_BG);
    const char *msg = "Measuring DC Offset...";
    int w, h; measureText(&aurora_247pt7b, msg, &w, &h);
    int msgX = PLOT_X0 + (PLOT_W - w) / 2;
    int msgY = PLOT_Y0 + (PLOT_H + h) / 2;
    tft.setCursor(msgX, msgY); tft.print(msg);
    DCOffset = estimateDCoffset(256);

    tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, PLOT_H, COL_BG);
    char line[32];
    snprintf(line, sizeof(line), "DC: %.2f V", (DCOffset / 4095.0f) * 3.3f);
    measureText(&aurora_247pt7b, line, &w, &h);
    int valX = PLOT_X0 + (PLOT_W - w) / 2;
    int valY = PLOT_Y0 + (PLOT_H + h) / 2;
    tft.setCursor(valX, valY); tft.print(line);
    delay(800);
  }

  drawPlotBackground();
  drawXAxis();
  drawBottomHUD();

  applyTicker(); // start pixel-rate ticker based on Fs & ZOH
}

void loop() {
  static int x = 0;               // current plot column [0..PLOT_W-1]
  static int lastY[PLOT_W];       // last center y for erasing
  static uint8_t lastThk[PLOT_W]; // last thickness used
  static bool inited = false;

  if (!inited) {
    for (int i = 0; i < PLOT_W; ++i) { lastY[i] = -1; lastThk[i] = traceThick; }
    inited = true;
  }

  handleSerial();

  if (micTickFlag) {
    micTickFlag = false;

    // ZOH logic: sample only when holdCount == 0
    if (holdCount == 0) {
      currentSampleRaw = analogRead(MIC_PIN);
      holdCount = (zoh > 0) ? (zoh - 1) : 0;
    } else {
      --holdCount;
    }

    int plotX = PLOT_X0 + x;

    // erase previous vertical stroke
    if (lastY[x] >= PLOT_Y0 && lastY[x] < PLOT_Y0 + PLOT_H) {
      int halfPrev = (int)lastThk[x] / 2;
      int y0 = lastY[x] - halfPrev;
      int y1 = y0 + lastThk[x] - 1;
      if (y0 < PLOT_Y0) y0 = PLOT_Y0;
      if (y1 >= PLOT_Y0 + PLOT_H) y1 = PLOT_Y0 + PLOT_H - 1;
      for (int yy = y0; yy <= y1; ++yy) {
        tft.drawPixel(plotX, yy, backgroundAt(plotX, yy));
      }
    }

    // store & draw current sample
    readings[x] = currentSampleRaw;
    int y = adcToY_raw(currentSampleRaw);

    if (y >= PLOT_Y0 && y < PLOT_Y0 + PLOT_H) {
      int half = (int)traceThick / 2;
      int y0 = y - half;
      int y1 = y0 + traceThick - 1;
      if (y0 < PLOT_Y0) y0 = PLOT_Y0;
      if (y1 >= PLOT_Y0 + PLOT_H) y1 = PLOT_Y0 + PLOT_H - 1;
      for (int yy = y0; yy <= y1; ++yy) tft.drawPixel(plotX, yy, COL_TRACE);

      // redraw dashed midline over it (if visible)
      if (gShowMidline) {
        int phase = (plotX - PLOT_X0) % (DASH_ON + DASH_OFF);
        if (phase < DASH_ON) tft.drawPixel(plotX, midY, COL_GRID);
      }

      lastY[x] = y;
      lastThk[x] = traceThick;
    } else {
      lastY[x] = -1;
      lastThk[x] = traceThick;
    }

    // advance one pixel; wrap cleanly
    ++x;
    if (x >= PLOT_W) x = 0;
  }

  // VU at ~100 Hz
  static uint32_t lastVU = 0;
  uint32_t now = millis();
  if (now - lastVU >= 10) { lastVU = now; updateVU(); }
}
// ==================== end main.cpp ====================
