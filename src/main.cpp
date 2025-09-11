// ==================== main.cpp (ESP32 + ILI9341, ZOH + interpolated render) ====================
#include <Arduino.h>
#include <SPI.h>
#include <Ticker.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// --- your custom fonts ---
#include "Aurora4pt7b.h"  // small font: HUD, axis labels (aurora_244pt7b)
#include "Aurora7pt7b.h"  // title font:                    (aurora_247pt7b)
// If you also have a 10pt header, keep it unused or remove the include.
// #include "Aurora10pt7b.h"

// -------------------- USER SETTINGS --------------------
// TFT pins (change if your wiring differs)
#define TFT_CS   5
#define TFT_DC   21
#define TFT_RST  22
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_MISO 19 // if your TFT has SDO; else use 5-arg ctor

// Quick RGB565 helper for custom colors
static inline uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// ----------- Color palette -----------
uint16_t COL_BG    = ILI9341_BLACK;
uint16_t COL_TEXT  = ILI9341_WHITE;         // HUD + axis labels
uint16_t COL_TITLE = RGB565(244, 206, 39);  // yellow title (#f4ce27)
uint16_t COL_AXIS  = ILI9341_WHITE;
uint16_t COL_TICKS = ILI9341_WHITE;
uint16_t COL_GRID  = RGB565(90, 90, 110);
uint16_t COL_TRACE = ILI9341_WHITE;
// -------------------------------------

// Mic input pin (ADC1 preferred). 36 = ADC1_CH0
#define MIC_PIN 36

// Screen geometry (rotation set to 1 => 320x240)
constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 240;

// --- Plot layout (title area + x-axis area + bottom banner) ---
constexpr int PLOT_TOPBANNER    = 24; // top title region
constexpr int XAXIS_HEIGHT      = 12; // reserved strip under plot for x-axis ticks/labels
constexpr int PLOT_BOTTOMBANNER = 20; // bottom HUD
constexpr int PLOT_LMARGIN      = 38; // left margin for Y axis & labels

constexpr int PLOT_X0 = PLOT_LMARGIN;
constexpr int PLOT_Y0 = PLOT_TOPBANNER;
constexpr int PLOT_W  = SCREEN_W - PLOT_X0;
constexpr int PLOT_H  = SCREEN_H - (PLOT_TOPBANNER + PLOT_BOTTOMBANNER + XAXIS_HEIGHT);

// VU LED pins (12 outputs)
constexpr uint8_t Level1=2, Level2=4, Level3=12, Level4=13, Level5=14, Level6=15,
                  Level7=25, Level8=26, Level9=27, Level10=32, Level11=33, Level12=16;

// -------------------- GLOBALS --------------------
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);
// If your TFT has no SDO/MISO, you can use: Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

Ticker micTicker;
volatile bool micTickFlag = false;

// Sampling & visualization
volatile uint32_t gSampleFreqHz = 5000; // default Fs
constexpr uint32_t FS_MIN = 1000;
constexpr uint32_t FS_MAX = 20000;

volatile uint8_t zoh = 2;               // default ZOH span (samples/pixels)
constexpr uint8_t ZOH_MIN = 1;
constexpr uint8_t ZOH_MAX = 10;

volatile uint8_t traceThick = 1;        // default thickness (px)
constexpr uint8_t THICK_MIN  = 1;
constexpr uint8_t THICK_MAX  = 4;

// Midline toggle
volatile bool gShowMidline = false;

volatile int readings[PLOT_W]; // for VU (stash current span value per column)
uint16_t DCOffset = 0;         // average mic DC in raw ADC units (0..4095)
int midY = 0;                  // y of ~1.65 V midline

// Dashed midline pattern (pixels)
constexpr int DASH_ON  = 6;
constexpr int DASH_OFF = 6;

// Interpolated ZOH state (1-span lag)
volatile int s_prevRaw = 0;
volatile int s_currRaw = 0;
volatile int s_nextRaw = 0;
volatile uint8_t interpIdx = 0; // 0..zoh-1

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

// pixels (columns) per second
static inline float colsPerSec() {
  return (float)gSampleFreqHz * (float)zoh;
}

// Ticker at Fs * zoh (one pixel per tick)
void applyTicker() {
  float rate = colsPerSec();
  if (rate < (float)FS_MIN)   rate = (float)FS_MIN;
  if (rate > (float)FS_MAX * ZOH_MAX) rate = (float)FS_MAX * ZOH_MAX;
  micTicker.detach();
  micTicker.attach(1.0f / rate, onMicTick);
}

void startSampling(uint32_t freqHz) {
  if (freqHz < FS_MIN) freqHz = FS_MIN;
  if (freqHz > FS_MAX) freqHz = FS_MAX;
  gSampleFreqHz = freqHz;
  applyTicker();
}

uint16_t estimateDCoffset(int numSamples) {
  uint32_t sum = 0;
  for (int i = 0; i < numSamples; ++i) {
    sum += analogRead(MIC_PIN);
    delay(2);
  }
  return (uint16_t)(sum / (uint32_t)numSamples);
}

// -------------------- DRAWING --------------------
void drawTitle() {
  const char *title = "Audio Signal Visualiser";
  tft.setFont(&aurora_247pt7b);
  int16_t x1, y1; uint16_t tw, th;
  tft.getTextBounds(title, 0, 0, &x1, &y1, &tw, &th);

  // Center over the plot width (from the Y axis to RHS)
  int x = PLOT_X0 + ((int)PLOT_W - (int)tw) / 2;
  int baselineY = (PLOT_TOPBANNER - (int)th) / 2 + (int)th;

  tft.setTextColor(COL_TITLE, COL_BG);
  tft.setCursor(x, baselineY);
  tft.print(title);
}

void drawBottomBannerHUD() {
  // Clear ONLY the plot part of the bottom banner (avoid left margin)
  tft.fillRect(PLOT_X0, SCREEN_H - PLOT_BOTTOMBANNER, PLOT_W, PLOT_BOTTOMBANNER, COL_BG);

  tft.setFont(&aurora_244pt7b);
  tft.setTextColor(COL_TEXT, COL_BG);

  char fsBuf[28]; snprintf(fsBuf, sizeof(fsBuf), "Fs: %.1fkHz", gSampleFreqHz / 1000.0f);
  char zBuf[28];  snprintf(zBuf, sizeof(zBuf),  "ZOH: %u", (unsigned)zoh);
  char thBuf[28]; snprintf(thBuf, sizeof(thBuf), "Thk: %upx", (unsigned)traceThick);

  int16_t x1, y1; uint16_t wFs, hFs, wZ, hZ, wTh, hTh;
  tft.getTextBounds(fsBuf, 0, 0, &x1, &y1, &wFs, &hFs);
  tft.getTextBounds(zBuf,  0, 0, &x1, &y1, &wZ,  &hZ);
  tft.getTextBounds(thBuf, 0, 0, &x1, &y1, &wTh, &hTh);

  const int gap = 16;
  int totalW = (int)wFs + gap + (int)wZ + gap + (int)wTh;
  int startX = PLOT_X0 + (PLOT_W - totalW) / 2;

  int textH = max((int)hFs, max((int)hZ, (int)hTh));
  int baselineY = SCREEN_H - (PLOT_BOTTOMBANNER - textH) / 2;

  int x = startX;
  tft.setCursor(x, baselineY); tft.print(fsBuf);
  x += wFs + gap;
  tft.setCursor(x, baselineY); tft.print(zBuf);
  x += wZ + gap;
  tft.setCursor(x, baselineY); tft.print(thBuf);
}

void drawYAxisScale() {
  // Left strip & axis line
  tft.fillRect(0, PLOT_Y0, PLOT_LMARGIN, PLOT_H, COL_BG);
  tft.drawFastVLine(PLOT_LMARGIN - 1, PLOT_Y0, PLOT_H, COL_AXIS);

  auto yForVolt = [](float v) -> int {
    int raw = (int)roundf((v / 3.3f) * 4095.0f);
    return adcToY_raw(raw);
  };

  tft.setFont(&aurora_244pt7b);
  tft.setTextColor(COL_TEXT, COL_BG);

  // Minor ticks every 0.1V; major every 0.5V
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
      dtostrf(v, 3, 1, buf);
      strcat(buf, "V");

      int16_t x1, y1; uint16_t tw, th;
      tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);

      int tx = (PLOT_LMARGIN - 3) - (int)tw;
      int baselineY = y + (int)th / 2;

      // clear a box to avoid overdraw on tick
      int boxX = tx - 2;
      int boxW = (PLOT_LMARGIN - 2) - boxX;
      int boxY = baselineY - (int)th - 1;
      int boxH = (int)th + 2;
      tft.fillRect(boxX, boxY, boxW, boxH, COL_BG);

      tft.setCursor(tx, baselineY);
      tft.print(buf);
    }
  }

  // Ensure 3.3V shows
  float v = 3.3f;
  int y = yForVolt(v);
  if (y >= PLOT_Y0 && y < PLOT_Y0 + PLOT_H) {
    char buf[12];
    dtostrf(v, 3, 1, buf); strcat(buf, "V");
    int16_t x1, y1; uint16_t tw, th;
    tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);
    int tx = (PLOT_LMARGIN - 3) - (int)tw;
    int baselineY = y + (int)th / 2;
    int boxX = tx - 2, boxW = (PLOT_LMARGIN - 2) - boxX;
    int boxY = baselineY - (int)th - 1, boxH = (int)th + 2;
    tft.fillRect(boxX, boxY, boxW, boxH, COL_BG);
    tft.setCursor(tx, baselineY); tft.print(buf);
  }
}

void drawPlotBackground() {
  tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, PLOT_H, COL_BG);

  // Dashed midline (~1.65V)
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

// X-axis time scale under plot (drawn AFTER plot & BEFORE bottom banner)
void drawXAxisScale() {
  const int bandY = PLOT_Y0 + PLOT_H;   // top of x-axis band
  // clear x-axis strip
  tft.fillRect(PLOT_X0, bandY, PLOT_W, XAXIS_HEIGHT, COL_BG);

  // axis baseline
  tft.drawFastHLine(PLOT_X0, bandY, PLOT_W, COL_AXIS);

  // time per pixel
  const float cps = colsPerSec();   // pixels per second
  const float dt  = 1.0f / cps;     // sec/pixel
  const float T   = dt * PLOT_W;    // total span

  // choose a "nice" major tick step targeting ~50 px
  static const float steps[] = {
    0.1e-3f, 0.2e-3f, 0.5e-3f,
    1e-3f,   2e-3f,   5e-3f,
    10e-3f,  20e-3f,  50e-3f,
    0.1f,    0.2f,    0.5f,  1.0f,  2.0f
  };
  const float targetPx = 50.0f;

  float step = steps[0];
  float best = 1e9f;
  for (size_t i=0;i<sizeof(steps)/sizeof(steps[0]);++i) {
    float px = steps[i] / dt;
    float diff = fabsf(px - targetPx);
    if (diff < best) { best = diff; step = steps[i]; }
  }
  int pxMajor = (int)roundf(step / dt);
  if (pxMajor < 16) pxMajor = 16;   // avoid clutter

  // label style
  tft.setFont(&aurora_244pt7b);
  tft.setTextColor(COL_TEXT, COL_BG);

  // vertically centered label baseline inside band
  int16_t bx1, by1; uint16_t btw, bth;
  tft.getTextBounds("Ag", 0, 0, &bx1, &by1, &btw, &bth);
  const int baselineY = bandY + (XAXIS_HEIGHT - (int)bth)/2 + (int)bth;

  // draw ticks/labels
  for (int xpix = 0; xpix <= PLOT_W; xpix += pxMajor) {
    const int x = PLOT_X0 + xpix;

    // major tick into the band
    tft.drawFastVLine(x, bandY, 6, COL_TICKS);

    // time at this position
    float t = xpix * dt;  // seconds
    char lab[20];
    if      (T < 5e-3f)  snprintf(lab, sizeof(lab), "%.0fus", t*1e6f);
    else if (T < 1.0f)   snprintf(lab, sizeof(lab), "%.0fms", t*1e3f);
    else                 snprintf(lab, sizeof(lab), "%.1fs",  t);

    int16_t x1,y1; uint16_t tw,th;
    tft.getTextBounds(lab, 0, 0, &x1, &y1, &tw, &th);

    // centered label; clamp within band width
    int tx = x - (int)tw/2;
    if (tx < PLOT_X0) tx = PLOT_X0;
    if (tx + (int)tw > PLOT_X0 + PLOT_W) tx = PLOT_X0 + PLOT_W - (int)tw;

    // clear a small box so text doesn’t clash with baseline
    tft.fillRect(tx-1, baselineY - (int)th - 1, (int)tw + 2, (int)th + 2, COL_BG);
    tft.setCursor(tx, baselineY);
    tft.print(lab);

    // minor tick halfway if it fits
    int xm = x + pxMajor/2;
    if (xm < PLOT_X0 + PLOT_W)
      tft.drawFastVLine(xm, bandY, 3, COL_TICKS);
  }
}

// -------------------- SETTINGS TWEAKERS --------------------
void setZOH(uint8_t n) {
  if (n < ZOH_MIN) n = ZOH_MIN;
  if (n > ZOH_MAX) n = ZOH_MAX;
  zoh = n;
  interpIdx = 0;
  applyTicker();
  drawPlotBackground();
  drawXAxisScale();
  drawBottomBannerHUD();
}

void setTraceThick(uint8_t newThick) {
  if (newThick < THICK_MIN) newThick = THICK_MIN;
  if (newThick > THICK_MAX) newThick = THICK_MAX;
  if (newThick == traceThick) return;
  traceThick = newThick;
  drawBottomBannerHUD();
}

void setSampleFreq(uint32_t newFs) {
  if (newFs < FS_MIN) newFs = FS_MIN;
  if (newFs > FS_MAX) newFs = FS_MAX;
  startSampling(newFs);
  drawPlotBackground();
  drawXAxisScale();
  drawBottomBannerHUD();
}

// --- serial parser ---
//  f8000 / fs=12000      → set Fs (Hz)
//  z / Z                 → ZOH - / +
//  { / }                 → thickness - / +
//  m                     → toggle midline
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
    } else {
      Serial.println(F("Parse Fs failed. Use: f8000 or fs=12000"));
    }
    return;
  }

  int c = Serial.read();
  if (c == 'z') setZOH(zoh > ZOH_MIN ? (uint8_t)(zoh - 1) : ZOH_MIN);
  if (c == 'Z') setZOH(zoh < ZOH_MAX ? (uint8_t)(zoh + 1) : ZOH_MAX);
  if (c == '{') setTraceThick(traceThick > THICK_MIN ? (uint8_t)(traceThick - 1) : THICK_MIN);
  if (c == '}') setTraceThick(traceThick < THICK_MAX ? (uint8_t)(traceThick + 1) : THICK_MAX);
  if (c == 'm' || c == 'M') { gShowMidline = !gShowMidline; drawPlotBackground(); drawXAxisScale(); }
}

// -------------------- VU OUTPUTS --------------------
void initVU() {
  const uint8_t pins[12] = {Level1, Level2, Level3, Level4, Level5, Level6,
                            Level7, Level8, Level9, Level10, Level11, Level12};
  for (uint8_t i = 0; i < 12; ++i) pinMode(pins[i], OUTPUT);
}
void setVU(uint8_t level) { // 0..12
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
  else                  VUlevel = 0;
  setVU(VUlevel);
}

// -------------------- SETUP / LOOP --------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("Commands: f8000 | fs=12000 | z/Z ZOH | { } thick | m midline"));

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);
  pinMode(MIC_PIN, INPUT);

  // TFT init
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(COL_BG);

  // Static UI
  drawTitle();
  drawYAxisScale();
  drawPlotBackground();
  drawXAxisScale();
  drawBottomBannerHUD();

  initVU();

  for (int i = 0; i < PLOT_W; ++i) readings[i] = 0;

  // --- DC offset message inside plot area, centered ---
  tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, PLOT_H, COL_BG);
  tft.setFont(&aurora_247pt7b);
  tft.setTextColor(COL_TEXT, COL_BG);
  const char *msg = "Measuring DC Offset...";
  int w, h; measureText(&aurora_247pt7b, msg, &w, &h);
  int msgX = PLOT_X0 + (PLOT_W - w) / 2;
  int msgY = PLOT_Y0 + (PLOT_H + h) / 2;
  tft.setCursor(msgX, msgY); tft.print(msg);

  DCOffset = estimateDCoffset(256);

  // show measured value briefly
  tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, PLOT_H, COL_BG);
  char line[32]; snprintf(line, sizeof(line), "DC: %.2f V", (DCOffset / 4095.0f) * 3.3f);
  measureText(&aurora_247pt7b, line, &w, &h);
  int valX = PLOT_X0 + (PLOT_W - w) / 2;
  int valY = PLOT_Y0 + (PLOT_H + h) / 2;
  tft.setCursor(valX, valY); tft.print(line);
  delay(600);

  // Redraw background & UI
  drawPlotBackground();
  drawXAxisScale();
  drawBottomBannerHUD();

  // Seed the 3-sample pipeline for interpolated ZOH
  s_prevRaw = analogRead(MIC_PIN); delay(1);
  s_currRaw = analogRead(MIC_PIN); delay(1);
  s_nextRaw = analogRead(MIC_PIN);
  interpIdx = 0;

  // Start sampling at defaults
  startSampling(gSampleFreqHz);
}

void loop() {
  static int x = 0;               // column within plot area [0..PLOT_W-1]
  static int lastY[PLOT_W];       // last center y for erasing
  static uint8_t lastThk[PLOT_W]; // last thickness used at that column
  static bool inited = false;

  if (!inited) {
    for (int i = 0; i < PLOT_W; ++i) { lastY[i] = -1; lastThk[i] = traceThick; }
    inited = true;
  }

  handleSerial();

  if (micTickFlag) {
    micTickFlag = false;

    int plotX = PLOT_X0 + x;

    // erase previous vertical stroke at this column
    if (lastY[x] >= PLOT_Y0 && lastY[x] < PLOT_Y0 + PLOT_H) {
      int halfPrev = (int)lastThk[x] / 2;
      int y0 = lastY[x] - halfPrev;
      int y1 = y0 + lastThk[x] - 1;
      if (y0 < PLOT_Y0) y0 = PLOT_Y0;
      if (y1 >= PLOT_Y0 + PLOT_H) y1 = PLOT_Y0 + PLOT_H - 1;
      for (int yy = y0; yy <= y1; ++yy) tft.drawPixel(plotX, yy, backgroundAt(plotX, yy));
    }

    // Interpolated ZOH rendering (lag one span)
    int yPrev = adcToY_raw(s_prevRaw);
    int yCurr = adcToY_raw(s_currRaw);
    int dy = yCurr - yPrev;

    int num = (int)interpIdx + 1; // 1..zoh
    int y = yPrev + ((dy * num + (int)zoh/2) / (int)zoh);

    // draw this pixel's vertical stroke
    if (y >= PLOT_Y0 && y < PLOT_Y0 + PLOT_H) {
      int half = (int)traceThick / 2;
      int y0 = y - half;
      int y1 = y0 + traceThick - 1;
      if (y0 < PLOT_Y0) y0 = PLOT_Y0;
      if (y1 >= PLOT_Y0 + PLOT_H) y1 = PLOT_Y0 + PLOT_H - 1;
      for (int yy = y0; yy <= y1; ++yy) tft.drawPixel(plotX, yy, COL_TRACE);

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

    // VU metering can use the current sample of the span
    readings[x] = s_currRaw;

    // Advance interpolation phase & sample pipeline when span completes
    ++interpIdx;
    if (interpIdx >= zoh) {
      interpIdx = 0;
      s_prevRaw = s_currRaw;
      s_currRaw = s_nextRaw;
      s_nextRaw = analogRead(MIC_PIN);
    }

    // Advance pixel column; wrap cleanly
    ++x; if (x >= PLOT_W) x = 0;
  }

  // Update VU at ~100 Hz
  static uint32_t lastVU = 0;
  uint32_t now = millis();
  if (now - lastVU >= 10) { lastVU = now; updateVU(); }
}
// ==================== end main.cpp ====================
