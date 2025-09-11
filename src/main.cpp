// ==================== main.cpp (ESP32 + ILI9341, dual Aurora fonts) ====================
#include <Arduino.h>
#include <SPI.h>
#include <Ticker.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// --- your custom fonts ---
#include "Aurora4pt7b.h"  // small font: HUD, axis labels
#include "Aurora7pt7b.h"  // large font: title
#include "Aurora10pt7b.h" // really large

// -------------------- USER SETTINGS --------------------aaa
// TFT pins (change if your wiring differs)
#define TFT_CS 5
#define TFT_DC 21
#define TFT_RST 22
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_MISO 19 // if your TFT has SDO; else use 5-arg ctor

// Quick RGB565 helper for custom colors
static inline uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// ----------- Color palette -----------
uint16_t COL_BG = ILI9341_BLACK;
uint16_t COL_TEXT = ILI9341_WHITE;         // HUD + axis labels
uint16_t COL_TITLE = RGB565(244, 206, 39); // yellow title text  (#f4ce27)
uint16_t COL_AXIS = ILI9341_WHITE;
uint16_t COL_TICKS = ILI9341_WHITE;
uint16_t COL_GRID = RGB565(90, 90, 110);
uint16_t COL_TRACE = ILI9341_WHITE;
// ------------------------------------------------------

// Mic input pin (ADC1 preferred). 36 = ADC1_CH0
#define MIC_PIN 36

// Screen geometry (rotation set to 1 => 320x240)
constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 240;

// --- Plot layout with top/bottom banners ---
constexpr int PLOT_TOPBANNER = 24; // a tad taller to fit big font title
constexpr int PLOT_BOTTOMBANNER = 20;
constexpr int PLOT_LMARGIN = 38; // left margin for Y axis & labels

constexpr int PLOT_X0 = PLOT_LMARGIN;
constexpr int PLOT_Y0 = PLOT_TOPBANNER;
constexpr int PLOT_W = SCREEN_W - PLOT_X0;
constexpr int PLOT_H = SCREEN_H - (PLOT_TOPBANNER + PLOT_BOTTOMBANNER);

// VU LED pins (12 outputs)
constexpr uint8_t Level1 = 2, Level2 = 4, Level3 = 12, Level4 = 13, Level5 = 14, Level6 = 15,
                  Level7 = 25, Level8 = 26, Level9 = 27, Level10 = 32, Level11 = 33, Level12 = 16;

// -------------------- GLOBALS --------------------
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);
// If your TFT has no SDO/MISO, you can use: Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

Ticker micTicker;
volatile bool micTickFlag = false;

// Adjustable sample frequency (Hz) via serial
volatile uint32_t gSampleFreqHz = 2500; // default
constexpr uint32_t FS_MIN = 1000;
constexpr uint32_t FS_MAX = 10000;

// Horizontal stride (how many pixels we advance per plotted sample)
volatile uint8_t xStep = 2; // 1 = every pixel, 2 = every 2 pixels, etc.
constexpr uint8_t XSTEP_MIN = 1;
constexpr uint8_t XSTEP_MAX = 6;

// Trace thickness (vertical pixels). Adjustable.
volatile uint8_t traceThick = 2;
constexpr uint8_t THICK_MIN = 1;
constexpr uint8_t THICK_MAX = 4;

// Midline toggle
volatile bool gShowMidline = false;

volatile int readings[PLOT_W]; // circular buffer in plot width
uint16_t DCOffset = 0;         // average mic DC in raw ADC units (0..4095)
int midY = 0;                  // y of ~1.65 V midline

// Dashed midline pattern (pixels)
constexpr int DASH_ON = 6;
constexpr int DASH_OFF = 6;

// -------------------- HELPERS --------------------
// Text measurement helper for any GFXfont
void measureText(const GFXfont *font, const char *text, int *w, int *h)
{
  int16_t x1, y1;
  uint16_t tw, th;
  tft.setFont(font);
  tft.getTextBounds(text, 0, 0, &x1, &y1, &tw, &th);
  *w = (int)tw;
  *h = (int)th;
}

static inline int adcToY_raw(int raw)
{
  if (raw < 0)
    raw = 0;
  if (raw > 4095)
    raw = 4095;
  // scale into plot height, then offset by top banner
  int y = PLOT_Y0 + PLOT_H - 1 - ((uint32_t)raw * PLOT_H >> 12);
  return y;
}

// Background color at a plot pixel (restores dashed midline instead of erasing it)
uint16_t backgroundAt(int plotX, int y)
{
  if (y < PLOT_Y0 || y >= PLOT_Y0 + PLOT_H)
    return COL_BG;
  if (gShowMidline && y == midY)
  {
    int xInPlot = plotX - PLOT_X0;
    int phase = xInPlot % (DASH_ON + DASH_OFF);
    if (phase < DASH_ON)
      return COL_GRID;
  }
  return COL_BG;
}

void IRAM_ATTR onMicTick() { micTickFlag = true; }

void startSampling(uint32_t freqHz)
{
  if (freqHz < FS_MIN)
    freqHz = FS_MIN;
  if (freqHz > FS_MAX)
    freqHz = FS_MAX;
  gSampleFreqHz = freqHz;
  micTicker.detach();
  micTicker.attach(1.0f / (float)gSampleFreqHz, onMicTick);
}

uint16_t estimateDCoffset(int numSamples)
{
  uint32_t sum = 0;
  for (int i = 0; i < numSamples; ++i)
  {
    sum += analogRead(MIC_PIN);
    delay(2);
  }
  return (uint16_t)(sum / (uint32_t)numSamples);
}

// -------------------- DRAWING --------------------
void drawTitleBanner()
{
  const char *title = "Audio Signal Visualiser";

  tft.setFont(&aurora_247pt7b);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds(title, 0, 0, &x1, &y1, &tw, &th);

  // Center only over the plot width
  int x = PLOT_X0 + (int(PLOT_W) - int(tw)) / 2;

  // Baseline so it sits nicely inside the top margin
  int baselineY = (PLOT_TOPBANNER - int(th)) / 2 + th;

  tft.setTextColor(COL_TITLE, COL_BG); // yellow text, black bg
  tft.setCursor(x, baselineY);
  tft.print(title);
}

void drawBottomBannerHUD()
{
  // Clear ONLY the plot part of the bottom banner (avoid left margin box)
  tft.fillRect(PLOT_X0, SCREEN_H - PLOT_BOTTOMBANNER, PLOT_W, PLOT_BOTTOMBANNER, COL_BG);

  tft.setFont(&aurora_244pt7b);
  tft.setTextColor(COL_TEXT, COL_BG);

  // Text we want to show
  char fsBuf[24];
  snprintf(fsBuf, sizeof(fsBuf), "Fs: %.1fkHz", gSampleFreqHz / 1000.0f);

  char xsBuf[24];
  snprintf(xsBuf, sizeof(xsBuf), "Xstep: %upx", (unsigned)xStep);

  char thBuf[24];
  snprintf(thBuf, sizeof(thBuf), "Thk: %upx", (unsigned)traceThick);

  // Measure each chunk
  int16_t x1, y1;
  uint16_t wFs, hFs, wXs, hXs, wTh, hTh;
  tft.getTextBounds(fsBuf, 0, 0, &x1, &y1, &wFs, &hFs);
  tft.getTextBounds(xsBuf, 0, 0, &x1, &y1, &wXs, &hXs);
  tft.getTextBounds(thBuf, 0, 0, &x1, &y1, &wTh, &hTh);

  // Spacing between blocks
  const int gap = 16;
  int totalW = (int)wFs + gap + (int)wXs + gap + (int)wTh;

  // Center across the plot area (not the entire screen)
  int startX = PLOT_X0 + (PLOT_W - totalW) / 2;

  // Baseline vertically centered inside the banner height
  // Use the tallest chunk's height for baseline placement
  int textH = max((int)hFs, max((int)hXs, (int)hTh));
  int baselineY = SCREEN_H - (PLOT_BOTTOMBANNER - textH) / 2;

  // Draw
  int x = startX;
  tft.setCursor(x, baselineY);
  tft.print(fsBuf);

  x += wFs + gap;
  tft.setCursor(x, baselineY);
  tft.print(xsBuf);

  x += wXs + gap;
  tft.setCursor(x, baselineY);
  tft.print(thBuf);
}

void drawYAxisScale()
{
  // Left scale strip (within plot band)
  tft.fillRect(0, PLOT_Y0, PLOT_LMARGIN, PLOT_H, COL_BG);

  // Axis line
  tft.drawFastVLine(PLOT_LMARGIN - 1, PLOT_Y0, PLOT_H, COL_AXIS);

  auto yForVolt = [](float v) -> int
  {
    int raw = (int)roundf((v / 3.3f) * 4095.0f);
    return adcToY_raw(raw);
  };

  tft.setFont(&aurora_244pt7b);
  tft.setTextColor(COL_TEXT, COL_BG);

  // Minor ticks every 0.1 V, major every 0.5 V
  for (int i = 0; i <= 33; ++i)
  {
    float v = i * 0.1f;
    int y = yForVolt(v);
    if (y < PLOT_Y0 || y >= PLOT_Y0 + PLOT_H)
      continue;

    bool major = (i % 5 == 0);
    int tickLen = major ? 7 : 4;
    int xStart = PLOT_LMARGIN - 1 - tickLen;
    tft.drawFastHLine(xStart, y, tickLen, major ? COL_AXIS : COL_TICKS);

    if (major)
    {
      // Text like "1.5V"
      char buf[12];
      dtostrf(v, 3, 1, buf);
      strcat(buf, "V");

      // Measure text
      int16_t x1, y1;
      uint16_t tw, th;
      tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);

      // Right-align, a couple pixels left of the axis
      int tx = (PLOT_LMARGIN - 3) - (int)tw;
      // Approx baseline to center around tick
      int baselineY = y + (int)th / 2;

      // *** Clear a background box from label to the axis ***
      int boxX = tx - 2;
      int boxW = (PLOT_LMARGIN - 2) - boxX; // reaches up to just left of axis
      int boxY = baselineY - (int)th - 1;
      int boxH = (int)th + 2;
      tft.fillRect(boxX, boxY, boxW, boxH, COL_BG);

      // Print label
      tft.setCursor(tx, baselineY);
      tft.print(buf);
    }
  }

  // *** Ensure 3.3 V shows even if rounding pushes it out ***
  {
    float v = 3.3f;
    int y = yForVolt(v);
    if (y >= PLOT_Y0 && y < PLOT_Y0 + PLOT_H)
    {
      char buf[12];
      dtostrf(v, 3, 1, buf);
      strcat(buf, "V");
      int16_t x1, y1;
      uint16_t tw, th;
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
}

void drawPlotBackground()
{
  // Clear plot area only; leave banners & left scale untouched
  tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, PLOT_H, COL_BG);

  // Dashed midline (~1.65V)
  midY = adcToY_raw((int)roundf((1.65f / 3.3f) * 4095.0f));
  if (gShowMidline)
  {
    int x = PLOT_X0;
    while (x < PLOT_X0 + PLOT_W)
    {
      int run = min(DASH_ON, PLOT_X0 + PLOT_W - x);
      tft.drawFastHLine(x, midY, run, COL_GRID);
      x += DASH_ON + DASH_OFF;
    }
  }
}

// -------------------- SETTINGS TWEAKERS --------------------
static inline uint8_t snapXStep(uint8_t v)
{
  if (v <= 1)
    return 1; // allow the special 1px mode
  if (v >= XSTEP_MAX)
    return XSTEP_MAX; // clamp to max
  // force even for anything other than 1
  return (v & 1) ? (uint8_t)(v + 1) : v; // make it even (2,4,6…)
}

static inline uint8_t nextXStepUp(uint8_t v)
{
  if (v < 1)
    return 1;
  if (v == 1)
    return 2; // 1 -> 2
  if (v >= XSTEP_MAX)
    return XSTEP_MAX; // stay at max
  uint8_t n = v + 2;  // even step
  return n > XSTEP_MAX ? XSTEP_MAX : n;
}

static inline uint8_t nextXStepDown(uint8_t v)
{
  if (v <= 1)
    return 1; // stay at 1
  if (v == 2)
    return 1;        // 2 -> 1
  uint8_t n = v - 2; // even step
  return n < 2 ? 1 : n;
}

void setXStep(uint8_t newStep)
{
  newStep = snapXStep(newStep);
  if (newStep == xStep)
    return;
  xStep = newStep;
  drawBottomBannerHUD();
  drawPlotBackground();
  Serial.print(F("xStep="));
  Serial.println(xStep);
}

void setTraceThick(uint8_t newThick)
{
  if (newThick < THICK_MIN)
    newThick = THICK_MIN;
  if (newThick > THICK_MAX)
    newThick = THICK_MAX;
  if (newThick == traceThick)
    return;
  traceThick = newThick;
  drawBottomBannerHUD();
  drawPlotBackground();
}
void setSampleFreq(uint32_t newFs)
{
  if (newFs < FS_MIN)
    newFs = FS_MIN;
  if (newFs > FS_MAX)
    newFs = FS_MAX;
  startSampling(newFs);
  drawBottomBannerHUD();
}

// --- simple serial parser for Fs/Xstep/Thk + midline toggle ---
//  f8000  (or fs=12000) to set sample freq
//  [ and ] to change xStep
//  { and } to change thickness
//  m toggles midline
void handleSerial()
{
  if (!Serial.available())
    return;

  int peekc = Serial.peek();
  if (peekc == 'f' || peekc == 'F')
  {
    String line = Serial.readStringUntil('\n'); // e.g. "f8000" or "fs=12000"
    long val = -1;
    String num = "";
    for (size_t i = 0; i < line.length(); ++i)
    {
      if (isDigit((unsigned char)line[i]))
        num += line[i];
      else if (num.length() && !isDigit((unsigned char)line[i]))
        break;
    }
    if (num.length())
      val = num.toInt();
    if (val > 0)
    {
      setSampleFreq((uint32_t)val);
      Serial.print(F("Fs set to "));
      Serial.println((uint32_t)val);
    }
    else
    {
      Serial.println(F("Parse Fs failed. Use: f8000 or fs=12000"));
    }
    return;
  }

  int c = Serial.read();
  if (c == '[')
    setXStep(nextXStepDown(xStep)); // decrease (…6,4,2,1)
  if (c == ']')
    setXStep(nextXStepUp(xStep)); // increase (1,2,4,6)
  if (c == '{')
    setTraceThick(traceThick > THICK_MIN ? traceThick - 1 : THICK_MIN);
  if (c == '}')
    setTraceThick(traceThick < THICK_MAX ? traceThick + 1 : THICK_MAX);
  if (c == 'm' || c == 'M')
  {
    gShowMidline = !gShowMidline;
    drawPlotBackground();
  }
}

// -------------------- VU OUTPUTS --------------------
void initVU()
{
  const uint8_t pins[12] = {Level1, Level2, Level3, Level4, Level5, Level6,
                            Level7, Level8, Level9, Level10, Level11, Level12};
  for (uint8_t i = 0; i < 12; ++i)
    pinMode(pins[i], OUTPUT);
}
void setVU(uint8_t level)
{ // 0..12
  const uint8_t pins[12] = {Level1, Level2, Level3, Level4, Level5, Level6,
                            Level7, Level8, Level9, Level10, Level11, Level12};
  for (uint8_t i = 0; i < 12; ++i)
    digitalWrite(pins[i], (i < level) ? HIGH : LOW);
}
void updateVU()
{
  int maxRead = 0;
  for (int i = 0; i < PLOT_W; ++i)
    if (readings[i] > maxRead)
      maxRead = readings[i];
  uint16_t amp10 = (uint16_t)abs(maxRead - (int)DCOffset) >> 2; // 0..1023

  uint8_t VUlevel = 0;
  if (amp10 > 446)
    VUlevel = 12;
  else if (amp10 > 385)
    VUlevel = 11;
  else if (amp10 > 328)
    VUlevel = 10;
  else if (amp10 > 288)
    VUlevel = 9;
  else if (amp10 > 228)
    VUlevel = 8;
  else if (amp10 > 185)
    VUlevel = 7;
  else if (amp10 > 146)
    VUlevel = 6;
  else if (amp10 > 111)
    VUlevel = 5;
  else if (amp10 > 82)
    VUlevel = 4;
  else if (amp10 > 57)
    VUlevel = 3;
  else if (amp10 > 36)
    VUlevel = 2;
  else if (amp10 > 20)
    VUlevel = 1;
  else
    VUlevel = 0;
  setVU(VUlevel);
}

// -------------------- SETUP / LOOP --------------------
void setup()
{
  Serial.begin(115200);
  Serial.println(F("Commands: f8000 | fs=12000 | [ ] xstep | { } thick | m midline"));

  // ADC: 12-bit and ~0..3.3V range
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);
  pinMode(MIC_PIN, INPUT);

  // TFT init
  tft.begin();
  tft.setRotation(1); // 320x240

  // Clean background first to prevent clipping artifacts
  tft.fillScreen(COL_BG);

  // Draw all static UI
  drawTitleBanner();
  drawYAxisScale();
  drawPlotBackground();
  drawBottomBannerHUD();

  initVU();

  // Clear buffers
  for (int i = 0; i < PLOT_W; ++i)
    readings[i] = 0;

  // --- DC offset message inside plot area, centered in screen ---
  tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, PLOT_H, COL_BG); // clear plot area

  tft.setFont(&aurora_247pt7b);
  tft.setTextColor(COL_TEXT, COL_BG);

  const char *msg = "Measuring DC Offset...";
  int w, h;
  measureText(&aurora_247pt7b, msg, &w, &h);

  // Center horizontally and vertically within the plot area
  int msgX = PLOT_X0 + (PLOT_W - w) / 2;
  int msgY = PLOT_Y0 + (PLOT_H + h) / 2;

  tft.setCursor(msgX, msgY);
  tft.print(msg);

  // Take samples
  DCOffset = estimateDCoffset(256);

  // --- show measured value ---
  tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, PLOT_H, COL_BG);

  char line[32];
  snprintf(line, sizeof(line), "DC: %.2f V", (DCOffset / 4095.0f) * 3.3f);
  measureText(&aurora_247pt7b, line, &w, &h);

  int valX = PLOT_X0 + (PLOT_W - w) / 2;
  int valY = PLOT_Y0 + (PLOT_H + h) / 2;

  tft.setCursor(valX, valY);
  tft.print(line);

  delay(1000);

  // Redraw background to clean the banner area we used
  drawPlotBackground();
  drawBottomBannerHUD();

  // Start sampling at default
  startSampling(gSampleFreqHz);
}

void loop()
{
  static int x = 0;               // column within plot area [0..PLOT_W-1]
  static int lastY[PLOT_W];       // last center y for erasing
  static uint8_t lastThk[PLOT_W]; // last thickness used at that column
  static bool inited = false;

  if (!inited)
  {
    for (int i = 0; i < PLOT_W; ++i)
    {
      lastY[i] = -1;
      lastThk[i] = traceThick;
    }
    inited = true;
  }

  handleSerial();

  if (micTickFlag)
  {
    micTickFlag = false;

    int plotX = PLOT_X0 + x;

    // erase previous vertical stroke (restore background per pixel)
    if (lastY[x] >= PLOT_Y0 && lastY[x] < PLOT_Y0 + PLOT_H)
    {
      int halfPrev = (int)lastThk[x] / 2;
      int y0 = lastY[x] - halfPrev;
      int y1 = y0 + lastThk[x] - 1;
      if (y0 < PLOT_Y0)
        y0 = PLOT_Y0;
      if (y1 >= PLOT_Y0 + PLOT_H)
        y1 = PLOT_Y0 + PLOT_H - 1;
      for (int yy = y0; yy <= y1; ++yy)
      {
        tft.drawPixel(plotX, yy, backgroundAt(plotX, yy));
      }
    }

    // sample mic (12-bit), store
    readings[x] = analogRead(MIC_PIN);

    // draw new vertical stroke with current thickness
    int y = adcToY_raw(readings[x]);
    if (y >= PLOT_Y0 && y < PLOT_Y0 + PLOT_H)
    {
      int half = (int)traceThick / 2;
      int y0 = y - half;
      int y1 = y0 + traceThick - 1;
      if (y0 < PLOT_Y0)
        y0 = PLOT_Y0;
      if (y1 >= PLOT_Y0 + PLOT_H)
        y1 = PLOT_Y0 + PLOT_H - 1;
      for (int yy = y0; yy <= y1; ++yy)
      {
        tft.drawPixel(plotX, yy, COL_TRACE);
      }
      // re-draw dashed midline segment if shown
      if (gShowMidline)
      {
        int phase = (plotX - PLOT_X0) % (DASH_ON + DASH_OFF);
        if (phase < DASH_ON)
          tft.drawPixel(plotX, midY, COL_GRID);
      }
      lastY[x] = y;
      lastThk[x] = traceThick;
    }
    else
    {
      lastY[x] = -1;
      lastThk[x] = traceThick;
    }

    // advance column with adjustable stride
    x += xStep;
    if (x >= PLOT_W)
      x -= PLOT_W;
  }

  // Update VU at ~100 Hz
  static uint32_t lastVU = 0;
  uint32_t now = millis();
  if (now - lastVU >= 10)
  {
    lastVU = now;
    updateVU();
  }
}
// ==================== end main.cpp ====================
