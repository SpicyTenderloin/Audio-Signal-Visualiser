// ==================== main.cpp (ESP32 + ILI9341, buttons + 6-LED VU + paused grid) ====================
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// --- custom fonts ---
#include "Aurora4pt7b.h" // small font  (aurora_244pt7b)
#include "Aurora7pt7b.h" // title font  (aurora_247pt7b)

// -------------------- USER SETTINGS --------------------
#define TFT_CS 5
#define TFT_DC 21
#define TFT_RST 22
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_MISO 19

#define MIC_PIN 36 // ADC1_CH0

// Buttons (active-low with internal pull-ups)
#define BTN_FS_DOWN 12
#define BTN_FS_UP 13
#define BTN_PX_DOWN 15
#define BTN_PX_UP 2
#define BTN_PAUSE 0  

// VU LEDs (6 levels) — outputs ONLY (34/35 are input-only on ESP32, so don't use them)
#define VU1 14
#define VU2 27
#define VU3 26
#define VU4 25
#define VU5 33
#define VU6 32

// Screen geometry
constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 240;

// --- Layout ---
constexpr int PLOT_TOPBANNER = 24;    // title region (text only)
constexpr int XAXIS_HEIGHT = 12;      // reserved strip under plot for x-axis ticks/labels
constexpr int PLOT_BOTTOMBANNER = 20; // bottom HUD
constexpr int PLOT_LMARGIN = 38;      // left margin for Y axis & labels

constexpr int PLOT_X0 = PLOT_LMARGIN;
constexpr int PLOT_Y0 = PLOT_TOPBANNER;
constexpr int PLOT_W = SCREEN_W - PLOT_X0;
constexpr int PLOT_H = SCREEN_H - (PLOT_TOPBANNER + PLOT_BOTTOMBANNER + XAXIS_HEIGHT);

// ----------- Color palette -----------
static inline uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
uint16_t COL_BG = ILI9341_BLACK;
uint16_t COL_TEXT = ILI9341_WHITE;
uint16_t COL_TITLE = RGB565(244, 206, 39); // yellow title
uint16_t COL_AXIS = ILI9341_WHITE;
uint16_t COL_TICKS = ILI9341_WHITE;
uint16_t COL_GRID = RGB565(30, 30, 30);
uint16_t COL_TRACE = ILI9341_WHITE;

// -------------------- GLOBALS --------------------
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);

// For 4" display
// Adafruit_ILI9488 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);

// Sampling & visualization
volatile uint32_t gSampleFreqHz = 5000; // default Fs (Hz)
constexpr uint32_t FS_MIN = 1000;
constexpr uint32_t FS_MAX = 20000;

volatile uint8_t pxPerSample = 2; // “Px/Sample” (1..10)
constexpr uint8_t PXS_MIN = 1;
constexpr uint8_t PXS_MAX = 10;

// Plot state
int16_t gLastY[PLOT_W];    // last drawn y per column, -1 means “none”
uint16_t gDCOffsetRaw = 0; // measured raw offset (0..4095)

// Pause + paused overlay grid
volatile bool gPaused = false;
bool gShowPausedGrid = true;

// Button debounce
struct Btn
{
  uint8_t pin;
  bool lastStable;     // last debounced state
  bool lastRaw;        // last read raw
  uint32_t lastChange; // ms
};
Btn btnFsDown{BTN_FS_DOWN, true, true, 0};
Btn btnFsUp{BTN_FS_UP, true, true, 0};
Btn btnPxDown{BTN_PX_DOWN, true, true, 0};
Btn btnPxUp{BTN_PX_UP, true, true, 0};
Btn btnPause{BTN_PAUSE, true, true, 0};
constexpr uint16_t DEBOUNCE_MS = 20;

// -------------------- HELPERS --------------------
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
  int y = PLOT_Y0 + PLOT_H - 1 - ((uint32_t)raw * PLOT_H >> 12);
  return y;
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

// -------------------- VU --------------------
void initVU()
{
  const uint8_t pins[6] = {VU1, VU2, VU3, VU4, VU5, VU6};
  for (uint8_t i = 0; i < 6; ++i)
    pinMode(pins[i], OUTPUT);
}

void setVU(uint8_t level)
{ // 0..6
  const uint8_t pins[6] = {VU1, VU2, VU3, VU4, VU5, VU6};
  for (uint8_t i = 0; i < 6; ++i)
    digitalWrite(pins[i], (i < level) ? HIGH : LOW);
}

void VUdance()
{
  for (int i = 0; i <= 6; ++i)
  {
    setVU(i);
    delay(120);
  }
  setVU(0);
}

// -------------------- DRAWING --------------------
void drawTitle()
{
  const char *title = "Audio Signal Visualiser";
  tft.setFont(&aurora_247pt7b);
  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds(title, 0, 0, &x1, &y1, &tw, &th);

  int x = PLOT_X0 + ((int)PLOT_W - (int)tw) / 2;
  int baselineY = (PLOT_TOPBANNER - (int)th) / 2 + (int)th;

  tft.setTextColor(COL_TITLE, COL_BG);
  tft.setCursor(x, baselineY);
  tft.print(title);
}

void drawBottomBannerHUD()
{
  // Clear HUD strip
  tft.fillRect(PLOT_X0, SCREEN_H - PLOT_BOTTOMBANNER, PLOT_W, PLOT_BOTTOMBANNER, COL_BG);

  tft.setFont(&aurora_244pt7b);
  tft.setTextColor(COL_TEXT, COL_BG);

  char fsBuf[28];
  snprintf(fsBuf, sizeof(fsBuf), "Fs: %.1fkHz", gSampleFreqHz / 1000.0f);
  char pxBuf[28];
  snprintf(pxBuf, sizeof(pxBuf), "Px/Sample: %u", (unsigned)pxPerSample);

  // If paused, append [PAUSED] to the right-most section
  char statusBuf[32];
  if (gPaused)
  {
    snprintf(statusBuf, sizeof(statusBuf), "[PAUSED]");
  }
  else
  {
    snprintf(statusBuf, sizeof(statusBuf), " "); // empty filler when running
  }

  int16_t x1, y1;
  uint16_t wFs, hFs, wPx, hPx, wStatus, hStatus;
  tft.getTextBounds(fsBuf, 0, 0, &x1, &y1, &wFs, &hFs);
  tft.getTextBounds(pxBuf, 0, 0, &x1, &y1, &wPx, &hPx);
  tft.getTextBounds(statusBuf, 0, 0, &x1, &y1, &wStatus, &hStatus);

  const int gap = 16;
  int totalW = (int)wFs + gap + (int)wPx + gap + (int)wStatus;
  int startX = PLOT_X0 + (PLOT_W - totalW) / 2;
  int textH = max(max((int)hFs, (int)hPx), (int)hStatus);
  int baselineY = SCREEN_H - (PLOT_BOTTOMBANNER - textH) / 2;

  int x = startX;
  tft.setCursor(x, baselineY);
  tft.print(fsBuf);
  x += wFs + gap;
  tft.setCursor(x, baselineY);
  tft.print(pxBuf);
  x += wPx + gap;
  tft.setCursor(x, baselineY);
  tft.print(statusBuf);
}

void drawYAxisScale()
{
  tft.fillRect(0, PLOT_Y0, PLOT_LMARGIN, PLOT_H, COL_BG);
  tft.drawFastVLine(PLOT_LMARGIN - 1, PLOT_Y0, PLOT_H, COL_AXIS);

  auto yForVolt = [](float v) -> int
  {
    int raw = (int)roundf((v / 3.3f) * 4095.0f);
    return adcToY_raw(raw);
  };

  tft.setFont(&aurora_244pt7b);
  tft.setTextColor(COL_TEXT, COL_BG);

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

int computePxPerMajor()
{
  const float dt = 1.0f / (float(gSampleFreqHz) * float(pxPerSample));
  const float steps[] = {
      1e-4f, 2e-4f, 5e-4f,
      1e-3f, 2e-3f, 5e-3f,
      1e-2f, 2e-2f, 5e-2f,
      1e-1f, 2e-1f, 5e-1f, 1.0f};
  float targetPx = 40.0f, bestStep = steps[0], bestDiff = 1e9f;
  for (size_t i = 0; i < sizeof(steps) / sizeof(steps[0]); ++i)
  {
    float px = steps[i] / dt;
    float d = fabsf(px - targetPx);
    if (d < bestDiff)
    {
      bestDiff = d;
      bestStep = steps[i];
    }
  }
  return (int)roundf(bestStep / dt);
}

void drawXAxisScale()
{
  const int y0 = PLOT_Y0 + PLOT_H;
  tft.fillRect(PLOT_X0, y0, PLOT_W, XAXIS_HEIGHT, COL_BG);
  tft.drawFastHLine(PLOT_X0, y0, PLOT_W, COL_AXIS);

  const float dt = 1.0f / (float(gSampleFreqHz) * float(pxPerSample));
  const int pxPerMajor = computePxPerMajor();

  tft.setFont(&aurora_244pt7b);
  tft.setTextColor(COL_TEXT, COL_BG);

  for (int x = 0; x <= PLOT_W; x += pxPerMajor)
  {
    int xx = PLOT_X0 + x;
    tft.drawFastVLine(xx, y0, 6, COL_TICKS);

    float t = x * dt; // seconds
    char lab[16];
    if (t >= 1.0f)
      snprintf(lab, sizeof(lab), "%.2fs", t);
    else if (t >= 1e-3f)
      snprintf(lab, sizeof(lab), "%.0fms", t * 1000.0f);
    else
      snprintf(lab, sizeof(lab), "%.0fus", t * 1e6f);

    int16_t x1, y1;
    uint16_t tw, th;
    tft.getTextBounds(lab, 0, 0, &x1, &y1, &tw, &th);
    int tx = xx - (int)tw / 2;
    int ty = y0 + 10 + (int)th / 2;
    if (tx < PLOT_X0)
      tx = PLOT_X0;
    if (tx + (int)tw > PLOT_X0 + PLOT_W)
      tx = PLOT_X0 + PLOT_W - tw;
    tft.setCursor(tx, ty);
    tft.print(lab);

    int xm = xx + pxPerMajor / 2;
    if (xm < PLOT_X0 + PLOT_W)
      tft.drawFastVLine(xm, y0, 3, COL_TICKS);
  }
}

void clearPlotAndHistory()
{
  tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, PLOT_H, COL_BG);
  for (int i = 0; i < PLOT_W; ++i)
    gLastY[i] = -1;
}

void drawPausedGrid()
{
  // Horizontal gridlines at major Y ticks
  for (int i = 0; i <= 33; i++)
  {
    if (i % 5 == 0)
    { // major ticks every 0.5V
      float v = i * 0.1f;
      int raw = (int)roundf((v / 3.3f) * 4095.0f);
      int y = adcToY_raw(raw);
      if (y >= PLOT_Y0 && y < PLOT_Y0 + PLOT_H)
      {
        tft.drawFastHLine(PLOT_X0, y, PLOT_W, COL_GRID);
      }
    }
  }

  // Vertical gridlines at major X ticks
  const float dt = 1.0f / (float(gSampleFreqHz) * float(pxPerSample));
  const float steps[] = {1e-4f, 2e-4f, 5e-4f, 1e-3f, 2e-3f, 5e-3f,
                         1e-2f, 2e-2f, 5e-2f, 1e-1f, 2e-1f, 5e-1f, 1.0f};
  float targetPx = 40.0f;
  float bestStep = steps[0], bestDiff = 1e9f;
  for (size_t i = 0; i < sizeof(steps) / sizeof(steps[0]); ++i)
  {
    float px = steps[i] / dt;
    float d = fabsf(px - targetPx);
    if (d < bestDiff)
    {
      bestDiff = d;
      bestStep = steps[i];
    }
  }
  const int pxPerMajor = (int)roundf(bestStep / dt);

  for (int x = 0; x <= PLOT_W; x += pxPerMajor)
  {
    int xx = PLOT_X0 + x;
    tft.drawFastVLine(xx, PLOT_Y0, PLOT_H, COL_GRID);
  }
}

void setPaused(bool p)
{
  if (gPaused == p)
    return;
  gPaused = p;
  if (gPaused)
  {
    drawPausedGrid();
    drawBottomBannerHUD();
    drawXAxisScale();
  }
  else
  {
    clearPlotAndHistory();
    // Your draw order preference:
    drawBottomBannerHUD();
    drawXAxisScale();
  }
}

// -------------------- SETTINGS (redraw HUD first, then X axis) --------------------
void redrawHUDandXAxis()
{
  drawBottomBannerHUD(); // banner first
  drawXAxisScale();      // then axis
}

void setPxPerSample(uint8_t n)
{
  if (n < PXS_MIN)
    n = PXS_MIN;
  if (n > PXS_MAX)
    n = PXS_MAX;
  if (n == pxPerSample)
    return;
  pxPerSample = n;
  clearPlotAndHistory();
  Serial.print(F("Px/Sample set to "));
  Serial.println(pxPerSample);
  redrawHUDandXAxis();
}

void setSampleFreq(uint32_t newFs)
{
  if (newFs < FS_MIN)
    newFs = FS_MIN;
  if (newFs > FS_MAX)
    newFs = FS_MAX;
  if (newFs == gSampleFreqHz)
    return;
  gSampleFreqHz = newFs;
  Serial.print(F("Fs set to "));
  Serial.println(gSampleFreqHz);
  redrawHUDandXAxis();
}

// -------------------- BUTTONS --------------------
bool debounceEdge(Btn &b)
{
  bool raw = digitalRead(b.pin); // HIGH=not pressed (pull-up), LOW=pressed
  uint32_t now = millis();
  if (raw != b.lastRaw)
  {
    b.lastRaw = raw;
    b.lastChange = now;
  }
  if ((now - b.lastChange) >= DEBOUNCE_MS)
  {
    if (raw != b.lastStable)
    {
      b.lastStable = raw;
      // We detect a *press* edge (HIGH->LOW)
      if (raw == LOW)
        return true;
    }
  }
  return false;
}

void pollButtons()
{
  if (debounceEdge(btnFsDown))
    setSampleFreq(gSampleFreqHz >= 2000 ? gSampleFreqHz - 1000 : FS_MIN);
  if (debounceEdge(btnFsUp))
    setSampleFreq(gSampleFreqHz + 1000);
  if (debounceEdge(btnPxDown))
    setPxPerSample(pxPerSample > PXS_MIN ? pxPerSample - 1 : PXS_MIN);
  if (debounceEdge(btnPxUp))
    setPxPerSample(pxPerSample < PXS_MAX ? pxPerSample + 1 : PXS_MAX);
  if (debounceEdge(btnPause))
    setPaused(!gPaused);
}

// -------------------- SERIAL CONTROLS --------------------
void handleSerial()
{
  if (!Serial.available())
    return;

  int peekc = Serial.peek();
  if (peekc == 'f' || peekc == 'F')
  {
    String line = Serial.readStringUntil('\n');
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
      setSampleFreq((uint32_t)val);
    else
      Serial.println(F("Parse Fs failed. Use: f8000 or fs=12000"));
    return;
  }

  int c = Serial.read();
  if (c == ' ')
  {
    setPaused(!gPaused);
    Serial.print(F("Paused: "));
    Serial.println(gPaused ? F("YES") : F("NO"));
    return;
  }
  if (c == 'g' || c == 'G')
  {
    gShowPausedGrid = !gShowPausedGrid;
    Serial.print(F("Paused grid: "));
    Serial.println(gShowPausedGrid ? F("ON") : F("OFF"));
    if (gPaused)
    {
      clearPlotAndHistory();
      if (gShowPausedGrid)
        drawPausedGrid();
    }
    return;
  }
  if (c == 'p')
    setPxPerSample(pxPerSample > PXS_MIN ? (uint8_t)(pxPerSample - 1) : PXS_MIN);
  if (c == 'P')
    setPxPerSample(pxPerSample < PXS_MAX ? (uint8_t)(pxPerSample + 1) : PXS_MAX);
}

// -------------------- SETUP / LOOP --------------------
void setup()
{
  Serial.begin(115200);
  Serial.println(F("Controls: f8000 | fs=12000 | p/P Px/Sample | <space> pause | g grid toggle"));
  Serial.println(F("Buttons: Fs-:13 Fs+:12 Px-:14 Px+:27 Pause:15"));
  Serial.println(F("VU pins: 25,26,32,33,2,4 (34/35 are input-only on ESP32)"));

  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);
  pinMode(MIC_PIN, INPUT);

  // Buttons with internal pull-ups
  pinMode(BTN_FS_DOWN, INPUT_PULLUP);
  pinMode(BTN_FS_UP, INPUT_PULLUP);
  pinMode(BTN_PX_DOWN, INPUT_PULLUP);
  pinMode(BTN_PX_UP, INPUT_PULLUP);
  pinMode(BTN_PAUSE, INPUT_PULLUP);

  // VU LEDs
  initVU();
  setVU(0);

  VUdance();

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(COL_BG);

  drawTitle();
  drawYAxisScale();
  drawBottomBannerHUD(); // your order: banner first
  drawXAxisScale();

  // DC offset splash
  {
    tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, PLOT_H, COL_BG);
    tft.setFont(&aurora_247pt7b);
    tft.setTextColor(COL_TEXT, COL_BG);
    const char *msg = "Measuring DC Offset...";
    int w, h;
    measureText(&aurora_247pt7b, msg, &w, &h);
    int msgX = PLOT_X0 + (PLOT_W - w) / 2;
    int msgY = PLOT_Y0 + (PLOT_H + h) / 2;
    tft.setCursor(msgX, msgY);
    tft.print(msg);
  }

  gDCOffsetRaw = estimateDCoffset(256);
  float dcV = (gDCOffsetRaw / 4095.0f) * 3.3f;
  Serial.print(F("DC Offset (raw): "));
  Serial.println(gDCOffsetRaw);
  Serial.print(F("DC Offset (V):   "));
  Serial.println(dcV, 3);

  // Show DC value for 1s
  tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, PLOT_H, COL_BG);
  {
    char line[32];
    snprintf(line, sizeof(line), "DC: %.2f V", dcV);
    int w, h;
    measureText(&aurora_247pt7b, line, &w, &h);
    int x = PLOT_X0 + (PLOT_W - w) / 2;
    int y = PLOT_Y0 + (PLOT_H + h) / 2;
    tft.setCursor(x, y);
    tft.print(line);
  }
  delay(1000);

  clearPlotAndHistory();
}

void loop()
{
  handleSerial();
  pollButtons();

  if (gPaused)
  {
    delay(5);
    return;
  }

  // ---- sample capture timed by micros() ----
  int Nsamples = (PLOT_W + pxPerSample - 1) / pxPerSample + 1; // +1 for segment end
  static int16_t buffer[SCREEN_W + 4];                         // generous

  uint32_t period_us = (uint32_t)(1000000UL / gSampleFreqHz);
  uint32_t t = micros();

  int16_t peak = 0; // for VU
  for (int i = 0; i < Nsamples; ++i)
  {
    t += period_us;
    while ((int32_t)(micros() - t) < 0)
    { /* spin to keep cadence */
    }
    int16_t v = analogRead(MIC_PIN);
    buffer[i] = v;
    int16_t centered = (int16_t)v - (int16_t)gDCOffsetRaw;
    if (abs(centered) > peak)
      peak = abs(centered);
  }

  // ---- render (erase-then-draw per column, 1px stroke) ----
  int xcol = 0; // 0..PLOT_W-1
  for (int i = 1; i < Nsamples && xcol < PLOT_W; ++i)
  {
    int y0 = adcToY_raw(buffer[i - 1]);
    int y1 = adcToY_raw(buffer[i]);

    for (int k = 0; k < pxPerSample && xcol < PLOT_W; ++k, ++xcol)
    {
      int num = k;
      int den = pxPerSample;
      int y = y0 + (((y1 - y0) * num + den / 2) / den);

      int lastY = gLastY[xcol];
      if (lastY >= PLOT_Y0 && lastY < (PLOT_Y0 + PLOT_H))
      {
        tft.drawPixel(PLOT_X0 + xcol, lastY, COL_BG);
      }

      if (y >= PLOT_Y0 && y < PLOT_Y0 + PLOT_H)
      {
        tft.drawPixel(PLOT_X0 + xcol, y, COL_TRACE);
        gLastY[xcol] = y;
      }
      else
      {
        gLastY[xcol] = -1;
      }
    }
  }

  // ---- 6-level VU from peak amplitude ----
  // Simple thresholds tuned for 12-bit ADC, scale down to 6 steps
  // (feel free to tweak empirically)
  uint8_t level = 0;
  if (peak > 768)
    level = 6;
  else if (peak > 640)
    level = 5;
  else if (peak > 512)
    level = 4;
  else if (peak > 384)
    level = 3;
  else if (peak > 256)
    level = 2;
  else if (peak > 128)
    level = 1;
  else
    level = 0;
  setVU(level);
}
// ==================== end main.cpp ====================
