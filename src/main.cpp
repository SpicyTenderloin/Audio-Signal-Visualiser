// ==================== main.cpp (ESP32 + ILI9341, banners + axis + adjustable thickness) ====================
#include <Arduino.h>
#include <SPI.h>
#include <Ticker.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// -------------------- USER SETTINGS --------------------
// TFT pins (change if your wiring differs)
#define TFT_CS    5
#define TFT_DC    21
#define TFT_RST   22
#define TFT_MOSI  23
#define TFT_SCLK  18
#define TFT_MISO  19   // if your TFT has SDO; else use 5-arg ctor

// Mic input pin (ADC1 recommended: 32,33,34,35,36,39). 36 = ADC1_CH0
#define MIC_PIN   34

// Sample frequency (Hz)
constexpr uint32_t SAMPLE_FREQ_HZ = 4400;

// Screen geometry (rotation set to 1 => 320x240)
constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 240;

// --- Plot layout with top/bottom banners ---
constexpr int PLOT_TOPBANNER    = 10;                 // reserved space at top
constexpr int PLOT_BOTTOMBANNER = 10;                 // reserved space at bottom
constexpr int PLOT_LMARGIN      = 38;                 // left margin reserved for Y axis

constexpr int PLOT_X0 = PLOT_LMARGIN;                 // plot area left
constexpr int PLOT_Y0 = PLOT_TOPBANNER;               // plot area top
constexpr int PLOT_W  = SCREEN_W - PLOT_X0;           // plot width
constexpr int PLOT_H  = SCREEN_H - (PLOT_TOPBANNER + PLOT_BOTTOMBANNER);

// VU LED pins (twelve digital outputs). Pick safe ESP32 GPIOs (not used by TFT).
constexpr uint8_t Level1  = 2;
constexpr uint8_t Level2  = 4;
constexpr uint8_t Level3  = 12;
constexpr uint8_t Level4  = 13;
constexpr uint8_t Level5  = 14;
constexpr uint8_t Level6  = 15;
constexpr uint8_t Level7  = 25;
constexpr uint8_t Level8  = 26;
constexpr uint8_t Level9  = 27;
constexpr uint8_t Level10 = 32;
constexpr uint8_t Level11 = 33;
constexpr uint8_t Level12 = 16;

// Colors (RGB565)
constexpr uint16_t Black   = 0x0000;
constexpr uint16_t Dark    = 0x4208; // dark grey for grid/dashes
constexpr uint16_t White   = 0xFFFF;

// -------------------- GLOBALS --------------------
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);
// If your display has no MISO/SDO pin, use this instead:
// Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

Ticker micTicker;
volatile bool micTickFlag = false;

// Horizontal stride (how many pixels we advance per plotted sample)
volatile uint8_t xStep = 2;         // 1 = every pixel, 2 = every 2 pixels, etc.
constexpr uint8_t XSTEP_MIN = 1;
constexpr uint8_t XSTEP_MAX = 8;

// Trace thickness (vertical pixels). Adjustable.
volatile uint8_t traceThick = 3;    // initial thickness = 3
constexpr uint8_t THICK_MIN = 1;
constexpr uint8_t THICK_MAX = 8;

volatile int readings[PLOT_W];       // circular buffer in plot width
uint16_t DCOffset = 0;               // average mic DC in raw ADC units (0..4095)
int midY = 0;                        // y of 1.65 V midline

// Dashed midline pattern (pixels)
constexpr int DASH_ON  = 6;
constexpr int DASH_OFF = 6;

// -------------------- HELPERS --------------------
static inline int adcToY_raw(int raw)
{
  if (raw < 0) raw = 0;
  if (raw > 4095) raw = 4095;
  // scale into plot height, then offset by top banner
  int y = PLOT_Y0 + PLOT_H - 1 - ((uint32_t)raw * PLOT_H >> 12);
  return y;
}

// Background “color” at a plot pixel (restores dashed midline instead of erasing it)
uint16_t backgroundAt(int plotX, int y)
{
  // only plot band
  if (y < PLOT_Y0 || y >= PLOT_Y0 + PLOT_H) return Black;

  // dashed midline preservation
  if (y == midY) {
    int xInPlot = plotX - PLOT_X0;
    int phase = xInPlot % (DASH_ON + DASH_OFF);
    if (phase < DASH_ON) return Dark;  // draw dash
  }
  return Black; // otherwise plain background
}

void IRAM_ATTR onMicTick() { micTickFlag = true; }

void startSampling(uint32_t freqHz)
{
  if (freqHz == 0) freqHz = 4400;
  micTicker.detach();
  micTicker.attach(1.0f / (float)freqHz, onMicTick);
}

uint16_t estimateDCoffset(int numSamples)
{
  uint32_t sum = 0;
  for (int i = 0; i < numSamples; ++i) {
    sum += analogRead(MIC_PIN);
    delay(2);
  }
  return (uint16_t)(sum / (uint32_t)numSamples);
}

void drawTitleBanner()
{
  // Top banner: keep it black; center the title text
  tft.fillRect(0, 0, SCREEN_W, PLOT_TOPBANNER, Black);

  const char* title = "Audio Signal Visualiser";
  int len = strlen(title);
  int textW = len * 6;        // GFX 5x7 font ~6 px/char at size 1
  int x = (SCREEN_W - textW) / 2;
  if (x < 0) x = 0;

  tft.setTextColor(White, Black);
  tft.setTextSize(1);
  tft.setCursor(x, 1);
  tft.print(title);
}

void drawBottomBannerHUD()
{
  // Bottom banner (keep black) + HUD text within it
  tft.fillRect(0, SCREEN_H - PLOT_BOTTOMBANNER, SCREEN_W, PLOT_BOTTOMBANNER, Black);
  tft.setTextColor(White, Black);
  tft.setTextSize(1);
  tft.setCursor(PLOT_X0 + 2, SCREEN_H - PLOT_BOTTOMBANNER + 1);
  tft.print("Fs:");
  tft.print(SAMPLE_FREQ_HZ / 1000.0f, 1);
  tft.print("kHz  Xstep:");
  tft.print(xStep);
  tft.print("  Thk:");
  tft.print(traceThick);
}

void drawYAxisScale()
{
  // Axis background strip (left margin within plot band)
  tft.fillRect(0, PLOT_Y0, PLOT_LMARGIN, PLOT_H, Black);

  // Axis line (inside plot band only)
  tft.drawFastVLine(PLOT_LMARGIN - 1, PLOT_Y0, PLOT_H, White);

  // Helpers to place ticks within plot band
  auto yForVolt = [](float v)->int {
    int raw = (int)roundf((v / 3.3f) * 4095.0f);
    return adcToY_raw(raw);
  };

  tft.setTextColor(White, Black);
  tft.setTextSize(1);

  // Small ticks every 0.1V; major every 0.5V. All constrained to plot band.
  for (int i = 0; i <= 33; ++i) {
    float v = i * 0.1f;                 // 0.0 .. 3.3
    int y = yForVolt(v);
    if (y < PLOT_Y0 || y >= PLOT_Y0 + PLOT_H) continue;
    int tickLen = (i % 5 == 0) ? 6 : 3; // major every 0.5V
    // draw ticks entirely to the left of the axis (won't invade plot area)
    int xStart = PLOT_LMARGIN - 1 - tickLen;
    tft.drawFastHLine(xStart, y, tickLen, (i % 5 == 0) ? White : Dark);
  }

  // Labels every 0.5 V + 3.3 V, within plot band
  for (float v = 0.0f; v <= 3.31f; v += 0.5f) {
    int y = yForVolt(v);
    if (y < PLOT_Y0 || y >= PLOT_Y0 + PLOT_H) continue;

    char buf[8];
    dtostrf(v, 3, 1, buf); // "0.0" .. "3.0"
    tft.setCursor(2, y - 4);
    tft.print(buf);
    tft.print("V");
  }
  // 3.3 V specifically
  {
    int y = yForVolt(3.3f);
    if (y >= PLOT_Y0 && y < PLOT_Y0 + PLOT_H) {
      tft.setCursor(2, y - 4);
      tft.print("3.3V");
    }
  }
}

void drawPlotBackground()
{
  // Clear plot area only; leave banners & left scale untouched
  tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, PLOT_H, Black);

  // Dashed midline (~1.65V)
  midY = adcToY_raw((int)roundf((1.65f / 3.3f) * 4095.0f));
  int x = PLOT_X0;
  while (x < PLOT_X0 + PLOT_W) {
    int run = min(DASH_ON, PLOT_X0 + PLOT_W - x);
    tft.drawFastHLine(x, midY, run, Dark);
    x += DASH_ON + DASH_OFF;
  }
}

void setXStep(uint8_t newStep)
{
  if (newStep < XSTEP_MIN) newStep = XSTEP_MIN;
  if (newStep > XSTEP_MAX) newStep = XSTEP_MAX;
  if (newStep == xStep) return;
  xStep = newStep;
  drawBottomBannerHUD();
  drawPlotBackground();
}

void setTraceThick(uint8_t newThick)
{
  if (newThick < THICK_MIN) newThick = THICK_MIN;
  if (newThick > THICK_MAX) newThick = THICK_MAX;
  if (newThick == traceThick) return;
  traceThick = newThick;
  drawBottomBannerHUD();
  drawPlotBackground();
}

void initVU()
{
  pinMode(Level1,  OUTPUT); pinMode(Level2,  OUTPUT); pinMode(Level3,  OUTPUT);
  pinMode(Level4,  OUTPUT); pinMode(Level5,  OUTPUT); pinMode(Level6,  OUTPUT);
  pinMode(Level7,  OUTPUT); pinMode(Level8,  OUTPUT); pinMode(Level9,  OUTPUT);
  pinMode(Level10, OUTPUT); pinMode(Level11, OUTPUT); pinMode(Level12, OUTPUT);
}

void setVU(uint8_t level) // 0..12
{
  const uint8_t pins[12] = {
    Level1,Level2,Level3,Level4,Level5,Level6,
    Level7,Level8,Level9,Level10,Level11,Level12
  };
  for (uint8_t i = 0; i < 12; ++i) digitalWrite(pins[i], (i < level) ? HIGH : LOW);
}

void updateVU()
{
  int maxRead = 0;
  for (int i = 0; i < PLOT_W; ++i)
    if (readings[i] > maxRead) maxRead = readings[i];

  // amplitude (raw 0..4095) -> scale to 10-bit thresholds by >>2
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
  else if (amp10 > 82 ) VUlevel = 4;
  else if (amp10 > 57 ) VUlevel = 3;
  else if (amp10 > 36 ) VUlevel = 2;
  else if (amp10 > 20 ) VUlevel = 1;
  else                   VUlevel = 0;

  setVU(VUlevel);
}

// -------------------- SETUP / LOOP --------------------
void setup()
{
  Serial.begin(115200);
  Serial.println(F("ESP32 + ILI9341 scope (banners + axis + adjustable thickness)"));

  // ADC: 12-bit and ~0..3.3V range
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);
  pinMode(MIC_PIN, INPUT);

  // TFT init
  tft.begin();
  tft.setRotation(1);  // 320x240
  tft.setFont();

  drawTitleBanner();
  drawYAxisScale();
  drawPlotBackground();
  drawBottomBannerHUD();

  // VU LEDs
  initVU();

  // Clear buffers
  for (int i = 0; i < PLOT_W; ++i) readings[i] = 0;

  // DC offset
  tft.setCursor(PLOT_X0 + 2, PLOT_Y0 + 4);
  tft.setTextColor(White, Black);
  tft.print("Measuring DC Offset...");
  DCOffset = estimateDCoffset(256);

  // Show measured offset (volts) in top of plot band briefly
  tft.fillRect(PLOT_X0, PLOT_Y0, PLOT_W, 12, Black);
  tft.setCursor(PLOT_X0 + 2, PLOT_Y0 + 2);
  tft.print("DC: ");
  tft.print((DCOffset / 4095.0f) * 3.3f, 2);
  tft.print(" V");
  delay(600);

  drawPlotBackground();
  drawBottomBannerHUD();

  // Start sampling
  startSampling(SAMPLE_FREQ_HZ);
}

void loop()
{
  static int x = 0;                 // column within plot area [0..PLOT_W-1]
  static int lastY[PLOT_W];         // last drawn center y for erasing
  static uint8_t lastThk[PLOT_W];   // last thickness used at that column
  static bool inited = false;

  if (!inited) {
    for (int i = 0; i < PLOT_W; ++i) { lastY[i] = -1; lastThk[i] = traceThick; }
    inited = true;
  }

  // Serial controls:
  // '[' dec Xstep, ']' inc Xstep, digits '1'..'8' set Xstep
  // '{' dec thickness, '}' inc thickness, digits '!'..'(' (shifted 1..8) optional set thickness
  if (Serial.available()) {
    int c = Serial.read();
    if (c == '[') setXStep(xStep > XSTEP_MIN ? xStep - 1 : XSTEP_MIN);
    if (c == ']') setXStep(xStep < XSTEP_MAX ? xStep + 1 : XSTEP_MAX);
    if (c >= '1' && c <= '8') setXStep(uint8_t(c - '0'));
    if (c == '{') setTraceThick(traceThick > THICK_MIN ? traceThick - 1 : THICK_MIN);
    if (c == '}') setTraceThick(traceThick < THICK_MAX ? traceThick + 1 : THICK_MAX);
  }

  // When the ticker fires, do one plotted sample
  if (micTickFlag) {
    micTickFlag = false;

    int plotX = PLOT_X0 + x;

    // erase previous vertical stroke (restore background per pixel)
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

    // sample mic (12-bit), store
    readings[x] = analogRead(MIC_PIN);

    // draw new vertical stroke with current thickness, restoring dashed midline on erase next time
    int y = adcToY_raw(readings[x]);
    if (y >= PLOT_Y0 && y < PLOT_Y0 + PLOT_H) {
      int half = (int)traceThick / 2;
      int y0 = y - half;
      int y1 = y0 + traceThick - 1;
      if (y0 < PLOT_Y0) y0 = PLOT_Y0;
      if (y1 >= PLOT_Y0 + PLOT_H) y1 = PLOT_Y0 + PLOT_H - 1;
      for (int yy = y0; yy <= y1; ++yy) {
        tft.drawPixel(plotX, yy, White);
      }
      lastY[x] = y;
      lastThk[x] = traceThick;
    } else {
      lastY[x] = -1;
      lastThk[x] = traceThick;
    }

    // advance column with adjustable stride
    x += xStep;
    if (x >= PLOT_W) x -= PLOT_W;
  }

  // Update VU at ~100 Hz
  static uint32_t lastVU = 0;
  uint32_t now = millis();
  if (now - lastVU >= 10) {
    lastVU = now;
    updateVU();
  }
}
// ==================== end main.cpp ====================
