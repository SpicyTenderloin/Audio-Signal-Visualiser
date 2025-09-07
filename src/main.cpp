// ==================== main.cpp (ESP32 + ILI9341, single file) ====================
#include <Arduino.h>
#include <SPI.h>
#include <Ticker.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// -------------------- USER SETTINGS --------------------
// TFT pins (change if your wiring differs)
#define TFT_CS 5
#define TFT_DC 21
#define TFT_RST 22
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_MISO 19 // if your TFT has SDO pin; otherwise use 5-arg constructor below

// Mic input pin (ADC1 recommended: 32,33,34,35,36,39). 36 = ADC1_CH0
#define MIC_PIN 34

// Sample frequency (Hz). Ticker ok up to audio-ish rates; for >20kHz consider hw timers
constexpr uint32_t SAMPLE_FREQ_HZ = 4400;

// Screen geometry (rotation set to 1 => 320x240)
constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 240;

// VU LED pins (twelve digital outputs). Pick safe ESP32 GPIOs (not used by TFT).
constexpr uint8_t Level1 = 2;
constexpr uint8_t Level2 = 4;
constexpr uint8_t Level3 = 12;
constexpr uint8_t Level4 = 13;
constexpr uint8_t Level5 = 14;
constexpr uint8_t Level6 = 15;
constexpr uint8_t Level7 = 25;
constexpr uint8_t Level8 = 26;
constexpr uint8_t Level9 = 27;
constexpr uint8_t Level10 = 32;
constexpr uint8_t Level11 = 33;
constexpr uint8_t Level12 = 16;

// Colors (RGB565)
constexpr uint16_t Black = 0x0000;
constexpr uint16_t Blue = 0x001F;
constexpr uint16_t Red = 0xF800;
constexpr uint16_t Green = 0x07E0;
constexpr uint16_t Cyan = 0x07FF;
constexpr uint16_t Magenta = 0xF81F;
constexpr uint16_t Yellow = 0xFFE0;
constexpr uint16_t White = 0xFFFF;

// -------------------- GLOBALS --------------------
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);
// If your display has no MISO/SDO pin, use this instead:
// Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

Ticker micTicker;
volatile bool micTickFlag = false;

volatile int readings[SCREEN_W]; // circular buffer, one sample per column
uint16_t DCOffset = 0;           // average mic DC in raw ADC units (0..4095)

// simple bounds (like your old 7..114 window, adapted to 240px tall)
// constexpr int Y_MIN = 7;
// constexpr int Y_MAX = SCREEN_H - 14;

constexpr int Y_MIN = 0;
constexpr int Y_MAX = SCREEN_H - 1;

// Horizontal stride: how many pixels we advance per plotted sample
volatile uint8_t xStep = 2;         // 1 = every pixel, 2 = every 2 pixels, etc.
constexpr uint8_t XSTEP_MIN = 1;
constexpr uint8_t XSTEP_MAX = 8;

// -------------------- HELPERS --------------------
// Map raw 12-bit ADC (0..4095) to screen Y (0..H-1), 0 = top
static inline int adcToY_raw(int raw)
{
  if (raw < 0)
    raw = 0;
  if (raw > 4095)
    raw = 4095;
  // y = H-1 - floor(raw * H / 4096)
  return (int)(SCREEN_H - 1 - ((uint32_t)raw * SCREEN_H >> 12));
}

void IRAM_ATTR onMicTick() { micTickFlag = true; }

void startSampling(uint32_t freqHz)
{
  if (freqHz == 0)
    freqHz = 4400;
  micTicker.detach();
  micTicker.attach(1.0f / (float)freqHz, onMicTick);
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

void drawBackground()
{
  tft.fillScreen(Black);

  // Top labels
  tft.setTextColor(White, Black);
  tft.setTextSize(1);
  tft.setCursor(1, 1);
  tft.print("5V     Transient");

  // Bottom labels
  tft.setCursor(1, SCREEN_H - 12);
  tft.print("0V");

  tft.setCursor(60, SCREEN_H - 12);
  tft.print("Fs: ");
  tft.print(SAMPLE_FREQ_HZ / 1000.0f, 1);
  tft.print(" kHz");

  // Show current stride
  tft.setCursor(160, SCREEN_H - 12);
  tft.print("Xstep: ");
  tft.print(xStep);
}

void setXStep(uint8_t newStep)
{
  if (newStep < XSTEP_MIN) newStep = XSTEP_MIN;
  if (newStep > XSTEP_MAX) newStep = XSTEP_MAX;

  if (newStep == xStep) return;
  xStep = newStep;

  // Clear prior trace (so gaps don’t leave ghosts)
  tft.fillRect(0, 12, SCREEN_W, SCREEN_H - 24, Black);
  drawBackground();
}

void initVU()
{
  pinMode(Level1, OUTPUT);
  pinMode(Level2, OUTPUT);
  pinMode(Level3, OUTPUT);
  pinMode(Level4, OUTPUT);
  pinMode(Level5, OUTPUT);
  pinMode(Level6, OUTPUT);
  pinMode(Level7, OUTPUT);
  pinMode(Level8, OUTPUT);
  pinMode(Level9, OUTPUT);
  pinMode(Level10, OUTPUT);
  pinMode(Level11, OUTPUT);
  pinMode(Level12, OUTPUT);
}

void setVU(uint8_t level) // 0..12
{
  // Turn on first N, off the rest
  const uint8_t pins[12] = {
      Level1, Level2, Level3, Level4, Level5, Level6,
      Level7, Level8, Level9, Level10, Level11, Level12};
  for (uint8_t i = 0; i < 12; ++i)
    digitalWrite(pins[i], (i < level) ? HIGH : LOW);
}

void updateVU()
{
  // find peak of current buffer
  int maxRead = 0;
  for (int i = 0; i < SCREEN_W; ++i)
    if (readings[i] > maxRead)
      maxRead = readings[i];

  // amplitude (raw 0..4095) -> scale to 10-bit old thresholds by >>2
  uint16_t amp10 = (uint16_t)abs(maxRead - (int)DCOffset) >> 2; // 0..1023

  // Map to 0..12 using your old thresholds
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
  Serial.println(F("ESP32 + ILI9341 scope (single file)"));

  // ADC: 12-bit and full 0..3.3V range
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db); // ~0..3.3V
  pinMode(MIC_PIN, INPUT);

  // TFT init
  tft.begin();
  tft.setRotation(1); // 320x240
  tft.setFont();
  drawBackground();

  // VU LEDs
  initVU();

  // Clear buffers
  for (int i = 0; i < SCREEN_W; ++i)
    readings[i] = 0;

  // DC offset
  tft.setCursor(15, SCREEN_H / 2 - 8);
  tft.setTextColor(White, Black);
  tft.print("Measuring DC Offset...");
  DCOffset = estimateDCoffset(256);

  // Show measured offset (volts)
  tft.fillRect(0, SCREEN_H / 2 - 12, SCREEN_W, 16, Black);
  tft.setCursor(15, SCREEN_H / 2 - 8);
  tft.print("DC Offset: ");
  tft.print((DCOffset / 4095.0f) * 3.3f, 2);
  tft.print(" V");
  delay(600);

  drawBackground();

  // Start sampling
  startSampling(SAMPLE_FREQ_HZ);
}

void loop()
{
  static int x = 0;           // current column
  static int lastY[SCREEN_W]; // last drawn y for erasing
  static bool inited = false;

  if (!inited)
  {
    for (int i = 0; i < SCREEN_W; ++i)
      lastY[i] = -1;
    inited = true;
  }

  // When the ticker fires, do one column: erase old, sample new, draw new, advance
  if (micTickFlag)
  {
    micTickFlag = false;

    // erase previous pixel if it was within the bounds
    if (lastY[x] >= Y_MIN && lastY[x] <= Y_MAX)
    {
      tft.drawPixel(x, lastY[x], Black);
    }

    // sample mic (12-bit)
    readings[x] = analogRead(MIC_PIN);

    // draw new point
    int y = adcToY_raw(readings[x]);
    if (y >= Y_MIN && y <= Y_MAX)
    {
      tft.drawPixel(x, y, White);
      lastY[x] = y;
    }
    else
    {
      lastY[x] = -1;
    }

    // next column with adjustable stride (wrap)
    x += xStep;
    if (x >= SCREEN_W) x -= SCREEN_W;
  }

  // Update VU at ~100 Hz (non-critical timing)
  static uint32_t lastVU = 0;
  uint32_t now = millis();
  if (now - lastVU >= 10)
  { // 10ms
    lastVU = now;
    updateVU();
  }
  
  // --- Serial controls for stride: '[' to decrease, ']' to increase ---
  if (Serial.available()) {
    int c = Serial.read();
    if (c == '[') setXStep(xStep > XSTEP_MIN ? xStep - 1 : XSTEP_MIN);
    if (c == ']') setXStep(xStep < XSTEP_MAX ? xStep + 1 : XSTEP_MAX);
    // Optional: digits 1..8 set exact stride
    if (c >= '1' && c <= '8') setXStep(uint8_t(c - '0'));
  }
}
// ==================== end main.cpp ====================
