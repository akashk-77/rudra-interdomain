/*
 * AS7343 Spectral Display Demo for ESP32-S2 TFT Feather
 *
 * Displays a real-time spectral bar chart on the built-in 1.14" TFT.
 * Each bar is colored to match the channel's wavelength.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code for
 * Adafruit Industries. MIT license, check license.txt for more information
 */

#include <Adafruit_AS7343.h>
#include <Adafruit_ST7789.h>
#include <Fonts/FreeSans9pt7b.h>

Adafruit_AS7343 as7343;
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);

// 12 spectral channels in wavelength order
const uint8_t NUM_BARS = 12;
const as7343_channel_t channels[NUM_BARS] = {
    AS7343_CHANNEL_F1,  AS7343_CHANNEL_F2,  AS7343_CHANNEL_FZ,
    AS7343_CHANNEL_F3,  AS7343_CHANNEL_F4,  AS7343_CHANNEL_F5,
    AS7343_CHANNEL_FY,  AS7343_CHANNEL_FXL, AS7343_CHANNEL_F6,
    AS7343_CHANNEL_F7,  AS7343_CHANNEL_F8,  AS7343_CHANNEL_NIR};

const char *labels[NUM_BARS] = {"F1",  "F2",  "FZ",  "F3",  "F4",  "F5",
                                "FY",  "FXL", "F6",  "F7",  "F8",  "NIR"};

// RGB565 colors approximating each channel's wavelength
const uint16_t barColors[NUM_BARS] = {
    0x780F, // F1  405nm violet
    0x401F, // F2  425nm blue-violet
    0x001F, // FZ  450nm blue
    0x02FF, // F3  475nm cyan-blue
    0x07E0, // F4  515nm green
    0xAFE0, // F5  550nm yellow-green
    0xFFE0, // FY  555nm yellow
    0xFC00, // FXL 600nm orange
    0xF800, // F6  640nm red
    0xC000, // F7  690nm deep red
    0x8000, // F8  745nm dark red
    0x4000, // NIR 855nm maroon
};

// Bar chart geometry
const uint16_t BAR_TOP = 18;    // top of chart area
const uint16_t BAR_BOTTOM = 118; // bottom of bars (leave room for labels)
const uint16_t BAR_LEFT = 4;
const uint16_t BAR_WIDTH = 17;
const uint16_t BAR_GAP = 3;

void setup() {
  Serial.begin(115200);
  delay(100);

  // Turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(100);

  display.init(135, 240);
  display.setRotation(3);

  canvas.setFont(&FreeSans9pt7b);
  canvas.setTextColor(ST77XX_WHITE);

  if (!as7343.begin()) {
    canvas.fillScreen(ST77XX_BLACK);
    canvas.setCursor(0, 40);
    canvas.setTextColor(ST77XX_RED);
    canvas.println(" AS7343 not found!");
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
    while (1)
      delay(10);
  }

  // Configure sensor
  as7343.setGain(AS7343_GAIN_64X);
  as7343.setATIME(29);
  as7343.setASTEP(599);

  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
}

void loop() {
  uint16_t readings[18];

  if (!as7343.readAllChannels(readings)) {
    Serial.println("Read failed!");
    delay(500);
    return;
  }

  // Find max value for auto-scaling
  uint16_t maxVal = 1;
  for (uint8_t i = 0; i < NUM_BARS; i++) {
    uint16_t val = readings[channels[i]];
    if (val > maxVal)
      maxVal = val;
  }

  // Draw frame
  canvas.fillScreen(ST77XX_BLACK);

  // Title
  canvas.setFont(&FreeSans9pt7b);
  canvas.setTextColor(ST77XX_WHITE);
  canvas.setCursor(40, 14);
  canvas.print("AS7343 Spectrum");

  // Draw bars
  uint16_t barHeight = BAR_BOTTOM - BAR_TOP;
  for (uint8_t i = 0; i < NUM_BARS; i++) {
    uint16_t val = readings[channels[i]];
    uint16_t h = (uint32_t)val * barHeight / maxVal;
    if (h < 1 && val > 0)
      h = 1;

    uint16_t x = BAR_LEFT + i * (BAR_WIDTH + BAR_GAP);
    uint16_t y = BAR_BOTTOM - h;

    // Filled bar
    canvas.fillRect(x, y, BAR_WIDTH, h, barColors[i]);

    // Channel label below bar
    canvas.setFont(NULL); // built-in 6x8 font for tiny labels
    canvas.setTextColor(barColors[i]);
    // Center the label under the bar
    uint8_t labelLen = strlen(labels[i]);
    uint16_t lx = x + (BAR_WIDTH - labelLen * 6) / 2;
    canvas.setCursor(lx, BAR_BOTTOM + 4);
    canvas.print(labels[i]);

    // Value on top of bar in tiny font
    canvas.setTextColor(ST77XX_WHITE);
    char vbuf[6];
    itoa(val, vbuf, 10);
    labelLen = strlen(vbuf);
    lx = x + (BAR_WIDTH - labelLen * 6) / 2;
    uint16_t vy = y - 2;
    if (vy < BAR_TOP)
      vy = BAR_TOP;
    canvas.setCursor(lx, vy);
    canvas.print(vbuf);
  }

  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  delay(200);
}
