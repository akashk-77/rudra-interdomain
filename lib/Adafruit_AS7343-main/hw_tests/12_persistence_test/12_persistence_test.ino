/*!
 * @file 12_persistence_test.ino
 * @brief Hardware test for AS7343 persistence setting
 *
 * Tests setPersistence() and getPersistence() for all values 0-15,
 * then verifies actual interrupt behavior:
 * - Persistence determines how many consecutive out-of-threshold
 *   readings are needed before AINT fires
 * - Higher persistence = more cycles before interrupt
 *
 * Hardware: AS7343 on I2C, NeoPixel ring (16 LEDs) on pin 6
 * Board: Metro Mini (arduino:avr:uno)
 */

#include <Adafruit_AS7343.h>
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 16
#define MAX_CYCLES 50

Adafruit_AS7343 as7343;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool readbackPassed = true;
bool timingPassed = true;

// Count dataReady cycles until AINT fires (or timeout)
// Returns cycle count, or MAX_CYCLES+1 if timeout
uint8_t countCyclesToAINT() {
  uint16_t buf[12];
  uint8_t cycles = 0;

  while (cycles < MAX_CYCLES) {
    // Wait for data ready
    while (!as7343.dataReady()) {
      delay(1);
    }
    as7343.readAllChannels(buf);
    cycles++;

    // Check if AINT bit is set (bit 3 of status register)
    uint8_t status = as7343.getStatus();
    if (status & 0x08) {
      break;
    }
  }

  return cycles;
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("AS7343 Persistence Test"));
  Serial.println(F("======================="));
  Serial.println();

  // Initialize NeoPixels OFF
  pixels.begin();
  pixels.clear();
  pixels.show();

  if (!as7343.begin()) {
    Serial.println(F("ERROR: AS7343 not found!"));
    Serial.println(F("RESULT: FAIL"));
    while (1)
      delay(10);
  }

  // ==========================================
  // Part 1: Register Readback Test
  // ==========================================
  Serial.println(F("Part 1: Register Readback"));
  Serial.println(F("Value    Readback   Status"));
  Serial.println(F("-----    --------   ------"));

  for (uint8_t val = 0; val <= 15; val++) {
    as7343.setPersistence(val);
    uint8_t readback = as7343.getPersistence();

    Serial.print(val);
    if (val < 10)
      Serial.print(F(" "));
    Serial.print(F("       "));
    Serial.print(readback);
    if (readback < 10)
      Serial.print(F(" "));
    Serial.print(F("         "));

    if (readback == val) {
      Serial.println(F("PASS"));
    } else {
      Serial.println(F("FAIL"));
      readbackPassed = false;
    }
  }

  Serial.println();
  Serial.print(F("Register readback: "));
  Serial.println(readbackPassed ? F("PASS") : F("FAIL"));
  Serial.println();

  // ==========================================
  // Part 2: Hardware Timing Verification
  // ==========================================
  Serial.println(F("Part 2: Hardware Timing Verification"));
  Serial.println();

  // Configure for fast readings
  as7343.setGain(AS7343_GAIN_16X);
  as7343.setATIME(9);
  as7343.setASTEP(599);

  // --- Calibration ---
  Serial.println(F("Calibration:"));

  // Measure baseline (NeoPixels OFF)
  pixels.clear();
  pixels.show();
  delay(50);

  as7343.startMeasurement();
  while (!as7343.dataReady())
    delay(1);
  uint16_t channels[12];
  as7343.readAllChannels(channels);
  as7343.stopMeasurement();
  uint16_t baseline = channels[0]; // FZ channel

  Serial.print(F("  NeoPixels OFF (FZ baseline): "));
  Serial.println(baseline);

  // Measure peak (NeoPixels ON bright white)
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i, 255, 255, 255);
  }
  pixels.show();
  delay(50);

  as7343.startMeasurement();
  while (!as7343.dataReady())
    delay(1);
  as7343.readAllChannels(channels);
  as7343.stopMeasurement();
  uint16_t peak = channels[0];

  Serial.print(F("  NeoPixels ON  (FZ peak):     "));
  Serial.println(peak);

  // Set threshold at baseline + 20% of range
  uint16_t range = peak - baseline;
  uint16_t threshold = baseline + (range / 5);

  Serial.print(F("  HIGH threshold (base+20%):   "));
  Serial.println(threshold);
  Serial.println();

  // Check we have usable range
  if (range < 100) {
    Serial.println(F("ERROR: Insufficient light range for test"));
    Serial.println(F("RESULT: FAIL"));
    while (1)
      delay(10);
  }

  // --- Counting Test ---
  Serial.println(F("Counting Cycles to AINT:"));
  Serial.println(F("Persistence  Cycles  Status"));
  Serial.println(F("-----------  ------  ------"));

  uint8_t count1 = 0;
  uint8_t count8 = 0;

  // Test persistence = 1
  pixels.clear();
  pixels.show();
  delay(20);
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i, 255, 255, 255);
  }
  pixels.show();
  delay(20);

  as7343.setPersistence(1);
  as7343.setThresholdChannel(0); // FZ
  as7343.setHighThreshold(threshold);
  as7343.clearStatus();
  as7343.enableSpectralInterrupt(true);
  as7343.startMeasurement();

  count1 = countCyclesToAINT();

  as7343.stopMeasurement();
  as7343.enableSpectralInterrupt(false);

  bool pass1 = (count1 > 0 && count1 < MAX_CYCLES);
  Serial.print(F("     1           "));
  if (count1 < 10)
    Serial.print(F(" "));
  Serial.print(count1);
  Serial.print(F("      "));
  Serial.println(pass1 ? F("PASS") : F("FAIL"));

  // Test persistence = 8
  pixels.clear();
  pixels.show();
  delay(20);
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i, 255, 255, 255);
  }
  pixels.show();
  delay(20);

  as7343.setPersistence(8);
  as7343.setThresholdChannel(0);
  as7343.setHighThreshold(threshold);
  as7343.clearStatus();
  as7343.enableSpectralInterrupt(true);
  as7343.startMeasurement();

  count8 = countCyclesToAINT();

  as7343.stopMeasurement();
  as7343.enableSpectralInterrupt(false);

  bool pass8 = (count8 > 0 && count8 < MAX_CYCLES);
  Serial.print(F("     8           "));
  if (count8 < 10)
    Serial.print(F(" "));
  Serial.print(count8);
  Serial.print(F("      "));
  Serial.println(pass8 ? F("PASS") : F("FAIL"));

  pixels.clear();
  pixels.show();

  Serial.println();

  // --- Verification ---
  Serial.println(F("Verification:"));

  Serial.print(F("  a) Persistence=1 triggers AINT:    "));
  Serial.println(pass1 ? F("PASS") : F("FAIL"));
  if (!pass1)
    timingPassed = false;

  Serial.print(F("  b) Persistence=8 triggers AINT:    "));
  Serial.println(pass8 ? F("PASS") : F("FAIL"));
  if (!pass8)
    timingPassed = false;

  bool orderPass = (count8 > count1);
  Serial.print(F("  c) Higher persistence = more cycles ("));
  Serial.print(count8);
  Serial.print(F(" > "));
  Serial.print(count1);
  Serial.print(F("): "));
  Serial.println(orderPass ? F("PASS") : F("FAIL"));
  if (!orderPass)
    timingPassed = false;

  // ==========================================
  // Summary
  // ==========================================
  Serial.println();
  Serial.print(F("Register readback: "));
  Serial.println(readbackPassed ? F("PASS") : F("FAIL"));
  Serial.print(F("Timing test:       "));
  Serial.println(timingPassed ? F("PASS") : F("FAIL"));

  bool allPass = readbackPassed && timingPassed;
  Serial.print(F("RESULT: "));
  Serial.println(allPass ? F("PASS") : F("FAIL"));
}

void loop() { delay(1000); }
