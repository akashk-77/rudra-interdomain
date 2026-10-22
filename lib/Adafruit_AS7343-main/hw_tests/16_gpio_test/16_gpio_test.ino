/*!
 * @file 16_gpio_test.ino
 * @brief Hardware test for AS7343 GPIO functionality
 *
 * Tests GPIO pin control with hardware verification:
 * - Output mode: verify actual pin state via Arduino feedback pin
 * - Inversion: verify inverted output
 * - Input mode: drive pin from Arduino and read via AS7343
 *
 * Hardware: AS7343 GPIO pin connected to Metro Mini pin 3
 * Note: AS7343 GPIO is open-drain - requires pull-up for HIGH output
 * Board: Metro Mini (arduino:avr:uno)
 */

#include <Adafruit_AS7343.h>

#define FEEDBACK_PIN 3 // Arduino pin connected to AS7343 GPIO

Adafruit_AS7343 as7343;
bool allPassed = true;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("AS7343 GPIO Test"));
  Serial.println(F("================"));
  Serial.println(F("Feedback pin: D3 connected to AS7343 GPIO"));
  Serial.println();

  if (!as7343.begin()) {
    Serial.println(F("ERROR: AS7343 not found!"));
    Serial.println(F("RESULT: FAIL"));
    while (1)
      delay(10);
  }

  // ==========================================
  // Part 1: Output Mode with Hardware Feedback
  // ==========================================
  Serial.println(F("Part 1: Output Mode Test"));

  // Set feedback pin as input with pull-up (AS7343 GPIO may be open-drain)
  pinMode(FEEDBACK_PIN, INPUT_PULLUP);

  // Configure AS7343 GPIO as output, not inverted
  as7343.setGPIOOutput(true);
  as7343.setGPIOInverted(false);

  // Test HIGH output
  as7343.setGPIOValue(true);
  delay(5);
  bool pinState = digitalRead(FEEDBACK_PIN);
  Serial.print(F("  Output HIGH -> Pin reads: "));
  Serial.print(pinState ? F("HIGH") : F("LOW"));
  bool passHigh = (pinState == HIGH);
  Serial.println(passHigh ? F(" PASS") : F(" FAIL"));
  if (!passHigh)
    allPassed = false;

  // Test LOW output
  as7343.setGPIOValue(false);
  delay(5);
  pinState = digitalRead(FEEDBACK_PIN);
  Serial.print(F("  Output LOW  -> Pin reads: "));
  Serial.print(pinState ? F("HIGH") : F("LOW"));
  bool passLow = (pinState == LOW);
  Serial.println(passLow ? F(" PASS") : F(" FAIL"));
  if (!passLow)
    allPassed = false;

  Serial.println();

  // ==========================================
  // Part 2: Inversion Test
  // ==========================================
  Serial.println(F("Part 2: Inversion Test"));

  // Enable inversion
  as7343.setGPIOInverted(true);

  // With inversion, setting HIGH should output LOW
  as7343.setGPIOValue(true);
  delay(5);
  pinState = digitalRead(FEEDBACK_PIN);
  Serial.print(F("  Inverted + Value HIGH -> Pin: "));
  Serial.print(pinState ? F("HIGH") : F("LOW"));
  bool passInvHigh = (pinState == LOW); // Inverted!
  Serial.println(passInvHigh ? F(" PASS") : F(" FAIL"));
  if (!passInvHigh)
    allPassed = false;

  // With inversion, setting LOW should output HIGH
  as7343.setGPIOValue(false);
  delay(5);
  pinState = digitalRead(FEEDBACK_PIN);
  Serial.print(F("  Inverted + Value LOW  -> Pin: "));
  Serial.print(pinState ? F("HIGH") : F("LOW"));
  bool passInvLow = (pinState == HIGH); // Inverted!
  Serial.println(passInvLow ? F(" PASS") : F(" FAIL"));
  if (!passInvLow)
    allPassed = false;

  // Disable inversion for remaining tests
  as7343.setGPIOInverted(false);

  Serial.println();

  // ==========================================
  // Part 3: Input Mode Test
  // ==========================================
  Serial.println(F("Part 3: Input Mode Test"));

  // Configure AS7343 GPIO as input
  as7343.setGPIOOutput(false);
  delay(5);

  // Drive pin from Arduino and read via AS7343
  pinMode(FEEDBACK_PIN, OUTPUT);

  // Drive HIGH
  digitalWrite(FEEDBACK_PIN, HIGH);
  delay(5);
  bool readVal = as7343.getGPIOValue();
  Serial.print(F("  Arduino drives HIGH -> AS7343 reads: "));
  Serial.print(readVal ? F("HIGH") : F("LOW"));
  bool passInHigh = (readVal == true);
  Serial.println(passInHigh ? F(" PASS") : F(" FAIL"));
  if (!passInHigh)
    allPassed = false;

  // Drive LOW
  digitalWrite(FEEDBACK_PIN, LOW);
  delay(5);
  readVal = as7343.getGPIOValue();
  Serial.print(F("  Arduino drives LOW  -> AS7343 reads: "));
  Serial.print(readVal ? F("HIGH") : F("LOW"));
  bool passInLow = (readVal == false);
  Serial.println(passInLow ? F(" PASS") : F(" FAIL"));
  if (!passInLow)
    allPassed = false;

  // Clean up - set pin back to input
  pinMode(FEEDBACK_PIN, INPUT);

  // ==========================================
  // Summary
  // ==========================================
  Serial.println();
  Serial.println(F("Summary:"));
  Serial.print(F("  Output mode:   "));
  Serial.println((passHigh && passLow) ? F("PASS") : F("FAIL"));
  Serial.print(F("  Inversion:     "));
  Serial.println((passInvHigh && passInvLow) ? F("PASS") : F("FAIL"));
  Serial.print(F("  Input mode:    "));
  Serial.println((passInHigh && passInLow) ? F("PASS") : F("FAIL"));

  Serial.println();
  Serial.print(F("RESULT: "));
  Serial.println(allPassed ? F("PASS") : F("FAIL"));
}

void loop() { delay(1000); }
