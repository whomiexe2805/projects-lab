/*
  S-TECH DIGITAL DICE
  Board: Arduino Nano (ATmega328P, 5V/16MHz)
  Display: SSD1306 128x64 I2C OLED
  Input: One momentary push button

  IMPORTANT:
  This is an experimental noise-sampling demonstration, NOT a
  validated True Random Number Generator (TRNG). An unconnected
  A0 pin is not a dependable entropy source. Results depend on
  the board, wiring and electrical environment. Do not use this
  sketch for security, cryptography, gambling or other high-stakes use.

  Libraries:
    - Adafruit GFX Library
    - Adafruit SSD1306

  Wiring:
    OLED SDA -> A4
    OLED SCL -> A5
    OLED VCC -> module-rated supply (check your OLED breakout)
    OLED GND -> GND
    Button    -> D2 and GND (uses INPUT_PULLUP)
    Experimental sample input -> A0 (left unconnected for this demo)

  OLED I2C address is set to 0x3C below. Change to 0x3D if needed.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_ADDRESS   0x3C

const byte BUTTON_PIN = 2;
const byte NOISE_PIN  = A0;

// Limits work if the sampled input does not produce differing pairs.
// This prevents the dice from getting stuck forever.
const unsigned long ENTROPY_TIMEOUT_MS = 2500;
const unsigned int SAMPLE_GAP_US = 37;

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

bool oledReady = false;

// --------------------------------------------------
// UI helpers
// --------------------------------------------------

void showMessage(const __FlashStringHelper *line1,
                 const __FlashStringHelper *line2 = nullptr) {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 12);
  display.println(line1);

  if (line2 != nullptr) {
    display.setCursor(0, 30);
    display.println(line2);
  }

  display.display();
}

void drawDice(byte number) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(38, 1);
  display.println(F("S-TECH DIGITAL DICE"));

  // Dice body
  display.drawRoundRect(36, 13, 56, 50, 8, SSD1306_WHITE);

  // Pip locations: top-left, top-right, middle-left,
  // middle-right, bottom-left, bottom-right.
  const int8_t pipX[6] = {49, 79, 49, 79, 49, 79};
  const int8_t pipY[6] = {25, 25, 38, 38, 51, 51};

  const bool patterns[6][6] = {
    {0, 0, 0, 1, 0, 0}, // 1: center is drawn separately below
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 1, 0, 1},
    {1, 1, 0, 0, 1, 1},
    {1, 1, 0, 1, 1, 1},
    {1, 1, 1, 1, 1, 1}
  };

  if (number == 1) {
    display.fillCircle(64, 38, 3, SSD1306_WHITE);
  } else {
    for (byte i = 0; i < 6; i++) {
      if (patterns[number - 1][i]) {
        display.fillCircle(pipX[i], pipY[i], 3, SSD1306_WHITE);
      }
    }
  }

  display.display();
}

void showRolling(byte frame) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(17, 8);
  display.println(F("ROLLING"));

  display.setTextSize(1);
  display.setCursor(15, 37);
  display.println(F("Collecting sample bits"));

  // Simple progress animation
  for (byte i = 0; i < 8; i++) {
    if (i <= frame) {
      display.fillRect(16 + (i * 12), 53, 8, 4, SSD1306_WHITE);
    } else {
      display.drawRect(16 + (i * 12), 53, 8, 4, SSD1306_WHITE);
    }
  }
  display.display();
}

// --------------------------------------------------
// Experimental bit extraction
// --------------------------------------------------

// Von Neumann extractor:
// input pairs 01 -> 0, 10 -> 1; discard 00 and 11.
// This can reduce bias under suitable assumptions, but it
// cannot create entropy or prove that A0 contains entropy.
bool getExperimentalBit(byte &result, unsigned long startedAt) {
  while (millis() - startedAt < ENTROPY_TIMEOUT_MS) {
    byte a = analogRead(NOISE_PIN) & 1;
    delayMicroseconds(SAMPLE_GAP_US);
    byte b = analogRead(NOISE_PIN) & 1;

    if (a != b) {
      result = (a == 1) ? 1 : 0;
      return true;
    }
  }

  return false;
}

// Collect 3 bits and reject 6 and 7 to avoid modulo bias.
// Returns false if the input does not provide enough differing pairs.
bool generateExperimentalDiceNumber(byte &diceNumber) {
  const unsigned long startedAt = millis();

  while (millis() - startedAt < ENTROPY_TIMEOUT_MS) {
    byte value = 0;

    for (byte i = 0; i < 3; i++) {
      byte bitValue;

      if (!getExperimentalBit(bitValue, startedAt)) {
        return false;
      }

      value = (value << 1) | bitValue;
    }

    if (value < 6) {
      diceNumber = value + 1;
      return true;
    }
    // Values 6 and 7 are rejected; collect another 3 bits.
  }

  return false;
}

// --------------------------------------------------
// Button handling
// --------------------------------------------------

bool waitForButtonPress() {
  static bool previousState = HIGH;
  bool currentState = digitalRead(BUTTON_PIN);

  if (previousState == HIGH && currentState == LOW) {
    delay(25); // debounce
    if (digitalRead(BUTTON_PIN) == LOW) {
      previousState = LOW;
      return true;
    }
  }

  previousState = currentState;
  return false;
}

void waitForButtonRelease() {
  while (digitalRead(BUTTON_PIN) == LOW) {
    delay(5);
  }
  delay(25);
}

// --------------------------------------------------
// Setup and main loop
// --------------------------------------------------

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(NOISE_PIN, INPUT);

  Wire.begin();

  oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);

  if (!oledReady) {
    // OLED not detected: keep the Nano from repeatedly trying
    // to draw to a missing display.
    while (true) {
      delay(1000);
    }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(17, 8);
  display.println(F("S-TECH"));

  display.setTextSize(1);
  display.setCursor(22, 34);
  display.println(F("DIGITAL DICE"));
  display.setCursor(17, 52);
  display.println(F("Press button to roll"));

  display.display();
}

void loop() {
  if (!waitForButtonPress()) {
    return;
  }

  showRolling(0);
  delay(100);
  showRolling(2);
  delay(100);
  showRolling(4);
  delay(100);
  showRolling(7);

  byte result = 0;

  if (generateExperimentalDiceNumber(result)) {
    drawDice(result);
  } else {
    showMessage(
      F("NO USABLE SAMPLE"),
      F("Check A0 demo input")
    );
  }

  waitForButtonRelease();
}
