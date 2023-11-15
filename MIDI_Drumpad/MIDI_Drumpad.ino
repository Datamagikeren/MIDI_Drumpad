#include "Wire.h"
#include "Adafruit_MPR121.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BLEMIDI.h"
#include "BluefruitConfig.h"
#include <FastLED.h>

#define NUM_LEDS 16
#define LED_PIN 6
CRGB leds[NUM_LEDS];

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_MPR121 cap = Adafruit_MPR121();
Adafruit_BLEMIDI blemidi(ble);

#define BUTTONS  6
#define IRQ_PIN  A4

uint16_t lasttouched = 0;
uint16_t currtouched = 0;

int pitch[] = {36, 38, 42, 46, 44, 55};
CRGB colors[] = {CRGB::Red, CRGB::Blue, CRGB::Green, CRGB::Yellow, CRGB::Purple, CRGB::Cyan};

void setup() {
  Serial.begin(115200);
  
  pinMode(IRQ_PIN, INPUT);

  if (! cap.begin(0x5A)) {
    while (1);
  }

  if (!ble.begin(VERBOSE_MODE)) {
    while (1);
  }

  ble.echo(false);
  
  if (!blemidi.begin(true)) {
    while (1);
  }
    
  ble.verbose(false);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();
}

void loop() {
  if (!ble.isConnected()) {
    pulsateLEDs(CRGB::Blue);
  } else {
    if (digitalRead(IRQ_PIN) == LOW) {
      readButtons();
    }
  }
}

void readButtons() {
  currtouched = cap.touched();

  for (uint8_t i = 0; i < BUTTONS; i++) {
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i))) {
      midi(0, 0x9, pitch[i], 100);
      fillAll(colors[i]);
    }

    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i))) {
      midi(0, 0x8, pitch[i], 0);
      fillAll(CRGB::Black);
    }
  }

  FastLED.show();

  lasttouched = currtouched;
}

void midi(byte channel, byte command, byte arg1, byte arg2) {
  byte combined = command;
  if (combined < 128) {
    combined <<= 4;
    combined |= channel;
  }
  blemidi.send(combined, arg1, arg2);
}

void fillAll(CRGB color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }
}

void pulsateLEDs(CRGB color) {
  static uint8_t brightness = 0;
  static int8_t direction = 1;
  const uint8_t maxBrightness = 127;  // set max brightness to 50%

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
    leds[i].fadeLightBy(255 - brightness);
  }

  FastLED.show();

  // Only change brightness if not at max or min value
  if (!(brightness == 0 && direction == -1) && !(brightness == maxBrightness && direction == 1)) {
    brightness += direction * 4;
  }

  if (brightness == 0 || brightness == maxBrightness) {
    direction *= -1;
  }

  delay(10);
}

