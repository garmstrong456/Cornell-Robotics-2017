/*
 * Simple RGB Led example
 * By Greg Armstrong
 * July 2017
 * 
 * Written for a single Adafruit Neopixel LED on a TEC-Bot 4.2 robot
 * This will cycle the LED through colors as a demonstration of what it can do
 * 
 * PINOUT (See adafruit website for LED pin diagram):
 * www.adafruit.com/product/1938
 * 
 * Data In:  DIO Pin 7
 * +5:       5V on Arduino (safe because we are only using 1 LED)
 * GND:      GND
 * Data Out: Not Connected
 */

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define LED_PIN 7

#define NUMPIXELS 1

Adafruit_NeoPixel led = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_RGB + NEO_KHZ800);

void setup() {
    led.begin();
}

void loop() {
    for (int i=0; i<255; i++) {
        led.setPixelColor(0, colorWheel(i));
        Serial.println(i);
        led.show();
        delay(10);
    }
}

//Takes a value between 0 and 255
//Returns an RGB color
//As hue moves from 0 - 255 the RGB color will cycle
//from r - g - b and back to r
//In other words: converts HSV(WheelPos, 255, 255) to RGB
uint32_t colorWheel(byte hue) {
    hue = 255 - hue;
    if (hue < 85) {
        return led.Color(255 - hue*3, 0, hue*3);
    }
    if (hue < 170) {
        hue -= 85;
        return led.Color(0, hue*3, 255 - hue*3);
    }
    hue -= 170;
    return led.Color(hue*3, 255 - hue*3, 0);
}

