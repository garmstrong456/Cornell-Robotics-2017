/*
 * Orientation LED
 * Greg Armstrong
 * July 2017
 */

#include <math.h>
#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>

FaBo9Axis mpu;

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
    #include <avr/power.h>
#endif

#define LED_PIN 7
#define NUMPIXELS 1

Adafruit_NeoPixel led = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_RGB + NEO_KHZ800);

void setup() {
    Serial.begin(9600);
    Serial.println("configuring mpu...");

    if (mpu.begin()) {
        Serial.println("configured");
    } else {
        Serial.println("device error");
        while(1);
    }

    led.begin();
}

//float hueAvg = 0, decayFactor = 0.99;

void loop() {
    float ax, ay, az;
    mpu.readAccelXYZ(&ax, &ay, &az);
    float hue = 255*(atan2(az, ay) + M_PI)/(2*M_PI);
    //hueAvg = hueAvg*decayFactor + (1 - decayFactor)*hue;

    led.setPixelColor(0, colorWheel(int(hue)));
    Serial.println(hue);
    led.show();
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
