#ifndef TCS34725SENSOR_H
#define TCS34725SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <TCS34725.h>   // hideakitai Library

// Globale Sensor-Objekte
extern TCS34725 tcs;

// Farben als Enum (wie bei dir)
enum BallColor {
  COLOR_RED,
  COLOR_GREEN,
  COLOR_BLUE,
  COLOR_UNKNOWN
};

// Rohdaten-Struct (leicht zu benutzen im Projekt)
struct ColorRaw {
  uint16_t r;
  uint16_t g;
  uint16_t b;
  uint16_t c;
};

// Funktionen
bool initColorSensor(TwoWire &wirePort = Wire);
bool readColorRaw(ColorRaw &out);
BallColor classifyBallColor(const ColorRaw &raw);

// Optional: Debug
const char* colorToString(BallColor c);
void printColorRaw(const ColorRaw &raw);

#endif
