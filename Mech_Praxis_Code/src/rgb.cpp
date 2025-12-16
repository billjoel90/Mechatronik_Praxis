#include "rgb.h"

// Globales Objekt (wie bei euren Motoren)
TCS34725 tcs;

// ---- interne Parameter (kannst du später fein einstellen) ----
static const uint16_t CLEAR_MIN = 150; // zu dunkel -> unknown

// Schwellen (Startwerte!)
static const float RED_MIN   = 0.33f;
static const float GREEN_MIN = 0.33f;
static const float BLUE_MIN  = 0.28f;

static const float DIFF_RG = 0.03f;
static const float DIFF_RB = 0.03f;
static const float DIFF_GB = 0.03f;

// Blau etwas "freundlicher" (weil blau oft dunkler ist)
static const float DIFF_BLUE = 0.02f;

// -------------------------------------------------------------

bool initColorSensor(TwoWire &wirePort) {
  // Beim Mega ist SDA=20, SCL=21 automatisch über Wire
  wirePort.begin();

  if (!tcs.attach(wirePort)) {
    return false;
  }

  // Grundeinstellungen (kannst du anpassen)
  tcs.integrationTime(50);                // ms
  tcs.gain(TCS34725::Gain::X04);          // X01, X04, X16, X60
  tcs.scale(1.0f);
  tcs.enableColorTempAndLuxCalculation(false);

  return true;
}

bool readColorRaw(ColorRaw &out) {
  if (!tcs.available()) return false;

  TCS34725::RawData raw = tcs.raw();
  out.r = raw.r;
  out.g = raw.g;
  out.b = raw.b;
  out.c = raw.c;
  return true;
}

BallColor classifyBallColor(const ColorRaw &raw) {
  if (raw.c < CLEAR_MIN) return COLOR_UNKNOWN;

  float rn = (float)raw.r / (float)raw.c;
  float gn = (float)raw.g / (float)raw.c;
  float bn = (float)raw.b / (float)raw.c;

  float maxVal = rn;
  if (gn > maxVal) maxVal = gn;
  if (bn > maxVal) maxVal = bn;

  float diffRG = fabs(rn - gn);
  float diffRB = fabs(rn - bn);
  float diffGB = fabs(gn - bn);

  // ROT
  if (rn == maxVal && rn > RED_MIN && diffRG > DIFF_RG && diffRB > DIFF_RB)
    return COLOR_RED;

  // GRÜN
  if (gn == maxVal && gn > GREEN_MIN && diffRG > DIFF_RG && diffGB > DIFF_GB)
    return COLOR_GREEN;

  // BLAU (lockerer)
  if (bn == maxVal && bn > BLUE_MIN && diffRB > DIFF_BLUE && diffGB > DIFF_BLUE)
    return COLOR_BLUE;

  return COLOR_UNKNOWN;
}

const char* colorToString(BallColor c) {
  switch (c) {
    case COLOR_RED:     return "RED";
    case COLOR_GREEN:   return "GREEN";
    case COLOR_BLUE:    return "BLUE";
    default:            return "UNKNOWN";
  }
}

void printColorRaw(const ColorRaw &raw) {
  Serial.print("R: "); Serial.print(raw.r);
  Serial.print(" G: "); Serial.print(raw.g);
  Serial.print(" B: "); Serial.print(raw.b);
  Serial.print(" C: "); Serial.println(raw.c);
}
