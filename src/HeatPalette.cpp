#include "HeatPalette.h"
#include <stdlib.h>
#include <math.h>

const uint8_t* buildPalette(int n) {
  uint8_t* pal = new uint8_t[n*3];
  const float step = 360.0f / (float)n;
  float hue = 0.0f;
  const float saturation = 0.85f;
  const float value = 0.9f;
  const float chroma = saturation * value;
  const float m = value - chroma;
  const uint8_t colZero = (uint8_t)(m * 255.0f);
  const uint8_t colC = (uint8_t)((chroma + m) * 255.0f);
  uint8_t* p = pal;
  for(int i = 0; i < n; i++, hue += step, p += 3) {
    const float h = hue / 60.0f;
    const float x = chroma * (1.0f - fabs(fmod(h, 2.0f) - 1.0f));
    const uint8_t colX = (uint8_t)((x + m) * 255.0f);
    
    if(h < 1) {
      p[0] = colC; p[1] = colX; p[2] = colZero;
    } else if(h < 2) {
      p[0] = colX; p[1] = colC; p[2] = colZero;
    } else if(h < 3) {
      p[0] = colZero; p[1] = colC; p[2] = colX;
    } else if(h < 4) {
      p[0] = colZero; p[1] = colX; p[2] = colC;
    } else if(h < 5) {
      p[0] = colX; p[1] = colZero; p[2] = colC;
    } else if(h < 6) {
      p[0] = colC; p[1] = colZero; p[2] = colX;
    }
  }
  return pal;
}

// Return the RGB color associated a normalized scalar value.
const uint8_t* getColor(float x) {
  const int palSize = 256;
  static const uint8_t* pal = buildPalette(palSize);
  return &pal[(int)(x*(float)(palSize - 1))*3];
}
