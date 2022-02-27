#ifndef _UTILS_H_
#define _UTILS_H_
#include <Arduino.h>
#include "Utils/types.h"

class Utils
{
private:
public:
  Utils();
  static uint32_t getMax(uint32_xyze_t data);
  static uint32_t getMax(uint32_t a, uint32_t b, uint32_t c);
  static float getAbs(float value);
};

#endif 