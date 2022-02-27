#include "Utils.h"

uint32_t Utils::getMax(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t max = a;
  if(b > max) max = b;
  if(c > max) max = c;
  return max;
}

uint32_t Utils::getMax(uint32_xyze_t data) {
  uint32_t max = data.x;
  if(data.y > max) max = data.y;
  if(data.z > max) max = data.z;
  if(data.e > max) max = data.e;
  return max;
}

float Utils::getAbs(float value) {
  if(value < 0) {
    return -value;
  } else {
    return value;
  }
}
