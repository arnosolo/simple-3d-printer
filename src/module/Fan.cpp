#include "Fan.hpp"

Fan::Fan(uint8_t fanPin) {
  _pin = fanPin;
}

void Fan::setDuty(int val) {
  if(val >= 255) val = 255;
  if(val <= 0) val = 0;
  analogWrite(_pin, val);
}
