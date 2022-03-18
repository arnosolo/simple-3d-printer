#ifndef _ENDSTOP_HPP_
#define _ENDSTOP_HPP_
#include <Arduino.h>

class Endstop
{
private:
  uint8_t _pin;
public:
  Endstop(uint8_t pin);
  // ~Endstop();
  bool isTriggered();
};

#endif 