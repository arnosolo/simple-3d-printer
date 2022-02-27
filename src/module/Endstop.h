#ifndef _ENDSTOP_H_
#define _ENDSTOP_H_
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