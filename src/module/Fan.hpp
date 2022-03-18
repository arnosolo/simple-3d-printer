#ifndef _FAN_HPP_
#define _FAN_HPP_
#include <Arduino.h>

class Fan
{
private:
  uint8_t _pin;
public:
  Fan(uint8_t fanPin);
  // ~Fan();
  void setDuty(int val);
};

#endif 