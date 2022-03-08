#include "Endstop.h"

Endstop::Endstop(uint8_t pin) {
  _pin = pin;
  pinMode(pin, INPUT);
}

bool Endstop::isTriggered() {
  if(digitalRead(_pin) == HIGH) {
    return true;
  } else {
    return false;
  }
}