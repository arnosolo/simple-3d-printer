#ifndef _STEPPER_H_
#define _STEPPER_H_
#include <Arduino.h>
#include "Planner.h"

// Int8 - [-128 : 127]
// Int16 - [-32768 : 32767]
// Int32 - [-2147483648 : 2147483647]
// Int64 - [-9223372036854775808 : 9223372036854775807]
// UInt8 - [0 : 255]
// UInt16 - [0 : 65535]
// UInt32 - [0 : 4294967295]
// UInt64 - [0 : 18446744073709551615]

class Stepper
{
private:
  uint8_t _enablePin = 0;
  uint8_t _stepPin = 0;
  uint8_t _dirPin = 0;
  bool _reverseDir = false;
public:
  float stepsPerUnit; // steps/mm
  int32_t deltaError = 0;
  int32_t position = 0;
  bool needAdvance;
  static block_t currentBlock;
  
  Stepper(
    uint8_t dirPin, 
    uint8_t stepPin, 
    uint8_t enablePin, 
    bool reverseDir, 
    float stepsPerUnit
  );
  // Stepper(uint8_t dirPin, uint8_t stepPin);
  // ~Stepper();
  static void init();
  void enable();
  void disable();
  void move(uint32_t steps, int16_t speed);
  void moveOneStep();
  void setStepPinHigh();
  void setStepPinLow();
  void setDir(int8_t dir);
};

#endif 