#ifndef _SETTING_H_
#define _SETTING_H_
#include <Arduino.h>
#include "Utils/types.h"

class Setting
{
private:
public:
  float_xyze_t stepsPerUnit; // steps/mm
  double_xyze_t maxAcceleration; // mm/s^2
  uint32_xyze_t maxAccelerateRate; // steps/s^2
  Setting();
  // ~Setting();
};

#endif 