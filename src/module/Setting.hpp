#ifndef _SETTING_HPP_
#define _SETTING_HPP_
#include <Arduino.h>
#include "Utils/types.hpp"

class Setting
{
private:
public:
  bool absoluteMode = true;
  bool eAbsoluteMode = false;
  double_xyze_t stepsPerUnit; // steps/mm
  double_xyze_t maxAcceleration; // mm/s^2
  uint32_xyze_t maxAccelerateRate; // steps/s^2
  Setting();
  // ~Setting();
};

#endif 