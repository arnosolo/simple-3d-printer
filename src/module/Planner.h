#ifndef _PLANNER_H_
#define _PLANNER_H_
#include <Arduino.h>
#include "config/config.h"
#include "Utils/Queue.h"
#include "Utils/types.h"
#include "config/Setting.h"
#include "gcode/Gcode.h"

class Planner
{
private:
public:
  static Queue<block_t> blockQueue;
  Planner();
  static void planBufferLine(Gcode* gcode, Setting* setting, bool absoluteMode);
  // ~Planner();
  static uint32_t getMax(uint32_xyze_t data);
  static double_xyze_t normalize(double_xyze_t vector);

  static double getMaxAllowSpeed(double startSpeed, double acceleration, double distance);

  static void calculateTrapezoid(block_t* block);

//   static void recalculate();
  static void reverseCheck();
  static void forwardCheck();
  static void recalculateTrapezoids();

  static double calculateAccelerateDistance(
      double startRate, double targetRate, double accRate);

  static double intersectionDistance(
      double entryRate, double exitRate, double accRate, double totalSteps);
};

#endif 