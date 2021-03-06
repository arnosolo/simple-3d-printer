#ifndef _PLANNER_HPP_
#define _PLANNER_HPP_
#include <Arduino.h>
#include "config/config.hpp"
#include "Utils/Queue.hpp"
#include "Utils/types.hpp"
#include "module/Setting.hpp"
#include "gcode/Gcode.hpp"

class Planner {
private:
public:
  static Queue<block_t> blockQueue;
  static double_xyze_t startPos;
  static double_xyze_t targetPos;
  static double_xyze_t prevStartPos;
  static double_xyze_t prevTargetPos;
  static bool prevSegmentUsed;

  Planner();
  static bool planBufferLine(double_xyze_t startPos, double_xyze_t targetPos, double nominalSpeed, double acceleration, Setting* setting);

  static void calculateTrapezoid(block_t* block);
  static void reverseCheck();
  static void forwardCheck();
  static void speedLimitCheck();
  static void recalculateTrapezoids();

  static uint32_t getMax(uint32_xyze_t data);
  static double_xyze_t normalize(double_xyze_t vector);
  static double getMaxAllowSpeed(double startSpeed, double acceleration, double distance);
  static uint32_t getMaxAllowRate(uint32_t startRate, uint32_t accelerateRate, uint32_t steps);
  static double getMaxAllowSpeedSq(double startSpeedSq, double acceleration, double distance);
  static double calculateAccelerateDistance(double startRate, double targetRate, double accRate);
  static uint32_t calculateAccSteps(uint32_t startRate, uint32_t targetRate, uint32_t accRate);
  static double intersectionDistance(double entryRate, double exitRate, double accRate, double totalSteps);
  static uint32_t intersectionSteps(uint32_t entryRate, uint32_t exitRate, uint32_t accRate, uint32_t totalSteps);
};

#endif 