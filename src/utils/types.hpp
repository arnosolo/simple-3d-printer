#ifndef _TYPES_HPP_
#define _TYPES_HPP_
#include <Arduino.h>

typedef struct {
  double x;
  double y;
  double z;
  double e;
} double_xyze_t;

typedef struct {
  float x;
  float y;
  float z;
  float e;
} float_xyze_t;

typedef struct {
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint32_t e;
} uint32_xyze_t;

typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
  int32_t e;
} int32_xyze_t;

typedef struct {
  int8_t x;
  int8_t y;
  int8_t z;
  int8_t e;
} int8_xyze_t;

typedef struct {
  volatile bool isBusy;
  volatile bool isReady;
  volatile bool isDone;
  bool needRecalculate;
  uint32_t id;

  double distance; // mm
  double stepsPerMm; // steps/mm
  int8_xyze_t dir; // -1 or 1
  int32_xyze_t startStep; // mm
  uint32_xyze_t steps; // steps
  uint32_t stepEventCount; // steps
  uint32_t stepEventCompleted; // steps
  uint32_t accelerateUntil;
  uint32_t decelerateAfter;
  
  double entrySpeed; // mm/s
  double exitSpeed; // mm/s
  double nominalSpeed; // mm/s
  uint32_t entryRate; // steps/s
  uint32_t exitRate; // steps/s
  uint32_t nominalRate; // steps/s
  uint32_t speedRate; // steps/s

  double acceleration; // steps/sec^2
  uint32_t accelerateRate; // steps/sec^2
} block_t;

#endif
