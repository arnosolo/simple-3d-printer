#ifndef _HEATER_H_
#define _HEATER_H_
#include <Arduino.h>

class Heater
{
private:
  float _temp = 0;
  float _maxTemp = 0;
  float _minTemp = 0;
  float _targetTemp = 0;
  uint8_t _tempSensorPin;
  uint8_t _heaterPin;
public:
  float kp = 3;
  float ki = 0.1;
  float kd = 0;
  float pidIntegral = 0;
  float pidPrevErr = 0;
  float rawPwmDuty = 0;
  float pwmDuty = 0;
  Heater(uint8_t tempSensorPin, uint8_t heaterPin);
  // ~Heater();
  bool setTargetTemp(float temp);
  void setPid(float p, float i, float d);
  float readTemp();
  uint16_t calculatePid();
  void update();
  static void init(uint16_t updateCycle); // ms
};

// extern Heater hotend;
// extern Heater hotbed;

#endif 