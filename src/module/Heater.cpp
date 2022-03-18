#include "Heater.hpp"
#include "thermistor/thermistor_1.h"

Heater::Heater(uint8_t tempSensorPin, uint8_t heaterPin)
{
  _tempSensorPin = tempSensorPin;
  _heaterPin = heaterPin;
  pinMode(heaterPin, OUTPUT);
}

/**
 * @brief  Update heater pwm duty every 150ms in a interrupt service function.
 * @param  updateCycle should be 1ms to 250ms
 * @return
 */
void Heater::init(uint16_t updateCycle) {
  Serial.println("Start heater init.");

  // Pin8 analogWrite very noisy!
  // so I try to use the Timer4 to generate pwm for hotbed.
  // TCCR4A = 0; // set entire TCCR1A register to 0
  // TCCR4B = 0; // same for TCCR1B
  // TCNT4 = 0;  // initialize counter value to 0
  // Fast PWM 10-bit mode, top value is 1023, base on Table 17-2 of datasheet
  // Table 16-9. Clock Select Bit Description
  // Table 17-4. Compare Output Mode, Fast PWM
  // TCCR4A |= (1 << WGM41) | (1 << COM4A1) | (1 << COM4B1)| (1 << COM4C1);
  // TCCR4B |= (1 << CS40) | (1 << CS42) | (1 << WGM42) | (1 << WGM43); // try 64 prescaler, 16MHz/64= 250KHz
  // OCR4A = 100;// (must be <65526)
  // OCR4B = 200;// (must be <65526)
  // TCCR4A |= (1 << WGM40) | (1 << COM4A1) | (1 << COM4B1)| (1 << COM4C1);
  // TCCR4B |= (1 << CS40) | (1 << CS42); // 1024 prescaler
  // // ICR4 = 255;
  // OCR4C = 0;// (must be <65526)

  // Name
  // Clear Timer on Compare (CTC) modes

  // Data
  // Timer/Counter (TCNTn)
  // Output Compare Flag (OCFnx)
  // Output Compare outputs OCnx

  // Setting
  // Timer/Counter Control Registers (TCCRnA/B/C)
  // Timer Interrupt Mask Register (TIMSKn)
  // Output Compare Registers (OCRnA/B/C)
  // Waveform Generation mode (WGMn3:0)
  // Compare Output mode (COMnx1:0)

  cli(); // stop interrupts
  // set timer4 interrupt at 1Hz
  TCCR3A = 0; // set entire TCCR1A register to 0
  TCCR3B = 0; // same for TCCR1B
  TCNT3 = 0;  // initialize counter value to 0
  // set compare match register for 1hz increments
  // OCR3A = 15624 / 1; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // OCR4A = 24999; // try 0.1s , so 0.1 / (1/250,000) - 1 (must be <65536)
  OCR3A = updateCycle * 250 - 1;// (must be <65526)
  // turn on CTC mode
  TCCR3B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  // TCCR4B |= (1 << CS12) | (1 << CS10);
  TCCR3B |= (1 << CS11) | (1 << CS10); // try 64 prescaler, 16MHz/64= 250KHz
  // enable timer compare interrupt
  TIMSK3 |= (1 << OCIE3A);
  sei(); // allow interrupts

  Serial.print("Heater inited, Timer1 be used, OCR3A = ");
  Serial.println(OCR3A);
}

bool Heater::setTargetTemp(float targetTemp) {
  _targetTemp = targetTemp;
  pidIntegral = 0;
 return true;
}

void Heater::setPid(float p, float i, float d) {
  kp = p;
  ki = i;
  kd = d;
}


uint16_t Heater::calculatePid() {
  float curTemp = readTemp();
  // float err = _targetTemp - readTemp();
  float err = _targetTemp - curTemp;
  pidIntegral += err;
  if(pidIntegral > 400) pidIntegral = 400;
  // if(pidIntegral < -400) pidIntegral = -400;
  // if(pidIntegral <= -255) pidIntegral = -255;
  if(pidIntegral <= 0) pidIntegral = 0;
  float p = kp * err;
  float i = ki * pidIntegral;
  if(i >= 160) i = 160;
  float d = kd * (err - pidPrevErr);
  rawPwmDuty = p + i + d;
  // rawPwmDuty =
  //     kp * err
  //   + ki * pidIntegral
  //   + kd * (err - pidPrevErr);
  pwmDuty = rawPwmDuty;
  pidPrevErr = err;
  static uint32_t cnt = 0;
  if(cnt % 100 == 1) {
    Serial.print("curTemp ");
    Serial.print(curTemp);
    Serial.print(" pid term ");
    Serial.print(p);
    Serial.print(' ');
    Serial.print(i);
    Serial.print(' ');
    Serial.print(d);
    Serial.print("\n");
  }
  cnt ++;
  if (pwmDuty >= 255) pwmDuty = 255;
  if (pwmDuty <= 0) pwmDuty = 0;
  
  return (uint16_t)pwmDuty;
}

void Heater::update() {
  // if(_heaterPin == 8) {
  //   // OCR4A = ((int)calculatePid()) << 3;
  //   uint16_t pwmDuty = calculatePid();
  //   switch (pwmDuty)
  //   {
  //   case 0:
  //     digitalWrite(_heaterPin, 0);
  //     break;
  //   case 255:
  //     digitalWrite(_heaterPin, 1);
  //     break;
  //   default:
  //     // Fast PWM 10-bit mode
  //     TCCR4A |= (1 << WGM41) | (1 << WGM40) | (1 << COM4A1) | (1 << COM4B1)| (1 << COM4C1);
  //     TCCR4B |= (1 << CS40) | (1 << CS42); // 1024 prescaler
  //     OCR4C = pwmDuty << 2;
  //     break;
  //   } 
  // } else {
  //   analogWrite(_heaterPin, (int)calculatePid());
  // }
  analogWrite(_heaterPin, (int)calculatePid());
}

float Heater::readTemp() {
  float curTempVal = 0;
  for (uint16_t i = 0; i < 5; i++)
  {
    curTempVal += analogRead(_tempSensorPin);
  }
  curTempVal /= 5;

  for (uint16_t i = 0; i < sizeof(value2TempTable) / sizeof(value2temp_t); i++)
  {
    if(value2TempTable[i].value > curTempVal) {
      if (i == 0)
      {
        _temp = value2TempTable[0].celsius;
      }else {
        _temp = (value2TempTable[i].value - curTempVal) / (value2TempTable[i].value - value2TempTable[i-1].value)
        * (value2TempTable[i-1].celsius - value2TempTable[i].celsius)
        + value2TempTable[i].celsius;
        break;
      }
    }
  }

  return _temp;
}
