#include "Stepper.hpp"

block_t* Stepper::curBlock = nullptr;

/**
 * @brief  Stepper motor init
 * @param  enablePin Set as 0 if you board don't have one.
 * @param  defaultDir 1 or 0
 * @return
 */
Stepper::Stepper(uint8_t dirPin, uint8_t stepPin, uint8_t enablePin, bool reverseDir, float stepsPerUnitInput){
  _dirPin = dirPin;
  _stepPin = stepPin;
  _enablePin = enablePin;
  _reverseDir = reverseDir;
  posInSteps = 0;
  needAdvance = false;
  pinMode(enablePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  digitalWrite(stepPin, LOW);
  this->disable();
}

void Stepper::enable() {
  digitalWrite(_enablePin, 0); // For A4988 Low = enable
}

void Stepper::disable() {
  digitalWrite(_enablePin, 1); // For A4988 Low = enable
}

/**
 * @brief  Demond motor to move
 * @param  steps unit: steps, must >0
 * @param  speed unit: steps/s, must >0
 * @return
 */
void Stepper::move(uint32_t steps, uint32_t speed){
  uint32_t interval = 1000000UL / speed;
  for (uint32_t i = 0; i < steps; i++) {
    digitalWrite(_stepPin, 1);
    digitalWrite(_stepPin, 0);
    delayMicroseconds(interval);
  }
}

void Stepper::moveOneStep(){
  digitalWrite(_stepPin, HIGH);
  digitalWrite(_stepPin, LOW);
}

void Stepper::setHigh(){
  digitalWrite(_stepPin, HIGH);
}

void Stepper::setLow(){
  digitalWrite(_stepPin, LOW);
}

/**
 * @brief  Set motor dir
 * @param  dir should be -1 or 1
 * @return
 */
void Stepper::setDir(int8_t dir) {
  if((dir == 1 && !_reverseDir) || (dir == -1 && _reverseDir)) {
    digitalWrite(_dirPin, 1);
  } else {
    digitalWrite(_dirPin, 0);
  }
}

/**
 * @brief  Setup timer for motor control
 * @param
 * @return
 */
void Stepper::init(){
  Serial.println("Start Stepper init.");

  // cli(); // stop interrupts
  // // set timer4 interrupt at 1Hz
  // TCCR1A = 0; // set entire TCCR1A register to 0
  // TCCR1B = 0; // same for TCCR1B
  // TCNT1 = 0;  // initialize counter value to 0
  // // set compare match register for 1hz increments
  // // turn on CTC mode
  // TCCR1B |= (1 << WGM12);
  // // Set CS12 and CS10 bits for 1024 prescaler
  // // TCCR4B |= (1 << CS12) | (1 << CS10);
  // // Set CS11 bit for 8 prescaler, so every 0.5us TCNT1 increase 1
  // TCCR1B |= (1 << CS11);
  // // enable timer compare interrupt
  // TIMSK1 |= (1 << OCIE1A);
  // // OCR4A = 24999; // try 0.1s , so 0.1 / (1/250,000) - 1 (must be <65536)
  // OCR1A = 24999; // max=65535, 32767us, 32ms

  // sei(); // allow interrupts

  cli(); // stop interrupts
  // set timer4 interrupt at 1Hz
  TCCR5A = 0; // set entire TCCR1A register to 0
  TCCR5B = 0; // same for TCCR1B
  TCNT5 = 0;  // initialize counter value to 0
  // set compare match register for 1hz increments
  // turn on CTC mode
  TCCR5B |= (1 << WGM12);
  TCCR5B |= (1 << CS51); // 8 prescale
  // enable timer compare interrupt
  TIMSK5 |= (1 << OCIE1A);
  OCR5A = 2000;

  sei(); // allow interrupts

  Serial.print("Stepper inited, Timer5 was used, OCR5A = ");
  Serial.println(OCR5A);
}

void Stepper::isr() {
  
}
