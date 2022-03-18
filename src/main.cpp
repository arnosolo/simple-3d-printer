#include <Arduino.h>
#include "config/pin.hpp"
#include "config/config.hpp"
#include "gcode/Gcode.hpp"
#include "module/Heater.hpp"
#include "module/Fan.hpp"
#include "module/Setting.hpp"
#include "module/Stepper.hpp"
#include "module/Planner.hpp"
#include "module/Endstop.hpp"
#include "module/SDcard.hpp"
#include "Utils/Queue.hpp"
#include "Utils/types.hpp"
#include "Utils/Array.hpp"

Queue<String> gcodeStrQueue = Queue<String>(5);
block_t initBlock = {};
block_t* curBlock = nullptr;
Setting setting;
Heater hotend(PIN_HOTEND_SENSOR, PIN_HOTEND);
Heater hotbed(PIN_HOTBED_SENSOR, PIN_HOTBED);
Fan fan(PIN_FAN);
Stepper motorX(PIN_X_DIR, PIN_X_STEP, PIN_X_EN, REVERSE_X_DIR, STEPS_PER_UNIT_X);
Stepper motorY(PIN_Y_DIR, PIN_Y_STEP, PIN_Y_EN, REVERSE_Y_DIR, STEPS_PER_UNIT_Y);
Stepper motorZ(PIN_Z_DIR, PIN_Z_STEP, PIN_Z_EN, REVERSE_Z_DIR, STEPS_PER_UNIT_Z);
Stepper motorZ1(PIN_Z1_DIR, PIN_Z1_STEP, PIN_Z1_EN, REVERSE_Z1_DIR, STEPS_PER_UNIT_Z);
Stepper motorE(PIN_E_DIR, PIN_E_STEP, PIN_E_EN, REVERSE_E_DIR, STEPS_PER_UNIT_E);
Endstop xMin(PIN_X_MIN);
Endstop yMin(PIN_Y_MIN);
Endstop zMin(PIN_Z_MIN);
Endstop z1Min(PIN_Z1_MIN);
SDcard sdCard(PIN_SPI_CS);

void setup() {
  setting.stepsPerUnit.x = STEPS_PER_UNIT_X;
  setting.stepsPerUnit.y = STEPS_PER_UNIT_Y;
  setting.stepsPerUnit.z = STEPS_PER_UNIT_Z;
  setting.stepsPerUnit.e = STEPS_PER_UNIT_E;
  setting.maxAcceleration.x = MAX_ACCELERATION_X;
  setting.maxAcceleration.y = MAX_ACCELERATION_Y;
  setting.maxAcceleration.z = MAX_ACCELERATION_Z;
  setting.maxAcceleration.e = MAX_ACCELERATION_E;
  
  Serial.begin(115200);
  Heater::init(200);
  Stepper::init();
  hotend.setPid(HOTEND_KP, HOTEND_KI, HOTEND_KD);
  hotbed.setPid(HOTBED_KP, HOTBED_KI, HOTBED_KD);

  curBlock = &initBlock;
  curBlock->isDone = true;
  curBlock->isBusy = false;
  Gcode::prevAcceleration = 1000; // mm/s

  // motorX.disable();
  // motorY.disable();
  // motorZ.disable();
  // motorZ1.disable();
  // motorE.disable();
}

// Motor control isr
ISR(TIMER5_COMPA_vect) {
  static uint32_t interval = 1000;
  static uint8_t stepLoops = 1;
  // static uint32_t accTime = 0; // us
  // static uint32_t decTime = 0; // us
  static int32_xyze_t stepErr = {};
  static int32_t ySteps = 0;
  
  if(curBlock->isBusy) {
    if(curBlock->stepEventCompleted == 0) interval = 0;
    // Which axis need advance one step during one step event
    for(uint8_t i=0; i<stepLoops; i++) {
      if(curBlock->stepEventCompleted >= curBlock->stepEventCount) break; // this is easy to forget but necessary

      motorX.deltaError += curBlock->steps.x;
      if (motorX.deltaError >= 0) {
        motorX.moveOneStep();
        motorX.posInSteps += curBlock->dir.x;
        motorX.deltaError -= curBlock->stepEventCount;
      }

      motorY.deltaError += curBlock->steps.y;
      if (motorY.deltaError >= 0) {
        motorY.moveOneStep();
        motorY.posInSteps += curBlock->dir.y;
        motorY.deltaError -= curBlock->stepEventCount;
        ySteps++;
      }

      motorE.deltaError += curBlock->steps.e;
      if (motorE.deltaError >= 0) {
        motorE.moveOneStep();
        motorE.posInSteps += curBlock->dir.e;
        motorE.deltaError -= curBlock->stepEventCount;
      }

      motorZ.deltaError += curBlock->steps.z;
      if (motorZ.deltaError >= 0) {
        motorZ.moveOneStep();
        motorZ1.moveOneStep();
        motorZ.posInSteps += curBlock->dir.z;
        motorZ1.posInSteps += curBlock->dir.z;
        motorZ.deltaError -= curBlock->stepEventCount;
      }

      if(xMin.isTriggered() || yMin.isTriggered() || zMin.isTriggered() || z1Min.isTriggered()){
        curBlock->stepEventCompleted = curBlock->stepEventCount;
      }

      curBlock->stepEventCompleted ++;
    }

    // What phase is current step event in?
    if (curBlock->stepEventCompleted >= curBlock->stepEventCount) { // block done
      curBlock->isBusy = false; // mark current block as completed
      curBlock->isDone = true; // mark current block as completed
      curBlock->isReady = false;
      // Serial.print("block");
      // Serial.print(curBlock->id);
      // Serial.print(" stepLoops ");
      // Serial.print(stepLoops);
      // Serial.print(" yStep isr vs plan ");
      // Serial.print(ySteps);
      // Serial.print(' ');
      // Serial.println(curBlock->steps.y);
      ySteps = 0;
      // prevBlockId = curBlock->id;
      Planner::blockQueue.dequeue();
    } else if (curBlock->stepEventCompleted <= curBlock->accelerateUntil) { // acc phase
      curBlock->speedRate += (interval * curBlock->accelerateRate) / 1000000UL;
      // curBlock->speedRate = sqrt(curBlock->stepEventCompleted * curBlock->accelerateRate * 2UL + sq(curBlock->entryRate));
      // curBlock->speedRate = curBlock->entryRate + (accTime * curBlock->accelerateRate) / 1000000UL;
      // accTime += interval;
      if(curBlock->speedRate > curBlock->nominalRate) curBlock->speedRate = curBlock->nominalRate;
    } else if (curBlock->stepEventCompleted > curBlock->decelerateAfter) { // dec phase
      curBlock->speedRate -= (interval * curBlock->accelerateRate) / 1000000UL;
      // curBlock->speedRate = sqrt(-(curBlock->stepEventCompleted - curBlock->decelerateAfter) * curBlock->accelerateRate * 2UL + sq(curBlock->nominalRate));
      // curBlock->speedRate = curBlock->nominalRate - (decTime * curBlock->accelerateRate) / 1000000UL;
      // decTime += interval;
      if(curBlock->speedRate < curBlock->exitRate) curBlock->speedRate = curBlock->exitRate;
    } else { // cruise phase
    }

    interval = 1000000UL / curBlock->speedRate;
    if(curBlock->speedRate > 10000UL) {
      stepLoops = 4;
      interval *= 4;
    } else if(curBlock->speedRate > 5000UL) {
      stepLoops = 2;
      interval *= 2;
    } else {
      stepLoops = 1;
    }

  } else {
    // If there is no block available, set interupt interval as 1ms
    interval = 1000UL;
  }

  if (curBlock->isDone && (Planner::blockQueue._length > 0) && Planner::blockQueue._data[Planner::blockQueue._firstIndex].isReady) {
    curBlock = &Planner::blockQueue._data[Planner::blockQueue._firstIndex];
    motorX.enable();
    motorY.enable();
    motorZ.enable();
    motorZ1.enable();
    motorE.enable();
    motorX.setDir(curBlock->dir.x);
    motorY.setDir(curBlock->dir.y);
    motorZ.setDir(curBlock->dir.z);
    motorZ1.setDir(curBlock->dir.z);
    motorE.setDir(curBlock->dir.e);
    motorX.deltaError = -(curBlock->stepEventCount / 2);
    motorY.deltaError = motorX.deltaError;
    motorZ.deltaError = motorX.deltaError;
    motorE.deltaError = motorX.deltaError;
    curBlock->speedRate = curBlock->entryRate;
    curBlock->isBusy = true;
    curBlock->isDone = false;
    stepLoops = 1;
    Serial.print("Exec block");
    Serial.print(curBlock->id);
    stepErr.x = curBlock->startStep.x - motorX.posInSteps;
    stepErr.y = curBlock->startStep.y - motorY.posInSteps;
    stepErr.z = curBlock->startStep.z - motorZ.posInSteps;
    stepErr.e = curBlock->startStep.e - motorE.posInSteps;
    Serial.print(" stepErr ");
    Serial.print(stepErr.x);
    Serial.print(' ');
    Serial.print(stepErr.y);
    Serial.print(' ');
    Serial.print(stepErr.z);
    Serial.print(' ');
    Serial.print(stepErr.e);
    Serial.print(" queLen ");
    Serial.println(Planner::blockQueue._length);
  }

  // Set ISR interval
  // If current speed is 1000steps/s, set interupt interval as 1000us(1/1000s)
  OCR5A = (interval * 2UL); // us * 16 - 1
}

// Heater isr
ISR(TIMER3_COMPA_vect) {
  hotbed.update();
  hotend.update();
}

void loop() {
  // Check endstops
  if (xMin.isTriggered()) {
    Serial.println("Endstop xMin triggered");
  }
  if (yMin.isTriggered()) {
    Serial.println("Endstop yMin triggered");
  }
  if (zMin.isTriggered()) {
    Serial.println("Endstop zMin triggered");
  }
  if (z1Min.isTriggered()) {
    Serial.println("Endstop z1Min triggered");
  }
  
  // Read Gcode from serial
  if (Serial.available() && !gcodeStrQueue.isFull()) {
    String str = Serial.readStringUntil('\n');
    Serial.print("Get a cmd from host: ");
    Serial.println(str);
    gcodeStrQueue.enqueue(str);
  }

  // Read Gcode from SD card
  if (sdCard.sdFileSelected && !gcodeStrQueue.isFull()) {
    String str = sdCard.selectedFile.readStringUntil('\n');
    // Serial.print("Read gcode from SD card ");
    // Serial.println(str);
    while(str.charAt(0) == ';'){
      str = sdCard.selectedFile.readStringUntil('\n');
    }
    gcodeStrQueue.enqueue(str);
    if(sdCard.selectedFile.position() >= sdCard.selectedFile.size()) {
      sdCard.sdFileSelected = false;
      sdCard.selectedFile.close();
      Serial.println("Print Complete");
    }
  }

  // Parse Gcode
  if (!gcodeStrQueue.isEmpty() && !Planner::blockQueue.isFull()){
    Gcode gcode = Gcode::parse(gcodeStrQueue.dequeue());
    static double_xyze_t startPos = {};
    static double_xyze_t targetPos = {};
    static double_xyze_t prevStartPos = {};
    static double_xyze_t prevTargetPos = {};
    static bool prevSegmentUsed = true;

    if (gcode.cmdtype == 'G') {
        switch (gcode.cmdnum) {
        // G01 - linear motion
        case 0:
        case 1:
          {
            // Serial.print("Parse result: G1 ");
            // if(gcode.hasX){
            //   Serial.print("X");
            //   Serial.print(gcode.X, 3);
            // } 
            // if(gcode.hasY){
            //   Serial.print(" Y");
            //   Serial.print(gcode.Y, 3);
            // } 
            // if(gcode.hasZ){
            //   Serial.print(" Z");
            //   Serial.print(gcode.Z, 3);
            // } 
            // if(gcode.hasE){
            //   Serial.print(" E");
            //   Serial.print(gcode.E, 5);
            // } 
            // if(gcode.hasF){
            //   Serial.print(" F");
            //   Serial.print(gcode.F);
            // }
            // Serial.println();

            if(prevSegmentUsed) {
              startPos = prevTargetPos;
            } else {
              startPos = prevStartPos;
            }

            if(setting.absoluteMode) {
              if(gcode.hasX) targetPos.x = gcode.X;
              if(gcode.hasY) targetPos.y = gcode.Y;
              if(gcode.hasZ) targetPos.z = gcode.Z;
              if(gcode.hasE) {
                // Prevent parse E716.16 as E7
                if(abs(gcode.prevE - gcode.E) < 100) {
                  targetPos.e = gcode.E;
                }
              };
            } else {
              targetPos.x = gcode.hasX ? (targetPos.x + gcode.X) : targetPos.x;
              targetPos.y = gcode.hasY ? (targetPos.y + gcode.Y) : targetPos.y;
              targetPos.z = gcode.hasZ ? (targetPos.z + gcode.Z) : targetPos.z;
              targetPos.e = gcode.hasE ? (targetPos.e + gcode.E) : targetPos.e;
            }

            if(!setting.eAbsoluteMode && gcode.hasE) {
              startPos.e = 0;
              targetPos.e = gcode.E;
            }
            
            double nominalSpeed = gcode.hasF ? (gcode.F / 60) : (gcode.prevFeedSpeed / 60);
            double acceleration = gcode.prevAcceleration;

            prevSegmentUsed = Planner::planBufferLine(startPos, targetPos, nominalSpeed, acceleration, &setting);

            prevStartPos = startPos;
            prevTargetPos = targetPos;
            if (gcode.hasF) Gcode::prevFeedSpeed = gcode.F;
            if (gcode.hasX) Gcode::prevX = gcode.X;
            if (gcode.hasY) Gcode::prevY = gcode.Y;
            if (gcode.hasZ) Gcode::prevZ = gcode.Z;
            if (gcode.hasE) Gcode::prevE = gcode.E;
          }
          break;

        // G28 - Auto Home
        case 28:
          Serial.println("G28 - Auto Home");
          // It's necessary to make sure all block is cleared before return home
          if(Planner::blockQueue._length > 0) {
            while(!Planner::blockQueue._data[Planner::blockQueue._lastIndex].isDone) {
              Serial.println("Some blocks are still unhandled, wait 1s ...");
              delay(1000);
            }
          }

          {
            motorZ.enable();
            motorZ1.enable();
            motorZ.setDir(1);
            motorZ1.setDir(1);
            delay(1);
            float speed = 3;
            float distance = 3; // mm
            uint32_t interval = 1000000UL / (speed * setting.stepsPerUnit.z);
            for (uint16_t i = 0; i < distance * setting.stepsPerUnit.z; i++) {
              motorZ.moveOneStep();
              motorZ1.moveOneStep();
              delayMicroseconds(interval);
            }

            if (gcode.hasX)
            {
              motorX.enable();
              motorX.setDir(-1);
              uint32_t speed = 20; // mm/s
              uint32_t interval = 1000000UL / (speed * setting.stepsPerUnit.x);
              while (xMin.isTriggered() == false)
              {
                motorX.moveOneStep();
                delayMicroseconds(interval); // 1600steps/s
              }
              motorX.setDir(1);
              speed = 10;
              interval = 1000000UL / (speed * setting.stepsPerUnit.x);
              while (xMin.isTriggered())
              {
                motorX.moveOneStep();
                delayMicroseconds(interval); // 80 steps/s
              }
              for (uint16_t i = 0; i < 1 * setting.stepsPerUnit.x; i++) {
                motorX.moveOneStep();
                delayMicroseconds(interval);
              }
              gcode.prevX = 0;
              motorX.posInSteps = 0;
              Serial.print("motorX.posInSteps = ");
              Serial.println(motorX.posInSteps);
            }

          if(gcode.hasY)
          {
            motorY.enable();
            motorY.setDir(-1);
            speed = 20; // mm/s
            interval = 1000000UL / (speed * setting.stepsPerUnit.y);
            while (yMin.isTriggered() == false)
            {
              motorY.moveOneStep();
              delayMicroseconds(interval); // 1600steps/s
            }
            motorY.setDir(1);
            speed = 10;
            interval = 1000000UL / (speed * setting.stepsPerUnit.y);
            while (yMin.isTriggered())
            {
              motorY.moveOneStep();
              delayMicroseconds(interval); // 80 steps/s
            }
            for (uint16_t i = 0; i < 1 * setting.stepsPerUnit.y; i++) {
              motorY.moveOneStep();
              delayMicroseconds(interval);
            }
            gcode.prevY = 0;
            motorY.posInSteps = 0;
            Serial.print("motorY.posInSteps = ");
            Serial.println(motorY.posInSteps);
          }

          if(gcode.hasZ) 
          {
            motorZ.enable();
            motorZ1.enable();

            // Move down quickly
            motorZ.setDir(-1);
            motorZ1.setDir(-1);
            float speed = 3; // mm/s
            uint32_t interval = 1000000UL / (speed * setting.stepsPerUnit.z);
            while(zMin.isTriggered() == false || z1Min.isTriggered() == false) {
              if(!zMin.isTriggered()) motorZ.moveOneStep();
              if(!z1Min.isTriggered()) motorZ1.moveOneStep();
              delayMicroseconds(interval);
            }
            
            // Move up a little
            motorZ.setDir(1);
            motorZ1.setDir(1);
            for (uint32_t i = 0; i < 2 * setting.stepsPerUnit.z; i++) {
              motorZ.moveOneStep();
              motorZ1.moveOneStep();
              delayMicroseconds(interval);
            }

            // Move down slowly until endstops are triggered
            motorZ.setDir(-1);
            motorZ1.setDir(-1);
            speed = 0.8; // mm/s
            interval = 1000000UL / (speed * setting.stepsPerUnit.z);
            while(!zMin.isTriggered() || !z1Min.isTriggered()) {
              if(!zMin.isTriggered()) motorZ.moveOneStep();
              if(!z1Min.isTriggered()) motorZ1.moveOneStep();
              delayMicroseconds(interval);
            }

            // Move up slowly until endstops are not triggered
            motorZ.setDir(1);
            motorZ1.setDir(1);
            while(zMin.isTriggered() || z1Min.isTriggered()) {
              if(zMin.isTriggered()) motorZ.moveOneStep();
              if(z1Min.isTriggered()) motorZ1.moveOneStep();
              delayMicroseconds(interval);
            }
            delay(30);
            while(zMin.isTriggered() || z1Min.isTriggered()) {
              if(zMin.isTriggered()) motorZ.moveOneStep();
              if(z1Min.isTriggered()) motorZ1.moveOneStep();
              delayMicroseconds(interval);
            }

            gcode.prevZ = 0;
            motorZ.posInSteps = 0;
            motorZ1.posInSteps = 0;
            Serial.print("motorZ.posInSteps");
            Serial.println(motorZ.posInSteps);
            Serial.print("motorZ1.posInSteps");
            Serial.println(motorZ1.posInSteps);
          }
        }
        break;
      
      // G90 - Absolute positioning
      case 90:
        Serial.println("G90 - Absolute positioning");
        setting.absoluteMode = true;
        setting.eAbsoluteMode = true;
        break;
      
      // G91 - Relative positioning
      case 91:
        Serial.println("G91 - Relative positioning");
        setting.absoluteMode = false;
        setting.eAbsoluteMode = false;
        break;

      // G92 - Set Position
      case 92:
        Serial.println("G92 - Set Position");
        if(gcode.hasX) {
          prevStartPos.x = gcode.X;
          prevTargetPos.x = gcode.X;
          gcode.prevX = gcode.X;
        }
        if(gcode.hasY) {
          prevStartPos.y = gcode.Y;
          prevTargetPos.y = gcode.Y;
          gcode.prevY = gcode.Y;
        }
        if(gcode.hasZ) {
          prevStartPos.z = gcode.Z;
          prevTargetPos.z = gcode.Z;
          gcode.prevZ = gcode.Z;
        }
        if(gcode.hasE) {
          prevStartPos.e = gcode.E;
          prevTargetPos.e = gcode.E;
          gcode.prevE = gcode.E;
          motorE.posInSteps = gcode.E;
        }
        break;

      break;

      default:
        Serial.print(gcode.cmdtype);
        Serial.print(gcode.cmdnum);
        Serial.println(" still not implement yet");
        break;
      }

      Serial.write("ok\n");
    } else if (gcode.cmdtype == 'M') {
      switch (gcode.cmdnum) {
      // M20 - List SD Card
      case 20:
        Serial.println("M20 - List SD Card");
        // Print file list
        sdCard.getFileList();
        Serial.println("Send M23 I4 to print option4");
        sdCard.printFileList();
        break;

      // M23 - Select SD file
      case 23:
        Serial.println("M23 - Select SD file");
        sdCard.readFile((int)gcode.I);
        break;

      // M82 - disable motors
      case 82:
        Serial.println("M82 - disable motors");
        if(Planner::blockQueue._length > 0) {
          while(!Planner::blockQueue._data[Planner::blockQueue._lastIndex].isDone) {
          Serial.println("Some blocks are still unhandled, wait 1s ...");
          delay(1000);
          }
        }
        motorX.disable();
        motorY.disable();
        motorZ.disable();
        motorZ1.disable();
        motorE.disable();
        break;

      // M83 - E Relative
      case 83:
        setting.eAbsoluteMode = false;
        break;

      // M104 - Set Hotend Temperature
      case 104:
        hotend.setTargetTemp(gcode.S);
        Serial.println("M104 - Set Hotend Temperature");
        Serial.print("Set targetTemp = ");
        Serial.print(gcode.S);
        Serial.print("\n");
        break;

      // M105 - Report Temperatures
      case 105:
        Serial.println("M105 - Report Temperatures");
        Serial.print("Hotend temp = ");
        Serial.print(hotend.readTemp());
        Serial.print(" Hotbed temp = ");
        Serial.println(hotbed.readTemp());
        break;

      // M106 - Set Fan Speed
      case 106:
        fan.setDuty(gcode.S);
        Serial.print("M106 - Set Fan Speed as ");
        Serial.println(gcode.S);
        break;

      // M107 - Fan Off
      case 107:
        fan.setDuty(0);
        Serial.println("M107 - Fan Off");
        break;

      // M109 - Wait for Hotend Temperature
      case 109:
        Serial.println("M109 - Wait for Hotend Temperature");
        if(gcode.hasS) {
          hotend.setTargetTemp(gcode.S);
          while(hotend.readTemp() < gcode.S) {
            Serial.print("hotend temp = ");
            Serial.println(hotend.readTemp());
            delay(2000);
          }
        } else {
          Serial.println("M109 should have S parameter, skip this cmd");
        }
        break;

      // M140 - Set Bed Temperature
      case 140:
        hotbed.setTargetTemp(gcode.S);
        Serial.println("M140 - Set Hotbed Temperature");
        Serial.print("Set targetTemp = ");
        Serial.print(gcode.S);
        Serial.print("\n");
        break;

      // M190 - Wait for Bed Temperature
      case 190:
        Serial.println("M190 - Wait for Bed Temperature");
        if(gcode.hasS) {
          hotbed.setTargetTemp(gcode.S);
          while(hotbed.readTemp() < (gcode.S - 3)) {
            Serial.print("Hotbed temp = ");
            Serial.println(hotbed.readTemp());
            delay(2000);
          }
        } else {
          Serial.println("M190 should have S parameter, skip this cmd");
        }

        break;

      // M204 - Set Starting Acceleration
      case 204:
        Serial.print("M204 - Set Acceleration as ");
        Serial.println(gcode.S);
        Gcode::prevAcceleration = gcode.S;
        break;

      // M301 - Set Hotend PID
      case 301:
        hotend.kp = gcode.P;
        hotend.ki = gcode.I;
        hotend.kd = gcode.D;
        Serial.println("M301 - Set Hotend PID");
        Serial.print("kp = ");
        Serial.print(hotend.kp);
        Serial.print(" ki = ");
        Serial.print(hotend.ki);
        Serial.print(" kd = ");
        Serial.print(hotend.kd);
        Serial.print("\n");
        break;
      // M304 - Set Bed PID
      case 304:
        hotbed.kp = gcode.P;
        hotbed.ki = gcode.I;
        hotbed.kd = gcode.D;
        Serial.println("M304 - Set Bed PID");
        Serial.print("kp = ");
        Serial.print(hotbed.kp);
        Serial.print(" ki = ");
        Serial.print(hotbed.ki);
        Serial.print(" kd = ");
        Serial.print(hotbed.kd);
        Serial.print("\n");
        break;

      default:
        Serial.print(gcode.cmdtype);
        Serial.print(gcode.cmdnum);
        Serial.println(" still not implement yet");
        break;
      }

      Serial.write("ok\n");
    }

  }

}
