#include <Arduino.h>
#include "gcode/Gcode.h"
#include "module/Heater.h"
#include "module/Fan.h"
#include "config/pin.h"
#include "config/config.h"
#include "config/Setting.h"
#include "module/Stepper.h"
#include "module/Planner.h"
#include "module/Endstop.h"
#include "Utils/Queue.h"
#include "Utils/Utils.h"
#include "Utils/types.h"
#include "Utils/Array.h"
#include <SPI.h>
#include <SD.h>

bool absoluteMode = true;
Array<String> sdFileList = Array<String>();
File selectedFile;
bool sdFileSelected = false;
double_xyze_t globalPos = {};
Queue<String> gcodeStrQueue = Queue<String>(4);
block_t curBlock = {};
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

void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void printStr(String str, int i) {
  Serial.print("Option ");
  Serial.print(i);
  Serial.print(": ");
  Serial.println(str);
};

void getFileList(Array<String> *fileList) {
  File root = SD.open("/");
  while (true) {
    File item =  root.openNextFile();
    if (!item) break;
    fileList->push(item.name());
    item.close();
  }
}

void setup()
{
  setting.stepsPerUnit.x = STEPS_PER_UNIT_X;
  setting.stepsPerUnit.y = STEPS_PER_UNIT_Y;
  setting.stepsPerUnit.z = STEPS_PER_UNIT_Z;
  setting.stepsPerUnit.e = STEPS_PER_UNIT_E;
  setting.maxAcceleration.x = MAX_ACCELERATION_X;
  setting.maxAcceleration.y = MAX_ACCELERATION_Y;
  setting.maxAcceleration.z = MAX_ACCELERATION_Z;
  setting.maxAcceleration.e = MAX_ACCELERATION_E;
  setting.maxAccelerateRate.x = floor(setting.maxAcceleration.x * setting.stepsPerUnit.x);
  setting.maxAccelerateRate.y = floor(setting.maxAcceleration.y * setting.stepsPerUnit.y);
  setting.maxAccelerateRate.z = floor(setting.maxAcceleration.z * setting.stepsPerUnit.z);
  setting.maxAccelerateRate.e = floor(setting.maxAcceleration.e * setting.stepsPerUnit.e);
  
  Serial.begin(115200);
  Heater::init(150);
  Stepper::init();
  hotend.setPid(HOTEND_KP, HOTEND_KI, HOTEND_KD);
  hotbed.setPid(HOTBED_KP, HOTBED_KI, HOTBED_KD);

  curBlock.isDone = true;
  curBlock.isBusy = false;
  Gcode::prevAcceleration = 1000; // mm/s

  // SD card init
  if(SD.begin(53) == false){
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset or reopen this serial monitor after fixing your issue!");
    while (true);
  };
  Serial.println("SD card initialization done.");
}

// Motor control isr
ISR(TIMER1_COMPA_vect) {
  static uint32_t interval = 0;
  // uint32_t interval = 0;
  static uint8_t stepLoops = 1;
  if (Planner::blockQueue.first().isReady && curBlock.isDone) {
    curBlock = Planner::blockQueue.first();
    motorX.setDir(curBlock.dir.x);
    motorY.setDir(curBlock.dir.y);
    motorZ.setDir(curBlock.dir.z);
    motorZ1.setDir(curBlock.dir.z);
    motorE.setDir(curBlock.dir.e);
    curBlock.isBusy = true;
    curBlock.isDone = false;
    motorX.enable();
    motorY.enable();
    motorZ.enable();
    motorZ1.enable();
    motorE.enable();
    motorX.deltaError = -(curBlock.stepEventCount >> 1);
    motorY.deltaError = motorX.deltaError;
    motorZ.deltaError = motorX.deltaError;
    motorE.deltaError = motorX.deltaError;

    curBlock.speedRate = curBlock.entryRate;
    // interval = 300;
  }

  if(curBlock.isBusy) {
    // Which axis need advance one step during one step event
    for(uint8_t i=0; i<stepLoops; i++) {
      motorX.deltaError += curBlock.steps.x;
      if (motorX.deltaError > 0)
      {
        motorX.setStepPinHigh();
        motorX.deltaError -= curBlock.stepEventCount;
        // motorX.position += curBlock.dir.x;
        // globalPos.x += curBlock.dir.x * 1.0 / 80;
        // motorX.moveOneStep();
        motorX.setStepPinLow();
      }

      motorY.deltaError += curBlock.steps.y;
      if (motorY.deltaError > 0)
      {
        motorY.setStepPinHigh();
        motorY.deltaError -= curBlock.stepEventCount;
        // motorY.position += curBlock.dir.y;
        // globalPos.y += curBlock.dir.y * 1.0 / 80;
        // motorY.moveOneStep();
        motorY.setStepPinLow();
      }

      motorZ.deltaError += curBlock.steps.z;
      if (motorZ.deltaError > 0)
      {
        motorZ.setStepPinHigh();
        motorZ1.setStepPinHigh();
        motorZ.deltaError -= curBlock.stepEventCount;
        // motorZ.position += curBlock.dir.z;
        // motorZ1.position += curBlock.dir.z;
        // globalPos.z += curBlock.dir.z * 1.0 / 80;
        // motorZ.moveOneStep();
        // motorZ1.moveOneStep();
        motorZ.setStepPinLow();
        motorZ1.setStepPinLow();
      }

      motorE.deltaError += curBlock.steps.e;
      if (motorE.deltaError > 0)
      {
        motorE.setStepPinHigh();
        motorE.deltaError -= curBlock.stepEventCount;
        // motorY.position += curBlock.dir.y;
        // globalPos.y += curBlock.dir.y * 1.0 / 80;
        // motorE.moveOneStep();
        motorE.setStepPinLow();
      }

      //  motorX.deltaError += curBlock.steps.x;
      // if (motorX.deltaError > 0)
      // {
      //   motorX.deltaError -= curBlock.stepEventCount;
      //   motorX.needAdvance = true;
      // }

      // motorY.deltaError += curBlock.steps.y;
      // if (motorY.deltaError > 0)
      // {
      //   motorY.deltaError -= curBlock.stepEventCount;
      //   motorY.needAdvance = true;
      // }

      // motorZ.deltaError += curBlock.steps.z;
      // if (motorZ.deltaError > 0)
      // {
      //   motorZ.deltaError -= curBlock.stepEventCount;
      //   motorZ.needAdvance = true;
      //   motorZ1.needAdvance = true;
      // }

      // motorE.deltaError += curBlock.steps.e;
      // if (motorE.deltaError > 0)
      // {
      //   motorE.deltaError -= curBlock.stepEventCount;
      //   motorE.needAdvance = true;
      // }

      // if(motorX.needAdvance) motorX.setStepPinHigh();
      // if(motorY.needAdvance) motorY.setStepPinHigh();
      // if(motorZ.needAdvance) motorZ.setStepPinHigh();
      // if(motorZ1.needAdvance) motorZ1.setStepPinHigh();
      // if(motorE.needAdvance) motorE.setStepPinHigh();

      // if(motorX.needAdvance) motorX.setStepPinLow();
      // if(motorY.needAdvance) motorY.setStepPinLow();
      // if(motorZ.needAdvance) motorZ.setStepPinLow();
      // if(motorZ1.needAdvance) motorZ1.setStepPinLow();
      // if(motorE.needAdvance) motorE.setStepPinLow();

      if(xMin.isTriggered() || yMin.isTriggered() || zMin.isTriggered() || z1Min.isTriggered()){
        curBlock.stepEventCompleted = curBlock.stepEventCount;
      }

      curBlock.stepEventCompleted += 1;
    }
    
    // What phase is current step event in?
    // accelerate, cruise, decelerate, done
    if (curBlock.stepEventCompleted >= curBlock.stepEventCount) { // block done
      curBlock.isDone = true; // mark current block as completed
      curBlock.isBusy = false; // mark current block as completed
      // Serial.print(" entryRate ");
      // Serial.print(curBlock.entryRate);
      // Serial.print(" exitRate ");
      // Serial.print(curBlock.exitRate);
      // Serial.print(" speedRate ");
      // Serial.print(curBlock.speedRate);
      Planner::blockQueue.dequeue();
    } else if (curBlock.stepEventCompleted <= curBlock.accelerateUntil) { // acc phase
      curBlock.speedRate += (interval * curBlock.accelerateRate) / 1000000UL;
      if(curBlock.speedRate > curBlock.nominalRate) curBlock.speedRate = curBlock.nominalRate;
      // Serial.print("acc phase ");
    } else if (curBlock.stepEventCompleted > curBlock.decelerateAfter) { // dec phase
      curBlock.speedRate -= (interval * curBlock.accelerateRate) / 1000000UL;
      if(curBlock.speedRate < curBlock.exitRate) curBlock.speedRate = curBlock.exitRate;
      // Serial.print("dec phase ");
    } else { // cruise phase
      // Serial.print("cruise phase ");
    }

    interval = 1000000UL / curBlock.speedRate;
    // interval = 500;
    if(curBlock.speedRate > 20000) {
      stepLoops = 4;
      interval *= 4;
    } else if(curBlock.speedRate > 10000) {
      stepLoops = 2;
      interval *= 2;
    } else {
      stepLoops = 1;
    }

    // Serial.print(" ID = ");
    // Serial.print(curBlock.id);
    // Serial.print(" entryRate = ");
    // Serial.print(curBlock.entryRate);
    // // Serial.print(" accelerateRate = ");
    // // Serial.print(curBlock.accelerateRate);
    // Serial.print(" speedRate = ");
    // Serial.print(curBlock.speedRate);
    // Serial.print(" interval(us) = ");
    // Serial.print(interval);
    // Serial.print("\n");

  } else {
    // If there is no block available, set interupt interval as 1ms
    interval = 1000;
  }

  // Set Speed
  // If current speed is 1000steps/s, set interupt interval as 1000us(1/1000s)
  // OCR1A = 20 /1000 * 250000 -1; // This val should be 24999, but somehow this val will be set as 65535
  OCR1A = interval * 2 - 1; // us * 2 - 1
}

// Heater isr
ISR(TIMER3_COMPA_vect) {
  hotbed.update();
  hotend.update();
}

void loop() {
  // Check endstops
  if (xMin.isTriggered()) {
    // motorX.disable();
    Serial.println("xMin.isTriggered()");
    globalPos.x = 0;
  }
  if (yMin.isTriggered()) {
    // motorY.disable();
    Serial.println("yMin.isTriggered()");
    globalPos.y = 0;
  }
  if (zMin.isTriggered()) {
    // motorZ.disable();
    Serial.println("zMin.isTriggered()");
    globalPos.z = 0;
  }
  if (z1Min.isTriggered()) {
    // motorZ1.disable();
    Serial.println("z1Min.isTriggered()");
    globalPos.z = 0;
  }
  
  // Read Gcode from serial
  while (Serial.available() && !gcodeStrQueue.isFull()) {
    String str = Serial.readStringUntil('\n');
    Serial.print("Get a cmd from host: ");
    Serial.println(str);
    gcodeStrQueue.enqueue(str);
  }

  // Read Gcode from SD card
  while (sdFileSelected && !gcodeStrQueue.isFull()) {
    String str = selectedFile.readStringUntil('\n');
    // Serial.print("Read gcode from SD card ");
    // Serial.println(str);
    while(str.charAt(0) == ';'){
      str = selectedFile.readStringUntil('\n');
    }
    gcodeStrQueue.enqueue(str);
    if(selectedFile.position() >= selectedFile.size()) {
      sdFileSelected = false;
      selectedFile.close();
      Serial.print("Print Complete");
    }
  }

  // Parse Gcode
  while (!gcodeStrQueue.isEmpty() && !Planner::blockQueue.isFull())
  {
        // Serial.println("Parsing gcode");
        Gcode gcode = Gcode::parse(gcodeStrQueue.dequeue());
        // Serial.print("cmdNum = ");
        // Serial.print(gcode.cmdnum);
        // Serial.print(" X = ");
        // Serial.print(gcode.X);
        // Serial.print(" I = ");
        // Serial.print(gcode.I);
        // Serial.print("\n");
        // Gcode gcode = Gcode::parse(str);

        if (gcode.prevX > X_RANGE_MAX || gcode.prevY > Y_RANGE_MAX || gcode.prevZ > Z_RANGE_MAX
         || gcode.prevX < X_RANGE_MIN || gcode.prevY < Y_RANGE_MIN || gcode.prevZ < Z_RANGE_MIN) {
          Serial.print("Cmd outrange!! Disable all motors");
          motorX.disable();
          motorY.disable();
          motorZ.disable();
          motorZ1.disable();
          break;
        }

        if (gcode.cmdtype == 'G')
          {
            switch (gcode.cmdnum)
            {
            // G01 - linear motion
            case 0:
            case 1:
              {
                Serial.println("G01 - linear motion");
                if (gcode.hasX) {
                  Serial.print("x = ");
                  Serial.print(gcode.X);
                }
                if (gcode.hasY) {
                  Serial.print(" y = ");
                  Serial.print(gcode.Y);
                }
                if (gcode.hasZ) {
                  Serial.print(" z = ");
                  Serial.print(gcode.Z);
                }
                if (gcode.hasE) {
                  Serial.print(" e = ");
                  Serial.print(gcode.E);
                }
                Serial.print("\n");

                Planner::planBufferLine(&gcode, &setting, absoluteMode);

                if (gcode.hasF) Gcode::prevFeedSpeed = gcode.F;
                if(absoluteMode) {
                  if (gcode.hasX) Gcode::prevX = gcode.X;
                  if (gcode.hasY) Gcode::prevY = gcode.Y;
                  if (gcode.hasZ) Gcode::prevZ = gcode.Z;
                  if (gcode.hasE) Gcode::prevE = gcode.E;
                } else {
                  if (gcode.hasX) gcode.prevX += gcode.X;
                  if (gcode.hasY) gcode.prevY += gcode.Y;
                  if (gcode.hasZ) gcode.prevZ += gcode.Z;
                  if (gcode.hasE) gcode.prevE += gcode.E;
                }
              }
              break;

            // G28 - Auto Home
            case 28:
              Serial.println("G28 - Auto Home");

              // If you want define variable in case, limit its scope
              // otherwise case below won't work
              {
                motorZ.enable();
                motorZ1.enable();
                motorZ.setDir(1);
                motorZ1.setDir(1);
                float speed = 3;
                uint32_t interval = 1000000UL / (speed * setting.stepsPerUnit.z);
                for (uint16_t i = 0; i < 5 * setting.stepsPerUnit.z; i++) {
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
                  // globalPos.x = 0;
                  gcode.prevX = 0;
                  Serial.print("gcode.prevX = ");
                  Serial.println(gcode.prevX);
                }

                if (gcode.hasY)
                {
                  motorY.enable();
                  motorY.setDir(-1);
                  uint32_t speed = 20; // mm/s
                  uint32_t interval = 1000000UL / (speed * setting.stepsPerUnit.y);
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
                  // globalPos.y = 0;
                  gcode.prevY = 0;
                  Serial.print("gcode.prevY = ");
                  Serial.println(gcode.prevY);
                }

              if(gcode.hasZ) {
                motorZ.enable();
                motorZ1.enable();
                motorZ.setDir(-1);
                motorZ1.setDir(-1);
                float speed = 2; // mm/s
                uint32_t interval = 1000000UL / (speed * setting.stepsPerUnit.z);
                while(zMin.isTriggered() == false || z1Min.isTriggered() == false) {
                  if(!zMin.isTriggered()) motorZ.moveOneStep();
                  if(!z1Min.isTriggered()) motorZ1.moveOneStep();
                  delayMicroseconds(interval);
                }
                motorZ.setDir(1);
                motorZ1.setDir(1);
                speed = 0.8; // mm/s
                interval = 1000000UL / (speed * setting.stepsPerUnit.z);
                while(zMin.isTriggered() || z1Min.isTriggered()) {
                  if(zMin.isTriggered()) motorZ.moveOneStep();
                  if(z1Min.isTriggered()) motorZ1.moveOneStep();
                  delayMicroseconds(interval);
                }
                delay(15);
                while(zMin.isTriggered() || z1Min.isTriggered()) {
                  if(zMin.isTriggered()) motorZ.moveOneStep();
                  if(z1Min.isTriggered()) motorZ1.moveOneStep();
                  delayMicroseconds(interval);
                }
                // globalPos.z = 0;
                gcode.prevZ = 0;
                Serial.print("gcode.prevZ = ");
                Serial.println(gcode.prevZ);
              }
            }
            break;
          
          // G90 - Absolute positioning
          case 90:
            Serial.println("G90 - Absolute positioning");
            absoluteMode = true;
            break;
          
          // G91 - Relative positioning
          case 91:
            Serial.println("G91 - Relative positioning");
            absoluteMode = false;
            break;

          // G92 - Set Position
          case 92:
            Serial.println("G92 - Set Position");
            if(gcode.hasX) gcode.prevX = gcode.X;
            if(gcode.hasY) gcode.prevY = gcode.Y;
            if(gcode.hasZ) gcode.prevZ = gcode.Z;
            if(gcode.hasE) gcode.prevE = gcode.E;
            break;

          default:
            break;
          }
        } else if (gcode.cmdtype == 'M') {
          // Serial.println("gcode type = M ");
          switch (gcode.cmdnum)
          {
          // Somehow if case 20 at top, other cases like M107 will not work.
          // this issue is handled after encapsulate the while func into getFileList().
          // M20 - List SD Card
          case 20:
            Serial.println("M20 - List SD Card");
            // Print file list
            getFileList(&sdFileList);
            Serial.println("Send M23 I4 to choose option 4");
            sdFileList.forEach(printStr);
            break;

          // M23 - Select SD file
          case 23:
            Serial.println("M23 - Select SD file");
            // int cmdNum = (int)gcode.I;
            // selectedFileName = sdFileList[cmdNum];
            selectedFile = SD.open(sdFileList[(int)gcode.I]);
            sdFileSelected = true;
            break;

          // M82 - disable motors
          case 82:
            Serial.println("M82 - disable motors");
            motorX.disable();
            motorY.disable();
            motorZ.disable();
            motorZ1.disable();
            motorE.disable();
            break;

          // M104 - Set Hotend Temperature
          case 104:
            // Gcode::M104(&gcode);
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
            break;
          }
        }

      Serial.write("ok\n");
  }

}
