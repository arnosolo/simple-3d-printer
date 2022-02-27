#include "Planner.h"

Queue<block_t> Planner::blockQueue = Queue<block_t>(BLOCK_BUFFER_SIZE);

Planner::Planner() {
}

void Planner::planBufferLine(Gcode* gcode, Setting* setting, bool absoluteMode) {
  static uint32_t id = 0;
  block_t block = {}; // Initilize all num as 0, otherwise entryRate would be extremely large
  block.id = id;

  // Set flag
  block.isReady = false;
  block.isBusy = false; // since it's not executed during motor isr
  block.isDone = false;

  // Calculate displacement
  if(absoluteMode) {
    block.targetPos.x = gcode->hasX ? gcode->X : gcode->prevX;
    block.targetPos.y = gcode->hasY ? gcode->Y : gcode->prevY;
    block.targetPos.z = gcode->hasZ ? gcode->Z : gcode->prevZ;
    block.targetPos.e = gcode->hasE ? gcode->E : gcode->prevE;
  } else {
    block.targetPos.x = gcode->hasX ? (gcode->prevX + gcode->X) : gcode->prevX;
    block.targetPos.y = gcode->hasY ? (gcode->prevY + gcode->Y) : gcode->prevY;
    block.targetPos.z = gcode->hasZ ? (gcode->prevZ + gcode->Z) : gcode->prevZ;
    block.targetPos.e = gcode->hasE ? (gcode->prevE + gcode->E) : gcode->prevE;
  }

  block.startPos.x = gcode->prevX;
  block.startPos.y = gcode->prevY;
  block.startPos.z = gcode->prevZ;
  block.startPos.e = gcode->prevE;  

  block.deltaPos.x = block.targetPos.x - block.startPos.x;
  block.deltaPos.y = block.targetPos.y - block.startPos.y;
  block.deltaPos.z = block.targetPos.z - block.startPos.z;
  block.deltaPos.e = block.targetPos.e - block.startPos.e;
  
  block.steps.x = fabs(block.deltaPos.x) * setting->stepsPerUnit.x;
  block.steps.y = fabs(block.deltaPos.y) * setting->stepsPerUnit.y;
  block.steps.z = fabs(block.deltaPos.z) * setting->stepsPerUnit.z;
  block.steps.e = fabs(block.deltaPos.e) * setting->stepsPerUnit.e;

  block.stepEventCount = getMax(block.steps);
  // if total steps need be advanced is too few(5 steps), drop this block
  if(block.stepEventCount < MIN_EVENT_COUNT) return;
  block.stepEventCompleted = 0;

  // Compute distance between start and target position
  if(block.deltaPos.x == 0 && block.deltaPos.y == 0 && block.deltaPos.z == 0) {
    block.distance = block.deltaPos.e;
    Serial.println("block.distance = block.deltaPos.e");
  } else {
    block.distance = sqrt(sq(block.deltaPos.x) + sq(block.deltaPos.y) + sq(block.deltaPos.z));
  }
  double stepsPerMm = block.stepEventCount * 1.0 / block.distance; // steps/mm
  block.stepsPerMm = stepsPerMm;

  // Set nominal speed
  double feedSpeed = gcode->hasF ? (gcode->F / 60) : (gcode->prevFeedSpeed / 60); // mm/min to mm/s
  block.nominalSpeed = feedSpeed; // mm/s
  block.nominalRate = ceil(feedSpeed * stepsPerMm); // steps/s

  // Set acceleration
  block.acceleration = gcode->prevAcceleration; // mm/s^2
  block.unitVector.x = block.deltaPos.x / block.distance;
  block.unitVector.y = block.deltaPos.y / block.distance;
  block.unitVector.z = block.deltaPos.z / block.distance;
  Serial.print("block.acceleration = ");
  Serial.println(block.acceleration);
  // limit acceleration according to every axis
  if(block.acceleration * fabs(block.unitVector.x) > setting->maxAcceleration.x) {
    block.acceleration = setting->maxAcceleration.x / fabs(block.unitVector.x);
  }
  if(block.acceleration * fabs(block.unitVector.y) > setting->maxAcceleration.y) {
    block.acceleration = setting->maxAcceleration.y / fabs(block.unitVector.y);
  }
  if(block.acceleration * fabs(block.unitVector.z) > setting->maxAcceleration.z) {
    block.acceleration = setting->maxAcceleration.z / fabs(block.unitVector.z);
  }
  if(block.acceleration > setting->maxAcceleration.e) {
    block.acceleration = setting->maxAcceleration.e;
  }

  block.accelerateRate = floor(block.acceleration * stepsPerMm); // steps/s^2
  if(((double)block.accelerateRate * (double)block.steps.x / (double)block.stepEventCount) > setting->maxAccelerateRate.x) {
    block.accelerateRate = setting->maxAccelerateRate.x;
  }
  if(((double)block.accelerateRate * (double)block.steps.y / (double)block.stepEventCount) > setting->maxAccelerateRate.y) {
    block.accelerateRate = setting->maxAccelerateRate.y;
  }
  if(((double)block.accelerateRate * (double)block.steps.z / (double)block.stepEventCount) > setting->maxAccelerateRate.z) {
    block.accelerateRate = setting->maxAccelerateRate.z;
  }
  if(((double)block.accelerateRate * (double)block.steps.e / (double)block.stepEventCount) > setting->maxAccelerateRate.e) {
    block.accelerateRate = setting->maxAccelerateRate.e;
  }

  // Set direction
  block.dir.x = block.deltaPos.x < 0 ? -1 : 1;
  block.dir.y = block.deltaPos.y < 0 ? -1 : 1;
  block.dir.z = block.deltaPos.z < 0 ? -1 : 1;
  block.dir.e = block.deltaPos.e < 0 ? -1 : 1;

  // Compute max junction speed
  if(blockQueue.length() > 0) {
    block_t* prevBlock = &blockQueue._data[blockQueue._lastIndex];
    double junctionDeviation = 0.02; // mm

    // Note that a -1 is multiplied here. Since The angle between the 
    // displacement vectors is not the angle between the two line segments
    double cosTheta = 
      -(block.deltaPos.x * prevBlock->deltaPos.x
      + block.deltaPos.y * prevBlock->deltaPos.y
      + block.deltaPos.z * prevBlock->deltaPos.z)
      / (prevBlock->distance * block.distance);
    // theta 0° ~ 18°
    double maxJunctionSpeed = MINIMUM_PLANNER_SPEED;
    if(cosTheta < 0.95) { // theta > 18°
    // theta 162° ~ 180°
      maxJunctionSpeed = min(prevBlock->nominalSpeed, block.nominalSpeed);
      if(cosTheta > -0.95) {// theta < 162°
      // theta 18° ~ 162°
      double sinHalfTheta = sqrt(0.5 * (1 - cosTheta));
      maxJunctionSpeed = min(maxJunctionSpeed,
        sqrt(block.acceleration * junctionDeviation * sinHalfTheta / (1.0 - sinHalfTheta)));
      }
    }

    // To ensure the exit speed is bigger than minimum planner speed
    // what is the max allowable entry speed?
    double maxAllowSpeed = getMaxAllowSpeed(
      MINIMUM_PLANNER_SPEED, block.acceleration, block.distance);
    
    block.entrySpeed = min(maxJunctionSpeed, maxAllowSpeed);
    block.needRecalculate = true;
    
    prevBlock->exitSpeed = block.entrySpeed;
    prevBlock->needRecalculate = true;
  }

  // Calculate trapezoid
  calculateTrapezoid(&block);

  blockQueue.enqueue(block);
  
  // Make sure current block can decelerate to the entry speed of next block
  reverseCheck();

  // Make sure the entry speed of current block can accelerate from previous block
  forwardCheck();

  // Since the entry speed of some blocks might changed
  recalculateTrapezoids();

  blockQueue._data[blockQueue._lastIndex].isReady = true;
  Serial.print("blockQueue.length = ");
  Serial.println(blockQueue.length());
  int queueIndex = blockQueue._firstIndex;
  while(queueIndex != blockQueue._lastIndex) {
    block_t* curBlock = &blockQueue._data[queueIndex];
    Serial.print("block ");
    Serial.print(curBlock->id);
    Serial.print(" ");
    Serial.print(curBlock->entryRate);
    Serial.print(" -> ");
    Serial.print(curBlock->exitRate);
    Serial.print(" ");
    Serial.print(curBlock->entrySpeed);
    Serial.print(" -> ");
    Serial.print(curBlock->exitSpeed);
    Serial.print("\n");
    queueIndex = (queueIndex + 1) % blockQueue._capacity;
  }
  
  id++;
}

/**
  * @brief  Get accelerateUntil and decelerateAfter from entryRate nominalRate exitRate accelerateRate stepEventCount, 
  *         should be called every time entryRate or exitRate changed.
  * @param  block
  * @return 
  */
void Planner::calculateTrapezoid(block_t* block) {
  double entryRate = block->entrySpeed * block->stepsPerMm;
  double exitRate = block->exitSpeed * block->stepsPerMm;
  
  // Set min step rate, otherwise the motor 
  // isr timer will overflow
  if(entryRate < 120) entryRate = 120;
  if(exitRate < 120) exitRate = 120;
  double accRate = block->accelerateRate;
  double accelerateSteps = ceil(calculateAccelerateDistance(
    entryRate, block->nominalRate, accRate
  ));
  
  double decelerateSteps = floor(calculateAccelerateDistance(
    block->nominalRate, exitRate ,-accRate
  ));
  double plateauSteps = block->stepEventCount - accelerateSteps - decelerateSteps;

  // If not able to reach max speed, then there is only two phase.
  if(plateauSteps < 0) {
    accelerateSteps = ceil(intersectionDistance(entryRate, exitRate, accRate, block->stepEventCount));
    plateauSteps = 0;
  }

  cli(); // stop interrupts
  if(block->isBusy == false) {
    block->accelerateUntil = (uint32_t)accelerateSteps;
    block->decelerateAfter = (uint32_t)(accelerateSteps + plateauSteps);
    block->entryRate = entryRate;
    block->exitRate = exitRate;
  }
  sei(); // allow interrupts

  // Serial.print("block.id = ");
  // Serial.print(block->id);
  // Serial.print(" stepEventCount = ");
  // Serial.print(block->stepEventCount);
  // Serial.print(" entryRate = ");
  // Serial.print(block->entryRate);
  // Serial.print(" exitRate = ");
  // Serial.print(block->exitRate);
  // Serial.print(" accUntil = ");
  // Serial.print(block->accelerateUntil);
  // Serial.print(" decAfter = ");
  // Serial.println(block->decelerateAfter);

}

double Planner::calculateAccelerateDistance(double startRate, double targetRate, double accRate) {
  if(accRate == 0) {
    return 0.0;
  } else {
    return (targetRate * targetRate - startRate * startRate) / (2.0 * accRate);
  }
}

double Planner::intersectionDistance(double entryRate, double exitRate, double accRate, double totalSteps) {
  if(accRate == 0) {
    return 0.0;
  } else {
    return (sq(exitRate) - sq(entryRate) + 2.0 * accRate * totalSteps) / (4.0 * accRate);
  }
}

uint32_t Planner::getMax(uint32_xyze_t data) {
  uint32_t max = data.x;
  if(data.y > max) max = data.y;
  if(data.z > max) max = data.z;
  if(data.e > max) max = data.e;
  return max;
}

double Planner::getMaxAllowSpeed(double startSpeed, double acceleration, double distance) {
  return sqrt(sq(startSpeed) + 2.0 * acceleration * distance);
}

void Planner::reverseCheck() {
  if(blockQueue.length() >= 3){
    int prevBlockIndex = blockQueue._lastIndex - 2;
    if(prevBlockIndex < 0) prevBlockIndex += blockQueue._capacity;
    for(int i=0; i < blockQueue.length() - 2; i++) {
    // while(prevBlockIndex != blockQueue._firstIndex) {
      int curBlockIndex = (prevBlockIndex + 1) % blockQueue._capacity;
      if(curBlockIndex < 0) curBlockIndex += blockQueue._capacity;
      int nextBlockIndex = (prevBlockIndex + 2) % blockQueue._capacity;
      block_t* prevBlock = &blockQueue._data[prevBlockIndex];
      block_t* curBlock = &blockQueue._data[curBlockIndex];
      block_t* nextBlock = &blockQueue._data[nextBlockIndex];
      
      if(curBlock->entrySpeed > nextBlock->entrySpeed) {
        double entrySpeed = min(curBlock->entrySpeed,
          getMaxAllowSpeed(nextBlock->entrySpeed, curBlock->acceleration, curBlock->distance));
        if(curBlock->entrySpeed != entrySpeed) {
          curBlock->entrySpeed = entrySpeed;
          // curBlock->exitSpeed = nextBlock->entrySpeed;
          curBlock->needRecalculate = true;
        }
      }

      prevBlockIndex = prevBlockIndex - 1;
      if(prevBlockIndex < 0) prevBlockIndex += blockQueue._capacity;
    }
  }
}

void Planner::forwardCheck() {
  // Serial.println("forwardCheck");
  if(blockQueue.length() >= 3) {
    int prevBlockIndex = blockQueue._firstIndex;
    for(int i=0; i < blockQueue.length() - 2; i++) {
    // while((prevBlockIndex+2) != blockQueue._lastIndex) {
      int curBlockIndex = (prevBlockIndex + 1) % blockQueue._capacity;
      int nextBlockIndex = (prevBlockIndex + 2) % blockQueue._capacity;
      block_t* prevBlock = &blockQueue._data[prevBlockIndex];
      block_t* curBlock = &blockQueue._data[curBlockIndex];
      block_t* nextBlock = &blockQueue._data[nextBlockIndex];
      
      if(prevBlock->entrySpeed < curBlock->entrySpeed) {
        double entrySpeed = min(curBlock->entrySpeed,
          getMaxAllowSpeed(prevBlock->entrySpeed, prevBlock->acceleration, prevBlock->distance));
        if(curBlock->entrySpeed != entrySpeed) {
          curBlock->entrySpeed = entrySpeed;
          curBlock->needRecalculate = true;
          // prevBlock->exitSpeed = curBlock->entrySpeed;
          // prevBlock->needRecalculate = true;
        }
      }

      prevBlockIndex = (prevBlockIndex + 1) % blockQueue._capacity;
    }
  }
  
}

void Planner::recalculateTrapezoids() {
  int curBlockIndex = blockQueue._firstIndex;
  for(int i=0; i < blockQueue.length(); i++) {
    block_t* curBlock = &blockQueue._data[curBlockIndex];
    if(curBlock->needRecalculate) {
      calculateTrapezoid(curBlock);
      curBlock->needRecalculate = false;
      // Serial.print(curBlock.);
      // Serial.print("\n");
    }
    curBlockIndex = (curBlockIndex + 1) % blockQueue._capacity;
  }
}

// double_xyze_t Planner::normalize(double_xyze_t vector) {
  // double magnitude = sqrt(sq(vector.x) + sq(vector.y) + sq(vector.z));
// }