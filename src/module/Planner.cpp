#include "Planner.h"

Queue<block_t> Planner::blockQueue = Queue<block_t>(BLOCK_BUFFER_SIZE);

Planner::Planner() {
}

void Planner::planBufferLine(double_xyze_t startPos, double_xyze_t targetPos, double nominalSpeed, double acceleration, Setting* setting) {
  static uint32_t id = 0;
  static double_xyze_t prevUnitVector = {};
  double_xyze_t unitVector = {};
  double_xyze_t accPerAxis = {};
  block_t block = {}; // Initilize all num as 0, otherwise entryRate would be extremely large
  block.id = id;

  // 1.Set flag
  block.isReady = false;
  block.isBusy = false; // since it's not executed during motor isr
  block.isDone = false;

  // 2. Calculate displacement
  block.startPos = startPos;
  block.targetPos = targetPos;
  // int32_xyze_t startStep = {};
  int32_xyze_t targetStep = {};
  int32_xyze_t deltaStep = {};
  block.startStep.x = startPos.x * STEPS_PER_UNIT_X;
  block.startStep.y = startPos.y * STEPS_PER_UNIT_Y;
  block.startStep.z = startPos.z * STEPS_PER_UNIT_Z;
  block.startStep.e = startPos.e * STEPS_PER_UNIT_E;
  targetStep.x = targetPos.x * STEPS_PER_UNIT_X;
  targetStep.y = targetPos.y * STEPS_PER_UNIT_Y;
  targetStep.z = targetPos.z * STEPS_PER_UNIT_Z;
  targetStep.e = targetPos.e * STEPS_PER_UNIT_E;
  deltaStep.x = targetStep.x - block.startStep.x;
  deltaStep.y = targetStep.y - block.startStep.y;
  deltaStep.z = targetStep.z - block.startStep.z;
  deltaStep.e = targetStep.e - block.startStep.e;

  block.steps.x = abs(deltaStep.x);
  block.steps.y = abs(deltaStep.y);
  block.steps.z = abs(deltaStep.z);
  block.steps.e = abs(deltaStep.e);

  block.deltaPos.x = (double)deltaStep.x / STEPS_PER_UNIT_X;
  block.deltaPos.y = (double)deltaStep.y / STEPS_PER_UNIT_Y;
  block.deltaPos.z = (double)deltaStep.z / STEPS_PER_UNIT_Z;
  block.deltaPos.e = (double)deltaStep.e / STEPS_PER_UNIT_E;

  block.dir.x = block.deltaPos.x > 0 ? 1 : -1;
  block.dir.y = block.deltaPos.y > 0 ? 1 : -1;
  block.dir.z = block.deltaPos.z > 0 ? 1 : -1;
  block.dir.e = block.deltaPos.e > 0 ? 1 : -1;
  
  if(block.steps.x < MIN_STEPS_PER_SEGMENT && block.steps.y < MIN_STEPS_PER_SEGMENT && block.steps.z < MIN_STEPS_PER_SEGMENT) {
    block.distance = fabs(block.deltaPos.e);
  } else {
    block.distance = sqrt(sq(block.deltaPos.x) + sq(block.deltaPos.y) + sq(block.deltaPos.z));
  }
  if(block.steps.x == 0 && block.steps.y == 0 && (block.steps.z < block.steps.e)) {
    Serial.println("May be this will solve G1 Z0.5 E5 speedRate too high problem?");
    block.distance = fabs(block.deltaPos.e);
  }
  block.stepEventCompleted = 0;
  block.stepEventCount = getMax(block.steps);
  // if(block.stepEventCount < MIN_STEPS_PER_SEGMENT) return;

  // Decide speed
  block.entrySpeed = MINIMUM_PLANNER_SPEED;
  block.exitSpeed = MINIMUM_PLANNER_SPEED;
  block.nominalSpeed = nominalSpeed;
  double inverseDistance = 1.0 / block.distance;
  double stepsPerMm = block.stepEventCount * inverseDistance;
  block.stepsPerMm = stepsPerMm;

  unitVector.x = block.deltaPos.x * inverseDistance;
  unitVector.y = block.deltaPos.y * inverseDistance;
  unitVector.z = block.deltaPos.z * inverseDistance;

  double_xyze_t nominalSpeedPerAxis = {};
  nominalSpeedPerAxis.x = fabs(block.nominalSpeed * unitVector.x);
  nominalSpeedPerAxis.y = fabs(block.nominalSpeed * unitVector.y);
  nominalSpeedPerAxis.z = fabs(block.nominalSpeed * unitVector.z);
  if(nominalSpeedPerAxis.x > MAX_FEEDRATE_X) {
    block.nominalSpeed = MAX_FEEDRATE_X/fabs(unitVector.x);
    Serial.print("x speed too large, reset nominalSpeed as ");
    Serial.println(block.nominalSpeed);
  }
  if(nominalSpeedPerAxis.y > MAX_FEEDRATE_Y) {
    block.nominalSpeed = MAX_FEEDRATE_Y/fabs(unitVector.y);
    Serial.print("y speed too large, reset nominalSpeed as ");
    Serial.println(block.nominalSpeed);
  }
  if(nominalSpeedPerAxis.z > MAX_FEEDRATE_Z) {
    block.nominalSpeed = MAX_FEEDRATE_Z/fabs(unitVector.z);
    Serial.print("z speed too large, reset nominalSpeed as ");
    Serial.println(block.nominalSpeed);
  }
  block.nominalRate = block.nominalSpeed * stepsPerMm;
  nominalSpeedPerAxis.e = block.nominalRate / STEPS_PER_UNIT_E;
  if(nominalSpeedPerAxis.e > MAX_FEEDRATE_E) {
    block.nominalRate = MAX_FEEDRATE_E * STEPS_PER_UNIT_E;
    block.nominalSpeed = block.nominalRate / stepsPerMm;
  }

  // Decide acceleration
  block.acceleration = acceleration;
  accPerAxis.x = fabs(block.acceleration * unitVector.x);
  accPerAxis.y = fabs(block.acceleration * unitVector.y);
  accPerAxis.z = fabs(block.acceleration * unitVector.z);
  if(accPerAxis.x > setting->maxAcceleration.x) {
    Serial.print("accX too large, reset acc ");
    Serial.print(block.acceleration);
    block.acceleration = setting->maxAcceleration.x / fabs(unitVector.x);
    Serial.print(" -> ");
    Serial.println(block.acceleration);
  }
  if(accPerAxis.y > setting->maxAcceleration.y) {
    Serial.print("accY too large, reset acc ");
    Serial.print(block.acceleration);
    block.acceleration = setting->maxAcceleration.y / fabs(unitVector.y);
    Serial.print(" -> ");
    Serial.println(block.acceleration);
  }
  if(accPerAxis.z > setting->maxAcceleration.z) {
    Serial.print("accZ too large, reset acc ");
    Serial.print(block.acceleration);
    block.acceleration = setting->maxAcceleration.z / fabs(unitVector.z);
    Serial.print(" -> ");
    Serial.println(block.acceleration);
  }
  block.accelerateRate = block.acceleration * stepsPerMm;

  // Calculate junction speed
  if(blockQueue.length() > 0) {
    block_t* prevBlock = &blockQueue._data[blockQueue._lastIndex];
    double junctionDeviation = 0.1; // mm

    double cosTheta = -unitVector.x * prevUnitVector.x
      - unitVector.y * prevUnitVector.y
      - unitVector.z * prevUnitVector.z;
    
    double maxJunctionSpeed = MINIMUM_PLANNER_SPEED;
    if(cosTheta < 0.95) { // theta > 18°
    // theta 162° ~ 180°
      maxJunctionSpeed = min(prevBlock->nominalSpeed, block.nominalSpeed);
      if(cosTheta > -0.94) {// theta < 160°
      // theta 18° ~ 160°
      double sinHalfTheta = sqrt(0.5 * (1 - cosTheta));
      maxJunctionSpeed = min(maxJunctionSpeed,
        sqrt(block.acceleration * junctionDeviation * sinHalfTheta / (1.0 - sinHalfTheta)));
      }
    }
    
    double maxAllowSpeed = getMaxAllowSpeed(
      MINIMUM_PLANNER_SPEED, block.acceleration, block.distance);
    
    block.entrySpeed = min(maxJunctionSpeed, maxAllowSpeed);
    block.exitSpeed = MINIMUM_PLANNER_SPEED;
    block.needRecalculate = true;
    
    prevBlock->exitSpeed = block.entrySpeed;
    prevBlock->needRecalculate = true;
  }

  Serial.print("Enqueue block");
  Serial.print(block.id);
  Serial.print(" queLen ");
  Serial.println(blockQueue._length);
  block.needRecalculate = true;
  blockQueue.enqueue(block);

  // reverseCheck();
  forwardCheck();
  recalculateTrapezoids();

  blockQueue._data[blockQueue._lastIndex].isReady = true;

  prevUnitVector = unitVector;
  id++;
}

/**
  * @brief  Get accelerateUntil and decelerateAfter from entryRate nominalRate exitRate accelerateRate stepEventCount, 
  *         should be called every time entryRate or exitRate changed.
  * @param  block
  * @return 
  */
void Planner::calculateTrapezoid(block_t* block) {
  double accDistance = calculateAccelerateDistance(block->entrySpeed, block->nominalSpeed, block->acceleration);
  double decDistance = calculateAccelerateDistance(block->exitSpeed, block->nominalSpeed, block->acceleration);
  double plateauDistance = block->distance - fabs(accDistance) - fabs(decDistance);
  
  double nominalSpeed = block->nominalSpeed;

  if(plateauDistance < 0.0) {
    accDistance = intersectionDistance(block->entrySpeed, block->exitSpeed, block->acceleration, block->distance);
    nominalSpeed = sqrt(sq(block->entrySpeed) + 2UL * block->acceleration * accDistance);
    plateauDistance = 0.0;
  }

  uint32_t entryRate = ceil(block->entrySpeed * block->stepsPerMm);
  uint32_t exitRate = ceil(block->exitSpeed * block->stepsPerMm);
  uint32_t nominalRate = ceil(block->nominalSpeed * block->stepsPerMm);
  uint32_t accelerateUntil = ceil(accDistance * block->stepsPerMm);
  uint32_t decelerateAfter = ceil((accDistance + plateauDistance) * block->stepsPerMm);
  if(entryRate < 120) entryRate = 120;
  if(exitRate < 120) exitRate = 120;
  
  cli(); // stop interrupts
  if(block->isBusy == false) {
    block->accelerateUntil = accelerateUntil;
    block->decelerateAfter = decelerateAfter;
    block->entryRate = entryRate;
    block->exitRate = exitRate;
    block->nominalRate = nominalRate;
    block->nominalSpeed = nominalSpeed;
  }
  sei(); // allow interrupts

  Serial.print("Plan block");
  Serial.print(block->id);
  Serial.print(" deltaPos ");
  Serial.print(block->deltaPos.x);
  Serial.print(", ");
  Serial.print(block->deltaPos.y);
  Serial.print(", ");
  Serial.print(block->deltaPos.z);
  Serial.print(", ");
  Serial.print(block->deltaPos.e);
  // Serial.print(" steps ");
  // Serial.print(block->steps.x);
  // Serial.print(", ");
  // Serial.print(block->steps.y);
  // Serial.print(", ");
  // Serial.print(block->steps.z);
  // Serial.print(", ");
  // Serial.print(block->steps.e);
  // Serial.print(" distance=");
  // Serial.print(block->distance);
  // Serial.print("\n");
  // Serial.print(" acc 0 ->");
  // Serial.print(block->accelerateUntil);
  // Serial.print(" -> ");
  // Serial.print(block->decelerateAfter);
  // Serial.print(" -> ");
  // Serial.print(block->stepEventCount);
  // Serial.print(" Rate ");
  // Serial.print(block->entryRate);
  // Serial.print(" -> ");
  // Serial.print(block->nominalRate);
  // Serial.print(" -> ");
  // Serial.print(block->exitRate);
  Serial.print(" Speed ");
  Serial.print(block->entrySpeed);
  Serial.print(" -> ");
  Serial.print(block->nominalSpeed);
  Serial.print(" -> ");
  Serial.print(block->exitSpeed);
  // Serial.print(" stepsPerMm=");
  // Serial.print(block->stepsPerMm);
  Serial.print("\n");
}

double Planner::calculateAccelerateDistance(double startRate, double targetRate, double accRate) {
  if(accRate == 0) {
    return 0.0;
  } else {
    return (targetRate * targetRate - startRate * startRate) / (2.0 * accRate);
  }
}

uint32_t Planner::calculateAccSteps(uint32_t startRate, uint32_t targetRate, uint32_t accRate) {
  if(accRate == 0) {
    return 0;
  } else {
    return (targetRate * targetRate - startRate * startRate) / (accRate * 2UL);
  }
}

double Planner::intersectionDistance(double entryRate, double exitRate, double accRate, double totalSteps) {
  if(accRate == 0) {
    return 0.0;
  } else {
    return (sq(exitRate) - sq(entryRate) + 2.0 * accRate * totalSteps) / (4.0 * accRate);
  }
}

uint32_t Planner::intersectionSteps(uint32_t entryRate, uint32_t exitRate, uint32_t accRate, uint32_t totalSteps) {
  if(accRate == 0) {
    return 0;
  } else {
    return (exitRate * exitRate - entryRate * entryRate + accRate * totalSteps * 2UL) / (accRate * 4UL);
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
  double speedSq = sq(startSpeed) + 2.0 * acceleration * distance;
  if(speedSq < 0) speedSq = 0;
  return sqrt(speedSq);
}

uint32_t Planner::getMaxAllowRate(uint32_t startRate, uint32_t accelerateRate, uint32_t steps) {
  return sqrt(sq(startRate) + steps * accelerateRate << 1);
}

void Planner::reverseCheck() {
  if(blockQueue.length() >= 3){
    int prevBlockIndex = blockQueue._lastIndex - 2;
    if(prevBlockIndex < 0) prevBlockIndex += blockQueue._capacity;
    for(int i=0; i < blockQueue.length() - 2; i++) {
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
          curBlock->needRecalculate = true;
          prevBlock->exitSpeed = curBlock->entrySpeed;
          prevBlock->needRecalculate = true;
        }
      }

      prevBlockIndex = prevBlockIndex - 1;
      if(prevBlockIndex < 0) prevBlockIndex += blockQueue._capacity;
    }
  }
}

// void Planner::forwardCheck() {
//   if(blockQueue.length() >= 3) {
//     int prevBlockIndex = blockQueue._firstIndex;
//     for(int i=0; i < blockQueue.length() - 2; i++) {
//       int curBlockIndex = (prevBlockIndex + 1) % blockQueue._capacity;
//       int nextBlockIndex = (prevBlockIndex + 2) % blockQueue._capacity;
//       block_t* prevBlock = &blockQueue._data[prevBlockIndex];
//       block_t* curBlock = &blockQueue._data[curBlockIndex];
//       block_t* nextBlock = &blockQueue._data[nextBlockIndex];
      
//       if(prevBlock->entrySpeed < curBlock->entrySpeed) {
//         double entrySpeed = min(curBlock->entrySpeed,
//           getMaxAllowSpeed(prevBlock->entrySpeed, prevBlock->acceleration, prevBlock->distance));
//         if(curBlock->entrySpeed != entrySpeed) {
//           curBlock->entrySpeed = entrySpeed;
//           curBlock->needRecalculate = true;
//           prevBlock->exitSpeed = curBlock->entrySpeed;
//           prevBlock->needRecalculate = true;
//         }
//       }

//       prevBlockIndex = (prevBlockIndex + 1) % blockQueue._capacity;
//     }
//   }
// }

void Planner::forwardCheck() {
  if(blockQueue._length > 1) {
    int curBlockIdx = blockQueue._firstIndex;
    int nextBlockIdx = (blockQueue._firstIndex + 1) % blockQueue._capacity;
    block_t* curBlock = &blockQueue._data[curBlockIdx];
    block_t* nextBlock = &blockQueue._data[nextBlockIdx];

    while(curBlockIdx != blockQueue._lastIndex) {
      if(curBlock->entrySpeed < curBlock->exitSpeed) {
        double exitSpeed = getMaxAllowSpeed(curBlock->entrySpeed, curBlock->acceleration, curBlock->distance);
        if(curBlock->exitSpeed > exitSpeed) {
          curBlock->exitSpeed = exitSpeed;
          nextBlock->entrySpeed = exitSpeed;
          curBlock->needRecalculate = true;
          nextBlock->needRecalculate = true;
        }
      } else if(curBlock->entrySpeed > curBlock->exitSpeed) {
        double exitSpeed = getMaxAllowSpeed(curBlock->entrySpeed, -curBlock->acceleration, curBlock->distance);
        if(curBlock->exitSpeed < exitSpeed) {
          curBlock->exitSpeed = exitSpeed;
          nextBlock->entrySpeed = exitSpeed;
          curBlock->needRecalculate = true;
          nextBlock->needRecalculate = true;
        }
      }
      curBlockIdx = (curBlockIdx + 1) % blockQueue._capacity;
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
    }
    curBlockIndex = (curBlockIndex + 1) % blockQueue._capacity;
  }
}