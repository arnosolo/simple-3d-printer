#define X_RANGE_MAX 220
#define X_RANGE_MIN 0
#define Y_RANGE_MAX 220
#define Y_RANGE_MIN 0
#define Z_RANGE_MAX 200
#define Z_RANGE_MIN 0

/******** Motors ********/
#define REVERSE_X_DIR false
#define REVERSE_Y_DIR false
#define REVERSE_Z_DIR true
#define REVERSE_Z1_DIR true
#define REVERSE_E_DIR true

#define STEPS_PER_UNIT_X 80
#define STEPS_PER_UNIT_Y 79.6
#define STEPS_PER_UNIT_Z 400
// #define STEPS_PER_UNIT_Z1 400
#define STEPS_PER_UNIT_E 400

/******** Heaters ********/
#define HOTEND_KP 4.7
#define HOTEND_KI 0.42
#define HOTEND_KD 16

#define HOTBED_KP 4.0
#define HOTBED_KI 1.0
#define HOTBED_KD 0

/******** Planner ********/
#define BLOCK_BUFFER_SIZE 16

// Minimum planner junction speed
#define MINIMUM_PLANNER_SPEED 0.05 // mm/s

#define MAX_ACCELERATION_X 1000 // mm/s^2
#define MAX_ACCELERATION_Y 1000 // mm/s^2
#define MAX_ACCELERATION_Z 40   // mm/s^2
#define MAX_ACCELERATION_E 10000 // mm/s^2

#define MAX_FEEDRATE_X 80 // mm/s
#define MAX_FEEDRATE_Y 80 // mm/s
#define MAX_FEEDRATE_Z 4  // mm/s
#define MAX_FEEDRATE_E 25 // mm/s

#define MIN_STEPS_PER_SEGMENT 6