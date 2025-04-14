
#ifndef CONFIG_H
#define CONFIG_H

#include <math.h>

// The pins of the servos.
#define LEFT_SERVO_PIN 9
#define RIGHT_SERVO_PIN 8

// The distance between the servo axes in mm.
#define SHOULDER_SPACING 100.0

// The length of the upper and lower arms in mm.
#define UPPER_ARM_LENGTH 130.0
#define LOWER_ARM_LENGTH 275.0

// The maximum servo angles in deg.
// 0 deg. is parallel to the x - axis, - is upwards, + is downwards.
#define MIN_SHOULDER_ANGLE -10.0
#define MAX_SHOULDER_ANGLE 90.0

// The maximum angle between the upper and lower arms in degrees.
#define MAX_ELBOW_ANGLE 170.0

// The offset of the servo angles in degrees.
// - is upwards, + is downwards.
#define LEFT_SERVO_ANGLE_OFFSET 0.0
#define RIGHT_SERVO_ANGLE_OFFSET 5.0

// The range of the servo angles in degrees.
#define SERVO_ANGLE_RANGE 270.0

// The minimum and maximum pulse widths in microseconds.
#define SERVO_MIN_PULSE_WIDTH 500
#define SERVO_MAX_PULSE_WIDTH 2500

// The size of the interpolation steps in mm.
#define INTERPOLATION_STEP_SIZE 2.0

// The default speed in mm/s.
#define DEFAULT_SPEED 100

#endif