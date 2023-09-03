// #pragma once

#ifndef GLOBALS_H
#define GLOBALS_H

#include "main.h"

extern pros::Motor FrontLeft;
extern pros::Motor FrontRight;
extern pros::Motor MiddleLeft;
extern pros::Motor MiddleRight;
extern pros::Motor BackLeft;
extern pros::Motor BackRight;

extern pros::Imu Inertial;
extern pros::Vision VisionSensor;

extern pros::Motor_Group leftMotorGroup;
extern pros::Motor_Group rightMotorGroup;

extern pros::Controller master;

typedef enum motor_encoder_units_e {
  E_MOTOR_ENCODER_DEGREES = 0,   // Position is recorded as angle in degrees
                                 // as a floating point number
  E_MOTOR_ENCODER_ROTATIONS = 1, // Position is recorded as angle in rotations
                                 // as a floating point number
  E_MOTOR_ENCODER_COUNTS = 2,    // Position is recorded as raw encoder ticks
                                 // as a whole number
  E_MOTOR_ENCODER_INVALID = INT32_MAX
} motor_encoder_units_e_t;


extern bool boost;






#endif