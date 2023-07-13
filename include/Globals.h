#pragma once

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

extern bool boost;


#endif