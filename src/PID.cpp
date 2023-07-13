
#include <math.h>
#include <stdlib.h>
#include <iostream>

#include "main.cpp"
#include "Globals.h"
#include "PID.h"

const double wheelDiameter = 3.250;
double distance;
////////////////////////////
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;

double desiredValue = 200;

double error;
double prevError = 0;
double proportional;
double intergral;
double derivative;

double resetSensors;
double leftMotorPosition;
double rightMotorPosition;
double averagePosition;
double motorPower;

double desiredDistance;


// double getDesiredRotations(wheelDiameter, distance){
//   double circumference = (M_PI * wheelDiameter);
//   double revolutions = distance / circumference;
//   return revolutions
// }

//bool resetSensors = true;
bool enableDrivePID = true;

double PID(){

    PIDcontroller pid;

    if(enableDrivePID == true){
        resetSensors = 0;
    }
    else{
        resetSensors = 1;
    }

    leftMotorPosition = FrontLeft.get_position();
    rightMotorPosition = FrontRight.get_position();
  
    averagePosition = (leftMotorPosition + rightMotorPosition)/2;
  
    proportional = averagePosition - desiredValue;
  
    intergral += proportional;
  
    derivative = proportional - prevError;
  
    motorPower = (proportional * kP + intergral * kI + derivative * kD) / 360;
  
    ////////////////////////////////
  
    leftMotorGroup.move_relative(desiredDistance, motorPower);
    rightMotorGroup.move_relative(desiredDistance, motorPower);
  
    prevError = proportional;
    pros::delay(20);
  // Test
  //test1
  return 1;
}
