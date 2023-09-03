
#include <math.h>
#include <stdlib.h>
#include <iostream>

#include "main.cpp"
#include "Globals.h"
#include "PID.h"


// double getDesiredRotations(wheelDiameter, distance){
//   double circumference = (M_PI * wheelDiameter);
//   double revolutions = distance / circumference;
//   return revolutions
// }

bool resetSensors = true;
bool enableDrivePID = true;

void PIDcontroller::setGains(double kP, double kI, double kD){
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

void PIDcontroller::setDesiredValue(double desiredValue){
    this->desiredValue = desiredValue;

}

void PIDcontroller::PID(double desiredDistance){

    // PIDcontroller pid(0, 0);
    // pid.setDesiredValue(200);

    if(enableDrivePID == true){
        resetSensors = 0;
    }
    else{
        resetSensors = 1;
    }
    while(true){
        leftMotorPosition = FrontLeft.get_position();
        rightMotorPosition = FrontRight.get_position();
    
        averagePosition = (leftMotorPosition + rightMotorPosition)/2;
    
        proportional = averagePosition - desiredValue;
    
        intergral += proportional;
    
        derivative = proportional - prevError;
    
        motorPower = (proportional * kP + intergral * kI + derivative * kD) / 360;
    
        ////////////////////////////////
    
        // leftMotorGroup.move_relative(desiredDistance, motorPower);
        // rightMotorGroup.move_relative(desiredDistance, motorPower);
    
        prevError = proportional;

        double timestamp = pros::millis();

        pros::delay(20);
        pros::screen::print(TEXT_MEDIUM, 3, "TimeStamp: %f",timestamp );
        pros::screen::print(TEXT_MEDIUM, 3, "Proportional: %f",proportional );
        printf("%f, %f \n" , timestamp, proportional );
    }
  
//   return 1;
}

double PIDcontroller::getMotorPower(){
    return motorPower;
}
