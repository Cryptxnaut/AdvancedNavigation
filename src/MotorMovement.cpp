#include "Globals.h"
#include "Functions.h"
#include "MotorMovement.h"



void moveForward(double distance){
    int ticks = static_cast<int>((distance / (M_PI * wheelDiameter)) * 360);

    leftMotorGroup.move_relative(ticks, 100);
    rightMotorGroup.move_relative(ticks, 100);    
}

void turnLeft(double angle){
    int ticks = static_cast<int>((angle / 360) * (M_PI * wheelBase) / (wheelDiameter * tpr)); 

    leftMotorGroup.move_relative(-ticks, MAX_SPEED);
    rightMotorGroup.move_relative(ticks, MAX_SPEED);
}

void turnRight(double angle){
    int ticks = static_cast<int>((angle / 360) * (M_PI * wheelBase) / (wheelDiameter * tpr)); 

    leftMotorGroup.move_relative(ticks, MAX_SPEED);
    rightMotorGroup.move_relative(-ticks, MAX_SPEED);
}