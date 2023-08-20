#include "Globals.h"
#include "Functions.h"
#include "MotorMovement.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor FrontLeft(15, true);   
pros::Motor FrontRight(16);
pros::Motor MiddleLeft(13);
pros::Motor MiddleRight(8, true);
pros::Motor BackLeft(20);
pros::Motor BackRight(14, true);

pros::Imu Inertial(12);
pros::Vision VisionSensor(9);

pros::Motor_Group leftMotorGroup ({FrontLeft, MiddleLeft, BackLeft});
pros::Motor_Group rightMotorGroup ({FrontRight, MiddleRight, BackRight});

void moveForward(double distance){
    int ticks = static_cast<int>((distance / (M_PI * wheelDiameter)) * 360);

    leftMotorGroup.move_relative(ticks, MAX_SPEED);
    rightMotorGroup.move_relative(ticks, MAX_SPEED);    
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