#include "main.h"
#include "Globals.h"
#include "PurePursuit.h"
#include "Odometry.h"
#include "api.h"
#include "pros/screen.h"


using namespace std;
#include <math.h>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <iostream>
#include <vector>

//using namespace std::clamp;
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor FrontLeft(15, true);
pros::Motor FrontRight(16);
pros::Motor MiddleLeft(13);
pros::Motor MiddleRight(8, true);
pros::Motor BackLeft(20);
pros::Motor BackRight(14, true);

pros::Imu Inertial(12);


//calibrate

//constant variables
#define WheelDiameter 3.25
#define Tl 4.500 // centre to left encoder wheel
#define Tr 4.500 // centre to right encoder wheel
//#define Ts 7.250 // centre to back encoder wheel
#define Tb 7.750
#define WheelCircumference 10.21017612
#define tpr 360


#define COMPETITION_MODE true

//Class

void OdometryClass::Odometry(){

  FrontLeft.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
	FrontRight.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
	MiddleLeft.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
	MiddleRight.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
	BackLeft.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
	BackRight.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);

  
  
  while(true){
  


    CurrentL = MiddleLeft.get_position();
    CurrentR = FrontRight.get_position();

    CurrentL = fabs(CurrentL);
    CurrentR = fabs(CurrentR);
    //CurrentS = BackEncoder.get_position();

    //Calculates the distance moved by the left and right odometry wheels
    DeltaL = ((CurrentL - PrevL) * WheelCircumference )/ tpr;
    DeltaR = ((CurrentR - PrevR) * WheelCircumference) / tpr;
    //Delta S = (CurrentS - PrevS) * WheelCircumference / tpr;

    //returns the absolute values of DeltaL and R
    // DeltaL = fabs(DeltaL);
    // DeltaR = fabs(DeltaR);
    //DeltaS = fabs(DeltaR);


    DeltaTheta = (DeltaL - DeltaR) / (Tl + Tr);  //rad

    if(DeltaTheta == 0){
      X += DeltaL * sin(Theta);
      Y += DeltaL * cos(Theta);
      
    }
    else {
      YChord = 2 * (DeltaL / DeltaTheta + Tl) * sin(DeltaTheta / 2); //Y coordinate vector - X chord not required
      //XChord = 2 * (DeltaS / DeltaTheta + Ts) * sin(DeltaTheta / 2);
      DeltaX = YChord * sin(Theta + (DeltaTheta / 2));
      DeltaY = YChord * cos(Theta + (DeltaTheta / 2));
      Theta += DeltaTheta;
      X += DeltaX;
      Y += DeltaY;
    }

    //OdometryHeading = Inertial.get_heading();
    OdometryHeading = Theta * (M_PI / 180);

    pros::screen::set_pen(COLOR_BLUE);
    //pros::screen::print(TEXT_MEDIUM, 2, "Hello1: %f");
    pros::screen::print(TEXT_MEDIUM, 3, "CurrentL: %f",CurrentL );
    pros::screen::print(TEXT_MEDIUM, 4, "DeltaL: %f", DeltaL);
    pros::screen::print(TEXT_MEDIUM, 5, "CurrentR: %f", CurrentR);
    pros::screen::print(TEXT_MEDIUM, 6, "DeltaR: %f", DeltaR);
    pros::screen::print(TEXT_MEDIUM, 7, "PrevL: %f",PrevL );
    pros::screen::print(TEXT_MEDIUM, 8, "PrevR: %f", PrevR);
    pros::screen::print(TEXT_MEDIUM, 9, "X coord: %lf", X);
    pros::screen::print(TEXT_MEDIUM, 10, "Y coord: %lf", Y);



    //Resetting the change in turn angle for the next loop
    DeltaTheta = 0;
    //Resets the change in movement of the odometry wheels for the next movement
    PrevL = CurrentL;
    PrevR = CurrentR;

    pros::delay(20);
  }

  
}






















// void PurePersuit(){
//   ChangeinX = DesiredX - X;
//   ChangeinY = DesiredY - Y;

//   TargetThetaRad = atan(ChangeinY / ChangeinX);
//   TargetThetaDeg = TargetThetaRad * (180 / M_PI);
//   TargetDistance = sqrt(pow (ChangeinX, 2) + pow (ChangeinY, 2));

//   IotaSquared = (pow (DesiredX, 2) + pow (DesiredY, 2));
//   Gamma = (DesiredX * 2) / IotaSquared;

//   /////////////////////////////////////////////////////////
//   //Turn Function

//   odometryHeading = Inertial.get_heading();

//   //Quadrant1
//   TurnAngle = 0;
//   if ((DesiredX >= X) && (DesiredY >= Y)){
//     TurnAngle = (GyroHeading + (90 - TargetThetaDeg));
//   }
//   //Quadrant 2
//   if ((DesiredX <= X) && (DesiredY >= Y)){
//     TurnAngle = (GyroHeading - (90 - TargetThetaDeg));
//   }
//   //Quadrant 3
//   if ((DesiredX < X) && (DesiredY < Y)){
//     TurnAngle = (GyroHeading - 90 - TargetThetaDeg);
//   }
//   //Quadrant 4
//   if ((DesiredX > X) && (DesiredY < Y)){
//     TurnAngle = (GyroHeading + 90 + TargetThetaDeg);
//   }

//   bool ret = false;

//   if (DesiredX >= X){
//     ret = turn_right (TurnAngle);
//   }
//   else if (DesiredX < X || DesiredX == X){
//     ret = turn_left (TurnAngle);
//   }

//   if (ret == true){
//     move_forwards_backwards (distance, speed);
//   }

//   //odometry();

//   if((X != std::clamp(DesiredX, DesiredX - 100, DesiredX + 100) || (Y != std::clamp(DesiredY, DesiredY - 100, DesiredY +100)))){
//     PurePersuit();
//     pros::delay(5);
//   }

// }


