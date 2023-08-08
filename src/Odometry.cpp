#include "main.h"
#include "Globals.h"
#include "PurePursuit.h"
#include "Odometry.h"

using namespace std;
#include <math.h>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <iostream>
#include <vector>

//using namespace std::clamp;


//calibrate

//constant variables
#define WheelDiameter 3.25
#define Tl 7.250
#define Tr 7.250
//#define Ts 7.250 // centre to back encoder wheel
#define Tb 7.750
#define WheelCircumference 10.21017612
#define tpr 360

#define COMPETITION_MODE false

//Class


void Odometry(){
  OdometryClass odometry;

  while(!COMPETITION_MODE){


    odometry.CurrentL = FrontLeft.get_position();
    odometry.CurrentR = FrontRight.get_position();
    //CurrentS = BackEncoder.get_position();

    //Calculates the distance moved by the left and right odometry wheels
    odometry.DeltaL = (odometry.CurrentL - odometry.PrevL) * WheelCircumference / tpr;
    odometry.DeltaR = (odometry.CurrentR - odometry.PrevR) * WheelCircumference / tpr;
    //Delta S = (CurrentS - PrevS) * WheelCircumference / tpr;

    //returns the absolute values of DeltaL and R
    odometry.DeltaL = fabs(odometry.DeltaL);
    odometry.DeltaR = fabs(odometry.DeltaR);
    //DeltaS = fabs(DeltaR);

    odometry.DeltaTheta = (odometry.DeltaL - odometry.DeltaR) / (Tl + Tr);

    if(odometry.DeltaTheta == 0){
      odometry.X += odometry.DeltaR * sin(odometry.Theta);
      odometry.Y += odometry.DeltaR * cos(odometry.Theta);
    }
    else {
      odometry.YChord = 2 * (odometry.DeltaR / odometry.DeltaTheta + Tr) * sin(odometry.DeltaTheta / 2); //Y coordinate vector - X chord not required
      //XChord = 2 * (DeltaS / DeltaTheta + Ts) * sin(DeltaTheta / 2);
      odometry.DeltaX = odometry.YChord * sin(odometry.Theta + (odometry.DeltaTheta / 2));
      odometry.DeltaY = odometry.YChord * cos(odometry.Theta + (odometry.DeltaTheta / 2));
      odometry.Theta += odometry.DeltaTheta;
      odometry.X += odometry.DeltaX;
      odometry.Y += odometry.DeltaY;
    }

    odometry.OdometryHeading = Inertial.get_heading();

    //Resetting the change in turn angle for the next loop
    odometry.DeltaTheta = 0;
    //Resets the change in movement of the odometry wheels for the next movement
    odometry.PrevL = odometry.CurrentL;
    odometry.PrevR = odometry.CurrentR;

  }
}


// void odometryTask(void* param){
//   while(true){
//     odometry.Odometry();
//     pros::delay(5);
//   }



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


