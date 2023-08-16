#pragma once

#include <math.h>


class OdometryClass{
    public:
    

    //Odometry variables
    double DeltaL, DeltaR, DeltaB;
    double DeltaTheta, Theta;
    //double PrevL, PrevR;
    double CurrentL, CurrentR;
    
    double X, Y;
    double DeltaX, DeltaY;
    double YChord;
    double OdometryHeading, GyroHeading;
    
    // Pure Persuit variables
    double DesiredX, DesiredY, DesiredHeading;
    double ChangeinX, ChangeinY;
    double TargetThetaRad, TargetThetaDeg, TargetDistance;
    double IotaSquared, Gamma;
    double TurnAngle;
    
    //Function to convert from degrees to radians
    double Radians(int degrees){
      return (degrees * M_PI) / 180;
    }
    
    bool set_Heading = false;
    double PrevL = 0;
    double PrevR = 0;

  void Odometry();

   
};


