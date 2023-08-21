#pragma once

#include <math.h>
#include "main.h"



class OdometryClass{
    public://inline
    double Radians(int degrees){
      return (degrees * M_PI) / 180;
    }
    void Odometry();

    
    //Odometry variables
    float DeltaL, DeltaR, DeltaB;
    float DeltaTheta, Theta;
    //double PrevL, PrevR;
    float CurrentL, CurrentR;
    
    float X = 0, Y = 0;
    float DeltaX, DeltaY;
    float YChord;
    float OdometryHeading, GyroHeading;
    
    // Pure Persuit variables
    double DesiredX, DesiredY, DesiredHeading;
    double ChangeinX, ChangeinY;
    double TargetThetaRad, TargetThetaDeg, TargetDistance;
    double IotaSquared, Gamma;
    double TurnAngle;
    
    //Function to convert from degrees to radians
    
    
    bool set_Heading = false;
    double PrevL = 0;
    double PrevR = 0;
   
};


