#pragma once

#include <math.h>


class OdometryClass{
    public://inline
    double Radians(int degrees){
      return (degrees * M_PI) / 180;
    }
    void Odometry();

    
    //Odometry variables
    double DeltaL, DeltaR, DeltaB;
    double DeltaTheta, Theta;
    //double PrevL, PrevR;
    double CurrentL, CurrentR;
    
    double X = 0, Y = 0;
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
    
    
    bool set_Heading = false;
    double PrevL = 0;
    double PrevR = 0;

    

   
};


