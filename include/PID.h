#ifndef PID_H
#define PID_H


#include "pros/distance.hpp"
class PIDcontroller{
    public:

    PIDcontroller();
    //PIDcontroller(double desiredDistance);


    void setGains(double kP, double kI, double kD);
    void setDesiredValue(double desiredValue);
    void update(double leftMotorPosition, double rightMotorPosition);
    double getMotorPower();


    private:

    const double wheelDiameter = 3.250;
    double distance;
    
    double kP = 0.3;
    double kI = 0.0021;
    double kD = 0.047;

    double desiredValue;

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
    void PID(double desiredDistance);


    double calculateMotorPower();
    private:
    // PIDcontroller pidController;
    
    
};

#endif