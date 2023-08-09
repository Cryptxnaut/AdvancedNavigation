#ifndef PID_H
#define PID_H


class PIDcontroller{
    public:

    PIDcontroller(double desiredDistance);

    void setGains(double kP, double kI, double kD);
    void setDesiredValue(double desiredValue);
    void update(double leftMotorPosition, double rightMotorPosition);
    double getMotorPower;


    private:

    const double wheelDiameter = 3.250;
    double distance;
    
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;

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

    double calculateMotorPower();
    
    
};

#endif