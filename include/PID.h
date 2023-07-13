

class PIDcontroller{
    public:

    PIDcontroller(double motorPower, double desiredDistance);

    const double wheelDiameter = 3.250;
    double distance;
    ////////////////////////////
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;

    double desiredValue = 200;

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

    
};