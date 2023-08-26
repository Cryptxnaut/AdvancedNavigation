#pragma once
#include <math.h>
#include <vector>

struct robotState{
  double x;
  double y;
  double theta;
  double linearVelocity;
  double angularVelocity;
};

//using wayPoints = std::pair<double, double>;

struct wayPoints{
  double x;
  double y;
};

class PurePursuitClass{
    public:
    using wayPoint = std::pair<double, double>;
    void PurePursuit(std::vector<wayPoints>& path, robotState& robot);

    double DesiredX, DesiredY, DesiredHeading;
    double ChangeinX, ChangeinY;
    double TargetThetaRad, TargetThetaDeg, TargetDistance;
    double IotaSquared, Gamma;
    double TurnAngle;

};