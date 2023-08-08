#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <vector>
#include <math.h>
#include <cmath>

struct robotState{
  double x;
  double y;
  double theta;
  double linearVelocity;
  double angularVelocity;
};

using wayPoint = std::pair<double, double>;

robotState deCasteljau(const std::vector<robotState>& wayPoints, double t);

double distance(double x1, double y1, double x2, double y2);

int getClosestPoint(const robotState& robot, const std::vector<wayPoint>& path);

#endif

