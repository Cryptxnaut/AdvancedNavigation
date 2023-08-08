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

//using wayPoints = std::pair<double, double>;

struct wayPoints{
  double x;
  double y;
};

const double LOOKAHEAD_DISTANCE = 0.5;
const double MAX_SPEED = 1.0;
const double MAX_ACCELERATION = 1.0;

wayPoints deCasteljau(const std::vector<wayPoints>& pathPoints, double t);

double distance(double x1, double y1, double x2, double y2);

int getClosestPoint(const robotState& robot, const std::vector<wayPoints>& path);

wayPoints getLookaheadPoint(const robotState& robot, const std::vector<wayPoints>& path);


#endif

