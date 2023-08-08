#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <vector>
#include <math.h>
#include <cmath>

struct Point{
  double x;
  double y;
};

using wayPoint = std::pair<double, double>;

Point deCasteljau(const std::vector<Point>& wayPoints, double t);

double calculateDistance(const std::pair<double, double>& point1, const std::pair<double, double>& point2);

int getClosestPoint(const Point& robot, const std::vector<wayPoint>& path);

#endif

