#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <vector>
#include <math.h>

struct Point{
  double x;
  double y;
};

Point deCasteljau(const std::vector<Point>& controlPoints, double t);

#endif

