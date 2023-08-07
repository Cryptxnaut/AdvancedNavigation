#include "Functions.h"


Point deCasteljau(const std::vector<Point>& controlPoints, double t){
  if(controlPoints.size() == 1){
    return controlPoints[0];
  }
  else{
    std::vector<Point> newControlPoints;
    for(int i = 0; i < controlPoints.size() - 1; i++){
      Point p;
      p.x = (1 - t) * controlPoints[i].x + t * controlPoints[i + 1].x;
      p.y = (1 - t) * controlPoints[i].y + t * controlPoints[i + 1].y;
      newControlPoints.push_back(p);
    }
    return deCasteljau(newControlPoints, t);
  }
}