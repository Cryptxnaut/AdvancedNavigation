#include "Functions.h"


Point deCasteljau(const std::vector<Point>& wayPoints, double t){
  if(wayPoints.size() == 1){
    return wayPoints[0];
  }
  else{
    std::vector<Point> newWayPoints;
    for(int i = 0; i < wayPoints.size() - 1; i++){
      Point p;
      p.x = (1 - t) * wayPoints[i].x + t * wayPoints[i + 1].x;
      p.y = (1 - t) * wayPoints[i].y + t * wayPoints[i + 1].y;
      newWayPoints.push_back(p);
    }
    return deCasteljau(newWayPoints, t);
  }
}

double calculateDistance(const std::pair<double, double>& point1, const std::pair<double, double>& point2){
    double dx = point1.first - point2.first;
    double dy = point1.second - point2.second;
    return sqrt(dx * dx + dy * dy);
}

int getClosestPoint(const Point& robot, const std::vector<wayPoint>& path){


}