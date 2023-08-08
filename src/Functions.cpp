#include "Functions.h"


robotState deCasteljau(const std::vector<robotState>& wayPoints, double t){
  if(wayPoints.size() == 1){
    return wayPoints[0];
  }
  else{
    std::vector<robotState> newWayPoints;
    for(int i = 0; i < wayPoints.size() - 1; i++){
      robotState p;
      p.x = (1 - t) * wayPoints[i].x + t * wayPoints[i + 1].x;
      p.y = (1 - t) * wayPoints[i].y + t * wayPoints[i + 1].y;
      newWayPoints.push_back(p);
    }
    return deCasteljau(newWayPoints, t);
  }
}

double distance(double x1, double y1, double x2, double y2){
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

//Finding the point on the path closest to the robots current state Lookahead point
int getClosestPoint(const robotState& robot, const std::vector<wayPoint>& path){
    double minDistance = INFINITY;
    int minIndex = 0;

    for(int i = 0; i < path.size(); i++){
        double d = distance(robot.x, robot.y, path[i].first, path[i].second);
        if(d < minDistance){
            minDistance = d;
            minIndex = i;
        }
    }
    return minIndex;
}

//Finding the point on the path furthest from the robots current state claculating curvature
int getFurthestPoint(const robotState& robot, const std::vector<wayPoint>& path){
    double maxDistance = 0;
    int maxIndex = 0;

    for(int i = 0; i < path.size(); i++){
        double d = distance(robot.x, robot.y, path[i].first, path[i].second);
        if(d > maxDistance){
            maxDistance = d;
            maxIndex = i;
        }
    }
    return maxIndex;
}

