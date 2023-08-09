#include "Functions.h"


wayPoints deCasteljau(const std::vector<wayPoints>& pathPoints, double t){
  if(pathPoints.size() == 1){
    return pathPoints[0];
  }
  else{
    std::vector<wayPoints> newPathPoints;
    for(int i = 0; i < pathPoints.size() - 1; i++){
      wayPoints p;
      p.x = (1 - t) * pathPoints[i].x + t * pathPoints[i + 1].x;
      p.y = (1 - t) * pathPoints[i].y + t * pathPoints[i + 1].y;
      newPathPoints.push_back(p);
    }
    return deCasteljau(newPathPoints, t);
  }
}


double distance(double x1, double y1, double x2, double y2){
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

double distanceToFinalPoint(const robotState& robot, const wayPoints& finalPoint){
    return distance(robot.x, robot.y, finalPoint.x, finalPoint.y);
}


//Finding the point on the path closest to the robots current state Lookahead point
int getClosestPoint(const robotState& robot, const std::vector<wayPoints>& path){
    double minDistance = INFINITY;
    int minIndex = 0;

    for(const auto& point : path){
        double d = distance(robot.x, robot.y, point.x, point.y);
        if(d < minDistance){
            minDistance = d;
            minIndex = &point - &path[0];
        }
    }
    return minIndex;
}



//Finding the point on the path furthest from the robots current state claculating curvature
int getFurthestPoint(const robotState& robot, const std::vector<wayPoints>& path){
    double maxDistance = 0;
    int maxIndex = 0;

    for(const auto& point : path){
        double d = distance(robot.x, robot.y, point.x, point.y);
        if(d > maxDistance){
            maxDistance = d;
            maxIndex = &point - &path[0];
        }
    }
    return maxIndex;
}

wayPoints getLookaheadPoint(const robotState& robot, const std::vector<wayPoints>& path){
    int closestpoint = getClosestPoint(robot, path);
    double remainingDistance = LOOKAHEAD_DISTANCE;
    int i = closestpoint;

    while(remainingDistance > 0 && i < path.size() - 1){
        double d = distance(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y);
        if(d > remainingDistance){
            double r = remainingDistance / d;

            //std::vector<wayPoints> controlPoints = {wayPoints{robot.x, robot.y}, path[i], path[i + 1]};
            std::vector<wayPoints> controlPoints;
            wayPoints robotPoints;
            robotPoints.x = robot.x;
            robotPoints.y = robot.y;
            controlPoints.push_back(robotPoints);
            controlPoints.push_back(path[i]);
            controlPoints.push_back(path[i + 1]);

            wayPoints lookAhead = deCasteljau(controlPoints, r);
            return lookAhead;
            
        }
 
        remainingDistance -= d;
        i++;
    }

    wayPoints lastPoint = path[path.size() - 1];
    return lastPoint;
}

double calculateCurvature(const robotState& robot, const std::vector<wayPoints>& path){
    int closestPoint = getClosestPoint(robot, path);
    int furthestPoint = getFurthestPoint(robot, path);

    double closestDistance = distance(path[closestPoint].x, path[closestPoint].y, robot.x, robot.y);
    double furthestDistance = distance(path[closestPoint].x, path[closestPoint].y, path[furthestPoint].x, path[furthestPoint].y);

    if(furthestDistance == 0){
        return 0;
    }

    double curvature = 2 * sin(0.5 * closestDistance / furthestDistance);
    return curvature;
}
