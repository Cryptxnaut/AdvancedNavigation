#include "main.h"
#include "Globals.h"
#include "Functions.h"
#include "Pure-Pursuit.h"
#include "Odometry.h"

using namespace std;

#include <math.h>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <utility>


//const std::vector<wayPoints>& path, const robotState& robot
void PurePursuit(){
    PurePursuitClass PurePursuit;
    OdometryClass Odometry;

    using wayPoint = std::pair<double, double>;

    std::vector<wayPoints> path = {
        {0.0, 0.0},
        {0.0, 1.0},
        {1.0, 1.0},
        {1.0, 0.0},
        {0.0, 0.0}
    };

    robotState robot = {
        Odometry.X,
        Odometry.Y,
        Odometry.Theta,
        Odometry.DeltaTheta,
        Odometry.DeltaTheta
    };

    wayPoints finalPoint = path.back();    


    for(int i = 0; i < path.size(); i++){
        wayPoints lookaheadPoint = getLookaheadPoint(robot, path);

        int closestPointIndex = 0;
        double minDistance = distance(lookaheadPoint.x, lookaheadPoint.y, path[0].x, path[0].y);

        for(int i = 1; i < path.size(); i++){
            double d = distance(lookaheadPoint.x, lookaheadPoint.y, path[i].x, path[i].y);
            if(d < minDistance){
                minDistance = d;
                closestPointIndex = i;
            }
        }

        double t = static_cast<double>(closestPointIndex) / static_cast<double>(path.size() - 1);


        double dx = lookaheadPoint.x - robot.x;
        double dy = lookaheadPoint.y - robot.y;
        double targetHeading = atan2(dy, dx);

        double headingError = targetHeading - robot.theta;

        double curvature = calculateCurvature(robot, path);
        double adjustedSpeed = MAX_SPEED / (1 + fabs(curvature));

        double linearVelocity = Kp * headingError;
        linearVelocity = std::max(adjustedSpeed, std::min( adjustedSpeed, linearVelocity));

        double acceleration = (linearVelocity - robot.linearVelocity) / 0.1;
        acceleration = std::max(MAX_ACCELERATION, std::min(MAX_ACCELERATION, acceleration));

        robot.theta += headingError;
        robot.linearVelocity += acceleration * 0.1;


    }
}

    

