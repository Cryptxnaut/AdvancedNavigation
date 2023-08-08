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
    

    // wayPoints lastPoint = path.back();

    // double lastX = lastPoint.x;
    // double lastY = lastPoint.y;

    for(int i = 0; i < path.size(); i++){
        wayPoints lookaheadPoint = getLookaheadPoint(robot, path);

        double dx = lookaheadPoint.x - robot.x;
        double dy = lookaheadPoint.y - robot.y;
        double targetHeading = atan2(dy, dx);

        double headingError = targetHeading - robot.theta;

        double curvature = calculateCurvature(robot, path);
        double adjustedSpeed = MAX_SPEED / (1 + fabs(curvature));


    }

}

    

