#include "main.h"
#include "Globals.h"
#include "Functions.h"
#include "PurePursuit.h"
#include "MotorMovement.h"
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
void PurePursuit(std::vector<wayPoints>& path, robotState& robot){
    // PurePursuitClass PurePursuit;
    // OdometryClass Odometry;

    //using wayPoint = std::pair<double, double>;

    
    // robotState robot = {
    //     Odometry.X,
    //     Odometry.Y,
    //     Odometry.Theta,
    //     Odometry.DeltaTheta,
    //     Odometry.DeltaTheta
    // };

    //final point coordinates
    wayPoints finalPoint = path.back();    


    
    wayPoints lookaheadPoint = getLookaheadPoint(robot, path);

    int closestPointIndex = 0;

    while(distanceToFinalPoint(robot, finalPoint) > TARGET_DISTANCE_THRESHOLD){
        //Iterate throught the path points to find the closest lookahead point
        double minDistance = distance(lookaheadPoint.x, lookaheadPoint.y, path[0].x, path[0].y);
        for(int i = 1; i < path.size(); i++){
            double d = distance(lookaheadPoint.x, lookaheadPoint.y, path[i].x, path[i].y);
            if(d < minDistance){
                minDistance = d;
                closestPointIndex = i;
            }
        }
    
        //t is the ratio of the closest point to the final point 0-1
        double t = static_cast<double>(closestPointIndex) / static_cast<double>(path.size() - 1);
    
        //Find the associated waypoint
        wayPoints currentLookaheadPoint = deCasteljau(path, t);
        
        //Find the heading to the lookahead point
        double dx = lookaheadPoint.x - robot.x;
        double dy = lookaheadPoint.y - robot.y;
        double targetHeading = atan2(dy, dx);
    
        //Find the heading error
        double headingError = targetHeading - robot.theta;
    
        //Find the curvature 
        double curvature = calculateCurvature(robot, path);
        double adjustedSpeed = MAX_SPEED / (1 + fabs(curvature));
    
        //Find the linear velocity using proportional control
        double linearVelocity = Kp * headingError;
        linearVelocity = std::max(adjustedSpeed, std::min( adjustedSpeed, linearVelocity));
    
        //Find the acceleration limit
        double acceleration = (linearVelocity - robot.linearVelocity) / 0.1;     
        acceleration = std::max(MAX_ACCELERATION, std::min(MAX_ACCELERATION, acceleration));
    
        robot.theta += headingError;
        robot.linearVelocity += acceleration * 0.1;
        //robot.x += cos(robot.theta) * robot.linear_velocity; // Update x position
        //robot.y += sin(robot.theta) * robot.linear_velocity; // Update y position

        //Calculate the motor speeds
        double leftMotorSpeed = linearVelocity - (curvature * wheelBase) / 2;
        double rightMotorSpeed = linearVelocity + (curvature * wheelBase) / 2;

        //Set the motor speeds
        leftMotorGroup.move_velocity(leftMotorSpeed);
        rightMotorGroup.move_velocity(rightMotorSpeed);

        pros::delay(10);

    }

    leftMotorGroup.move_velocity(0);
    rightMotorGroup.move_velocity(0);

    std::cout << "Robot has reached the final destination" << std::endl;

    
}

    

