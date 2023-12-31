#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <vector>
#include <math.h>
#include <cmath>
#include "PurePursuit.h"



const double LOOKAHEAD_DISTANCE = 0.5;
const double MAX_SPEED = 100.0;
const double MAX_ACCELERATION = 100.0;
const double MAX_ANGULAR_VELOCITY = 100.0;
const double Kp = 0.5;
const double TARGET_DISTANCE_THRESHOLD = 0.1;

wayPoints deCasteljau(const std::vector<wayPoints>& pathPoints, double t);

double distance(double x1, double y1, double x2, double y2);

double distanceToFinalPoint(const robotState& robot, const wayPoints& finalPoint);

int getClosestPoint(const robotState& robot, const std::vector<wayPoints>& path);

wayPoints getLookaheadPoint(const robotState& robot, const std::vector<wayPoints>& path);

double calculateCurvature(const robotState& robot, const std::vector<wayPoints>& path);

robotState robot();


#endif

