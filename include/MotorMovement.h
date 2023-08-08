#ifndef MOTORMOVEMENT_H
#define MOTORMOVEMENT_H


const double wheelDiameter = 3.250;
const double wheelBase = 10.0;
const double tpr = 360.0;

void moveForward(double distance);

void turnLeft(double angle);

void turnRight(double angle);


#endif