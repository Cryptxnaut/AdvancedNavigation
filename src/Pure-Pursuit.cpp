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

    wayPoints lastPoint = path.back();

    double lastX = lastPoint.x;
    double lastY = lastPoint.y;

 }

    

