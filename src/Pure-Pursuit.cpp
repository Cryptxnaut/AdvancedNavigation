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

    std::vector<wayPoint> path;

    path.push_back(std::make_pair(1.0, 2.0));
    path.push_back(std::make_pair(3.0, 4.0));
    path.push_back(std::make_pair(5.0, 6.0));

    // if(!coordinatePairs.empty()){
    //     const auto& lastPair = coordinatePairs.back();
    // }


    // if(coordinatePairs.size() >= 2){
    //     double distance = calculateDistance(coordinatePairs.front(), coordinatePairs.back());
    // }


  


    

}