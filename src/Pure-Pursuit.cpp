#include "main.h"
#include "Globals.h"
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

    std::vector<std::pair<double, double>> coordinatePairs;

    coordinatePairs.push_back(std::make_pair(1.0, 2.0));
    coordinatePairs.push_back(std::make_pair(3.0, 4.0));
    coordinatePairs.push_back(std::make_pair(5.0, 6.0));

    if(!coordinatePairs.empty()){
        const auto& lastPair = coordinatePairs.back();
        std::cout << "Last Coordinates: (" << lastPair.first << ", " << lastPair.second << ")" << std::endl;
    }
    else{
        std::cout << "No coordinates" << std::endl;
    }

    

}