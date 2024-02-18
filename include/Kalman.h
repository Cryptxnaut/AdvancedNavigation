#pragma once

#include <iostream>
#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(
        const Eigen::MatrixXd& x,
        const Eigen::MatrixXd& F,
        const Eigen::MatrixXd& G,
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P

    );
};

