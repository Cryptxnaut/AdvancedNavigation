#include <iostream>
#include <Eigen/Dense>
#include "Header.hpp"

using namespace Eigen;

typedef Matrix<float, 4, 1> StateVector;
typedef Matrix<float, 4, 4> StateTransitionMatrix;
typedef Matrix<float, 4, 1> ControlInputMatrix;
typedef Matrix<float, 2, 4> MeasurementMatrix;
typedef Matrix<float, 4, 4> ProcessNoiseCovarianceMatrix;
typedef Matrix<float, 2, 2> MeasurementNoiseCovarianceMatrix;

int main() {

	StateVector x;
	x << 0, 0, 0, 0;

	StateTransitionMatrix F;
	F << 1, 0, 1, 0,
		 0, 1, 0, 1,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	ControlInputMatrix G;
	G << 0, 0, 0, 0;

	MeasurementMatrix H;
	H << 1, 0, 0, 0,
		 0, 1, 0, 0;

	ProcessNoiseCovarianceMatrix Q;
	Q << 0.1, 0, 0, 0,
		 0, 0.1, 0, 0,
		 0, 0, 0.1, 0,
		 0, 0, 0, 0.1;

	MeasurementNoiseCovarianceMatrix R;
	R << 1, 0,
		 0, 1;

	Matrix<float, 4, 4> P = Matrix4f::Identity();	//Initial error covariance matrix
	Matrix<float, 2, 1> z;	//Measurement vector
	Matrix<float, 4, 1> u;	// Control input vector

	KalmanFilter kf(x, F, G, H, Q, R, P);


	while (true) {
		StateVector x_predicted = F * x + G * u;
		ProcessNoiseCovarianceMatrix P_predicted = F * P * F.transpose() + Q;

		Matrix<float, 2, 1> y = z - H * x_predicted;
		Matrix<float, 2, 2> S = H * P_predicted * H.transpose() + R;
		Matrix<float, 4, 2> K = P_predicted * H.transpose() * S.inverse();

		x = x_predicted + K * y;
		P = (Matrix<float, 4, 4>::Identity() - K * H) * P_predicted;
	}


}