#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using namespace std;
using std::vector;

Tools::Tools() {
}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i) {

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

VectorXd Tools::RadarToCTRV(const VectorXd &radar_data) {
    // Convert radar from polar to cartesian coordinates and initialize state.
    float rho = radar_data[0]; // range
    float phi = radar_data[1]; // bearing
    float rho_dot = radar_data[2]; // velocity of rho
    // Coordinates convertion from polar to cartesian
    float px = rho * cos(phi);
    float py = rho * sin(phi);
    float vx = rho_dot * cos(phi);
    float vy = rho_dot * sin(phi);
    float v  = sqrt(vx * vx + vy * vy);

    VectorXd x = VectorXd(5);
    x << px, py, v, 0, 0;
    return x;
}

float Tools::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2. * M_PI;
    while (angle < -M_PI) angle += 2. * M_PI;
    return angle;
}