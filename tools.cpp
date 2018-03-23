#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO: done
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // necessary argument checks
    if (!estimations.size()) {
        std::cout << "The size of estimation vector is zero." << endl;
        return rmse;
    }
    else if (estimations.size() != ground_truth.size()) {
        std::cout << "Dimensions mismatch." << endl;
        return rmse;
    }

    // calculate RMS error in three steps: first, sum up squares of residuals
    // then calculate mean and take element wise square root:
    for (auto i = 0; i < estimations.size(); i++) {
        VectorXd residuals = estimations[i] - ground_truth[i];
        residuals = residuals.array().pow(2);
        rmse += residuals;
    }
    rmse /= estimations.size();
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO: done
    * Calculate a Jacobian here.
  */

    MatrixXd Hj(3,4);

    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    // Avoid unnecessary calculation by introducing a few variables:
    double p_2 = pow(px, 2) + pow(py, 2);
    double p = pow(p_2, 0.5);
    double p_32 = pow(p, 3);


    //check division by zero
    if( fabs(p) < 0.001 ) {
        cout << "Error! Division by a small number!" << endl;
        Hj.setZero();
        return Hj;
    }

    //compute the Jacobian matrix
    Hj <<   px/p, py/p, 0, 0,
            -py/p_2, px/p_2, 0, 0,
            py*(vx*py - vy*px)/p_32, px*(vy*px - vx*py)/p_32, px/p, py/p;

    return Hj;
}
