#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ <<     0.0225, 0,
                    0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ <<     0.09, 0, 0,
                    0, 0.0009, 0,
                    0, 0, 0.09;
    /**
    TODO: done
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
    */

    H_laser_ <<     1, 0, 0, 0,
                    0, 1, 0, 0;

    // Initialise Jacobian matrix with zeros at start
    Hj_.setZero();

    // Inititalise state transition matrix F and process covariance matrix Q:
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.Q_ = MatrixXd(4, 4);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
    *  Initialization
    ****************************************************************************/
    if (!is_initialized_) {
    /**
    TODO: done
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: initialization" << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ <<  1, 1, 1, 1;

    // Create the covariance matrix.
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<  1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

        double rho = measurement_pack.raw_measurements_[0];
        double phi = measurement_pack.raw_measurements_[1];

        ekf_.x_ << rho*cos(phi), rho*sin(phi), 0, 0;

        ekf_.R_ = MatrixXd(3, 3);
        ekf_.R_ = R_radar_;
        ekf_.H_ = MatrixXd(3, 4);
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      set the state with the initial location and zero velocity
      */
        ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

        ekf_.R_ = MatrixXd(2, 2);
        ekf_.R_ = R_laser_;
        ekf_.R_ = MatrixXd(2, 4);
        ekf_.H_ = H_laser_;
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
    TODO: done
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
    */

    // Calculate delta t in seconds:
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    float noise_ax = 9.0;
    float noise_ay = 9.0;

    // Update the state transition matrix F according to the new elapsed time.
    ekf_.F_ <<  1, 0, dt, 0,
                0, 1, 0, dt,
                0, 0, 1, 0,
                0, 0, 0, 1;

    // Update the process noise covariance matrix.
    ekf_.Q_ <<  pow(dt, 4)*noise_ax/4, 0, pow(dt, 3)*noise_ax/2, 0,
                0, pow(dt, 4)*noise_ay/4, 0, pow(dt, 3)*noise_ay/2,
                pow(dt, 3)*noise_ax/2, 0, pow(dt, 2)*noise_ax, 0,
                0, pow(dt, 3)*noise_ay/2, 0, pow(dt, 2)*noise_ay;

    ekf_.Predict();

    /*****************************************************************************
    *  Update
    ****************************************************************************/

    /**
    TODO: done
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
    */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.R_ = MatrixXd(3, 3);
        ekf_.R_ = R_radar_;
        ekf_.H_ = MatrixXd(3, 4);
        Hj_ = tools.CalculateJacobian(ekf_.x_);

        // if Jacobian is ill conditioned we use it from the previous step:
        if(!Hj_.isZero(0)) {
            ekf_.H_ = Hj_;
        }

        ekf_.UpdateEKF(measurement_pack.raw_measurements_);

    } else {
        // Laser updates
        ekf_.R_ = MatrixXd(2, 2);
        ekf_.R_ = R_laser_;
        ekf_.H_ = MatrixXd(2, 4);
        ekf_.H_ = H_laser_;

        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << endl << ekf_.x_ << endl;
    cout << "P_ = " << endl << ekf_.P_ << endl;
    cout << endl;
}
