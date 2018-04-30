#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  /**
  TODO COMPLETED
  */
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = (H_ * P_ * Ht) + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - (K * H_)) * P_;
  /**
  TODO COMPLETED
  */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations, mainly use the h
    function to convert cartesian to polar coordinates. 
  */
   
  MatrixXd h = MatrixXd(z.rows(), z.cols());
  //state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float sqrt_px2_py2 = sqrt(pow(px, 2) + pow(py, 2));
  //Prevent division by zero
  if (sqrt_px2_py2 < 0.0001) {
    sqrt_px2_py2 = 0.0001;
  }

  //Set measurement function
  h << sqrt_px2_py2,
      atan2(py, px),
      (px * vx + py * vy)/sqrt_px2_py2;
  VectorXd y = z - h;

  /*In C++, atan2() returns values between -pi and pi. 
  When calculating phi in y = z - h(x) for radar measurements, 
  the resulting angle phi in the y vector should be adjusted 
  so that it is between -pi and pi. 
  The Kalman filter is expecting small angle values between the range -pi and pi. 
  HINT: when working in radians, you can add 2\pi 2π or subtract 2\pi 2π 
  until the angle is within the desired range. */
  /*Radar measurement : y[0] - radial distance, y[1] - phi/angle, y[2]- radial velocity*/

  if (y[1] > M_PI) {
    y[1] -= 2.0 * M_PI;

  }
  if (y[1] <-M_PI) {
    y[1] += 2.0 * M_PI;
  } 

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  //new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - (K * H_)) * P_;
}
