#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
  debug_option = true;
  if(debug_option) {
    //Set initial headers
    log.log("metric_type,sensor_type,px,py,vx,vy,ro,phi,ro_dot");
  }

}

KalmanFilter::~KalmanFilter() {
  log.close();
}

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
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;

  if(debug_option) {
    string px = to_string(x_(0));
    string py = to_string(x_(1));
    string vx = to_string(x_(2));
    string vy = to_string(x_(3));
    log.log("prediction,none,"+ px +","+ py +","+ vx +","+ vy +",0,0,0");
  }
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ = x_ + (K * y);
  uint x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd xr_(3);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  xr_ << sqrt(px*px + py*py),
         atan2(py,px),
         (px*vx + py*vy)/sqrt(px*px + py*py);

  VectorXd y = z - xr_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ = x_ + (K * y);
  uint x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  if(debug_option) {
    string ro = to_string(xr_(0));
    string phi = to_string(xr_(1));
    string ro_dot = to_string(xr_(2));
    string log_line = "update,lidar," +
                      to_string(px) +","+ to_string(py) +
                      ","+ to_string(vx) +","+ to_string(vy) +
                      ","+ ro +","+ phi +","+ ro_dot;
    log.log(log_line);
  }

}
