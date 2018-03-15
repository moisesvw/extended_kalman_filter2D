#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

// Returns Root Mean Square Error of positions and velocities
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  if(estimations.size() == 0 || ground_truth.size() == 0){
    return rmse ;
  }

  if(estimations.size() != ground_truth.size()){
    return rmse ;
  }

  int n = estimations.size();
  for(int i=0; i < n; ++i){
    rmse = rmse.array() + (estimations[i] - ground_truth[i]).array().square();
  }
  rmse = (rmse/n).array().sqrt();
  return rmse;
}

// Returns Jacobian, maps from position and velocity space to radar measurement space
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  MatrixXd m(3,4);

  if((px+py) == 0) {
    //avoid div by zero.
    return m;
  }

  float px_2 = px * px;
  float py_2 = py * py;
  float px_py_root = sqrt(px_2+py_2);
  float px_py_pow  = pow(px_2+py_2, 3.0/2.0 );

  m << px/px_py_root, py/px_py_root, 0, 0,
       -py/(px_2+py_2), px/(px_2+py_2), 0, 0,
       py*(vx*py-vy*px)/px_py_pow, px*(vx*py-vy*px)/px_py_pow, px/px_py_root, py/px_py_root;

  return m;
}
