#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

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

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd m(2,2) ;
  m << 1,1,1,1;
  return m;
}
