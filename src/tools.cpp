#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0.0, 0.0, 0.0, 0.0;

  if(estimations.size() != ground_truth.size() || estimations.size() == 0){
  	cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // computing the accumulate squared residuals
  for (unsigned int x=0; x < estimations.size(); ++x) {
    VectorXd my_residual = estimations[x] - ground_truth[x];

    // computing the coefficient-wise multiplication
    my_residual = my_residual.array() * my_residual.array();
    rmse += my_residual;
  }

  // computing the mean
  rmse = rmse / estimations.size();

  // computing the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 
  float c1 = (px * px) + (py * py);
  float c2 = sqrt(c1);
  float c3 = ( c1 * c2 );

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }
  
  // compute the Jacobian matrix
  Hj << (px/c2), 				(py/c2), 				0, 		0,
      	-(py/c1), 				(px/c1), 				0, 		0,
      	py*(vx*py - vy*px)/c3, 	px*(px*vy - py*vx)/c3, 	px/c2, 	py/c2;

  return Hj;
}
