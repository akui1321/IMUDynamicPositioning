#ifndef MATRIX_TO_EULER_H
#define MATRIX_TO_EULER_H

#include </usr/include/eigen3/Eigen/Dense>

void calculateEulerAngles(const Eigen::MatrixXd &C, double &pitch, double &roll, double &yaw);

#endif