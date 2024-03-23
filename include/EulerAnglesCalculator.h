#ifndef EULER_ANGLES_CALCULATOR_H
#define EULER_ANGLES_CALCULATOR_H

#include </usr/include/eigen3/Eigen/Dense>
#include <vector>
#include <cmath>

#define pi 3.1415926535897932384626433832795028841971

class EulerAnglesCalculator {
public:
  EulerAnglesCalculator(const std::vector<double> &data);
  void calculateMatrix();
  double getRoll() const;
  double getPitch() const;
  double getYaw() const;

private:
  Eigen::MatrixXd g_b;
  Eigen::MatrixXd omega_b;
  Eigen::MatrixXd C;
  double yaw;
  double pitch;
  double roll;
  double phi = 30.531651244 / 180.0 * pi;
  double omega_e = 7.292115e-5 * 180.0 / pi;
  double g = 9.7936174;

  Eigen::MatrixXd g_n;
  Eigen::MatrixXd omega_n;
};

#endif
