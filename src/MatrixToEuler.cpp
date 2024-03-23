#include "MatrixToEuler.h"

void calculateEulerAngles(const Eigen::MatrixXd &C, double &pitch, double &roll, double &yaw) {
  pitch = atan2(-C(2, 0), sqrt(C(2, 1) * C(2, 1) + C(2, 2) * C(2, 2)));
  roll = atan2(C(2, 1), C(2, 2));
  yaw = atan2(C(1, 0), C(0, 0));
}