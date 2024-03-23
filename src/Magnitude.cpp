#include "Magnitude.h"

double Magnitude(const Eigen::MatrixXd &mat) {
  return sqrt(mat(0, 0) * mat(0, 0) + mat(1, 0) * mat(1, 0) + mat(2, 0) * mat(2, 0));
}