#include "CrossProduct.h"

Eigen::MatrixXd CrossProduct(const Eigen::MatrixXd &mat1, const Eigen::MatrixXd &mat2) {
  Eigen::MatrixXd temp(3, 3);
  temp.fill(0);
  temp(0, 1) = -mat1(2, 0);
  temp(0, 2) = mat1(1, 0);
  temp(1, 0) = mat1(2, 0);
  temp(1, 2) = -mat1(0, 0);
  temp(2, 0) = -mat1(1, 0);
  temp(2, 1) = mat1(0, 0);

  return temp * mat2;
}