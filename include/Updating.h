#ifndef UPDATING_H
#define UPDATING_H

#include </usr/include/eigen3/Eigen/Dense>
#include <vector>
#include <cmath>

#define omega_e 7.292115e-5
#define a 6378137
#define e 0.08181919104
#define g 9.7936174
#define pi 3.1415926535897932384626433832795028841971

class Updating {
public:
  Updating(std::vector<double> epoch1, std::vector<double> epoch2, Eigen::MatrixXd v_k1, Eigen::MatrixXd pos_k1, Eigen::MatrixXd att_k1, Eigen::MatrixXd v_k0, Eigen::MatrixXd pos_k0);
  void calculate();
  Eigen::MatrixXd att_k2;
  Eigen::MatrixXd pos_k2;
  Eigen::MatrixXd v_k2;

private:
  std::vector<double> epoch1;
  std::vector<double> epoch2;
  Eigen::MatrixXd delta_theta_k1;
  Eigen::MatrixXd delta_v_k1;
  Eigen::MatrixXd delta_theta_k2;
  Eigen::MatrixXd delta_v_k2;
  Eigen::MatrixXd delta_v_b_k;
  Eigen::MatrixXd delta_v_n;
  Eigen::MatrixXd delta_v_cor;
  Eigen::MatrixXd xi;
  Eigen::MatrixXd v_k1;
  Eigen::MatrixXd pos_k1;
  Eigen::MatrixXd att_k1;
  Eigen::MatrixXd v_k0;
  Eigen::MatrixXd pos_k0;

  double R_M;
  double R_N;
  Eigen::MatrixXd omega_ie_n;
  Eigen::MatrixXd omega_en_n;

  Eigen::MatrixXd C_b_n;
  Eigen::MatrixXd phi_k;
  Eigen::MatrixXd xi_k;
  Eigen::MatrixXd C_b;
  Eigen::MatrixXd C_n;
  Eigen::MatrixXd C_b_n_k;

  double delta_t = 0.005;
};

#endif