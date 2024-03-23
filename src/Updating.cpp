#include "Updating.h"
#include "CrossProduct.h"
#include "Magnitude.h"
#include "MatrixToEuler.h"

// Data initialization
Updating::Updating(std::vector<double> epoch1, std::vector<double> epoch2, Eigen::MatrixXd v_k1, Eigen::MatrixXd pos_k1, Eigen::MatrixXd att_k1, Eigen::MatrixXd v_k0, Eigen::MatrixXd pos_k0) : epoch1(epoch1), epoch2(epoch2), v_k1(v_k1), pos_k1(pos_k1), att_k1(att_k1), v_k0(v_k0), pos_k0(pos_k0) {
  delta_theta_k1 = Eigen::MatrixXd(3, 1);
  delta_v_k1 = Eigen::MatrixXd(3, 1);
  delta_theta_k2 = Eigen::MatrixXd(3, 1);
  delta_v_k2 = Eigen::MatrixXd(3, 1);
  C_b_n = Eigen::MatrixXd(3, 3);
  C_b = Eigen::MatrixXd(3, 3);
  C_n = Eigen::MatrixXd(3, 3);
  C_b_n_k = Eigen::MatrixXd(3, 3);

  for (int i = 0; i < 3; ++i) {
    delta_theta_k1(i, 0) = epoch1[i + 1];
    delta_v_k1(i, 0) = epoch1[i + 4];
    delta_theta_k2(i, 0) = epoch2[i + 1];
    delta_v_k2(i, 0) = epoch2[i + 4];
  }

  omega_en_n = Eigen::MatrixXd(3, 1);
  omega_ie_n = Eigen::MatrixXd(3, 1);

  pos_k2 = Eigen::MatrixXd(3, 1);
  v_k2 = Eigen::MatrixXd(3, 1);
  att_k2 = Eigen::MatrixXd(3, 1);
}

// Updating algorithm
void Updating::calculate() {
  // Velocity updating
  double phi = (3 / 2.0 * pos_k1(0, 0) - 1 / 2.0 * pos_k0(0, 0)) / 180.0 * pi;
  double h = 3 / 2.0 * pos_k1(2, 0) - 1 / 2.0 * pos_k0(2, 0);
  double v_N = 3 / 2.0 * v_k1(0, 0) - 1 / 2.0 * v_k0(0, 0);
  double v_E = 3 / 2.0 * v_k1(1, 0) - 1 / 2.0 * v_k0(1, 0);
  delta_v_b_k = delta_v_k2 + 1 / 2.0 * CrossProduct(delta_theta_k2, delta_v_k2) + 1 / 12.0 * (CrossProduct(delta_theta_k1, delta_v_k2) + CrossProduct(delta_v_k1, delta_theta_k2));

  R_M = (a * (1 - e * e)) / (1 - e * e * sin(phi) * sin(phi)) / sqrt(1 - e * e * sin(phi) * sin(phi));
  R_N = a / sqrt(1 - e * e * sin(phi) * sin(phi));
  omega_ie_n(0, 0) = omega_e * cos(phi);
  omega_ie_n(1, 0) = 0;
  omega_ie_n(2, 0) = -omega_e * sin(phi);
  omega_en_n(0, 0) = v_E / (R_N + h);
  omega_en_n(1, 0) = -v_N / (R_M + h);
  omega_en_n(2, 0) = -(v_E * tan(phi)) / (R_N + h);
  xi = (omega_ie_n + omega_en_n) * delta_t;
  double roll, pitch, yaw;
  roll = att_k1(0, 0);
  pitch = att_k1(1, 0);
  yaw = att_k1(2, 0);

  C_b_n(0, 0) = cos(pitch) * cos(yaw);
  C_b_n(1, 0) = cos(pitch) * sin(yaw);
  C_b_n(2, 0) = -sin(pitch);
  C_b_n(0, 1) = -cos(roll) * sin(yaw) + sin(roll) * sin(pitch) * cos(yaw);
  C_b_n(1, 1) = cos(roll) * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw);
  C_b_n(2, 1) = sin(roll) * cos(pitch);
  C_b_n(0, 2) = sin(roll) * sin(yaw) + cos(roll) * sin(pitch) * cos(yaw);
  C_b_n(1, 2) = -sin(roll) * cos(yaw) + cos(roll) * sin(pitch) * sin(yaw);
  C_b_n(2, 2) = cos(roll) * cos(pitch);

  delta_v_n = C_b_n * delta_v_b_k - 1 / 2.0 * CrossProduct(xi, C_b_n * delta_v_b_k);

  Eigen::MatrixXd g_p_n(3, 1);
  g_p_n.fill(0);
  double g0 = 9.7803267715 * (1 + 0.0052790414 * sin(phi) * sin(phi) + 0.0000232718 * pow(sin(phi), 4));
  g_p_n(2, 0) = g0 - (3.087691089e-6 - 4.397731e-9 * sin(phi) * sin(phi)) * h + 0.721e-12 * h * h;

  Eigen::MatrixXd v_n(3, 1);
  v_n = 3 / 2.0 * v_k1 - 1 / 2.0 * v_k0;
  delta_v_cor = (g_p_n - CrossProduct((2 * omega_ie_n + omega_en_n), v_n)) * delta_t;
  v_k2 = v_k1 + delta_v_n + delta_v_cor;

  // Position updating
  pos_k2(2, 0) = pos_k1(2, 0) - 1 / 2.0 * (v_k1(2, 0) + v_k2(2, 0)) * delta_t;
  double R_M0 = (a * (1 - e * e)) / (1 - e * e * sin(pos_k1(0, 0) / 180.0 * pi) * sin(pos_k1(0, 0) / 180.0 * pi)) / sqrt(1 - e * e * sin(pos_k1(0, 0) / 180.0 * pi) * sin(pos_k1(0, 0) / 180.0 * pi));
  pos_k2(0, 0) = pos_k1(0, 0) + ((v_k2(0, 0) + v_k1(0, 0)) / 2.0 / (R_M0 + (pos_k2(2, 0) + pos_k1(2, 0)) / 2.0) * delta_t) * 180.0 / pi;
  pos_k2(1, 0) = pos_k1(1, 0) + ((v_k2(1, 0) + v_k1(1, 0)) / 2.0 / (R_N + (pos_k2(2, 0) + pos_k1(2, 0)) / 2.0) / cos((pos_k1(0, 0) / 180.0 * pi + pos_k2(0, 0) / 180.0 * pi) / 2.0) * delta_t) * 180.0 / pi;

  //Attitude updating
  phi_k = delta_theta_k2 + 1 / 12.0 * CrossProduct(delta_theta_k1, delta_theta_k2);

  Eigen::MatrixXd omega_ie_n0(3, 1);
  omega_ie_n0(0, 0) = omega_e * cos(pos_k1(0, 0) / 180.0 * pi);
  omega_ie_n0(1, 0) = 0;
  omega_ie_n0(2, 0) = -omega_e * sin(pos_k1(0, 0) / 180.0 * pi);

  double R_N0 = a / sqrt(1 - e * e * sin(pos_k1(0, 0) / 180.0 * pi) * sin(pos_k1(0, 0) / 180.0 * pi));
  Eigen::MatrixXd omega_en_n0(3, 1);
  omega_en_n0(0, 0) = v_k1(1, 0) / (R_N0 + pos_k1(2, 0));
  omega_en_n0(1, 0) = -v_k1(0, 0) / (R_M0 + pos_k1(2, 0));
  omega_en_n0(2, 0) = -v_k1(1, 0) * tan(pos_k1(0, 0) / 180.0 * pi) / (R_N0 + pos_k1(2, 0));

  xi_k = (omega_en_n0 + omega_ie_n0) * delta_t;

  Eigen::Matrix3d identityMatrix = Eigen::Matrix3d::Identity();
  C_b = identityMatrix + sin(Magnitude(phi_k)) / Magnitude(phi_k) * CrossProduct(phi_k, identityMatrix) + (1 - cos(Magnitude(phi_k))) / pow(Magnitude(phi_k), 2) * CrossProduct(phi_k, identityMatrix) * CrossProduct(phi_k, identityMatrix);

  C_n = identityMatrix - sin(Magnitude(xi_k)) / Magnitude(xi_k) * CrossProduct(xi_k, identityMatrix) + (1 - cos(Magnitude(xi_k))) / pow(Magnitude(xi_k), 2) * CrossProduct(xi_k, identityMatrix) * CrossProduct(xi_k, identityMatrix);

  C_b_n_k = C_n * C_b_n * C_b;

  calculateEulerAngles(C_b_n_k, att_k2(1, 0), att_k2(0, 0), att_k2(2, 0));
}
