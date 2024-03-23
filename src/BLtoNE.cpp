#include "BLtoNE.h"

// Conversion of latitude and longitude to northeast coordinates
std::vector<std::vector<double>> BLtoNE(std::vector<std::vector<double>> BL) {
  std::vector<std::vector<double>> NE;
  NE = BL;
  for(auto &row : NE) {
    double R_M = a * (1 - e * e) / (1 - e * e * sin(row[1] / 180.0 * pi) * sin(row[1] / 180.0 * pi)) / sqrt(1 - e * e * sin(row[1] / 180.0 * pi) * sin(row[1] / 180.0 * pi));
    double R_N = a / sqrt(1 - e * e * sin(row[1] / 180.0 * pi) * sin(row[1] / 180.0 * pi));
    row[2] = (row[2] - lambda_0) / 180.0 * pi * (R_N + row[3]) * cos(row[1] / 180.0 * pi);
    row[1] = (row[1] - phi_0) / 180.0 * pi * (R_M + row[3]);
  }

  return NE;
}