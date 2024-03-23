#ifndef BL_TO_NE_H
#define BL_TO_NE_H

#include <cmath>
#include <vector>

#define a 6378137
#define e 0.08181919104
#define phi_0 30.5278800281
#define lambda_0 114.3556717814
#define pi 3.1415926535897932384626433832795028841971

std::vector<std::vector<double>> BLtoNE(std::vector<std::vector<double>> BL);

#endif