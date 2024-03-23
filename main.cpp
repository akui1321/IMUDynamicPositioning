#include "IMUDataReader.h"
#include "EulerAnglesCalculator.h"
#include "Updating.h"
#include "BLtoNE.h"
#include "Plotter.h"

int main() {
  /*************************Computation preparation*************************/
  std::vector<double> averageData;

  // Read the sample data, measured data, sample reference results, and measured reference results separately
  IMUDataReader data("/home/akui/Downloads/10_23_20/10_23_20_CHA3_IMU.bin");
  IMUDataReader result("/home/akui/Downloads/real.txt");
  IMUDataReader dataExa("/home/akui/Downloads/01/IMU.bin");
  IMUDataReader resultExa("/home/akui/Downloads/01/PureINS.bin");

  dataExa.readData(7);
  resultExa.readData(10);
  result.readTxtFile();

  // Split the measured data
  if (data.readData(7)) {
    data.SplitData();
    averageData = data.calculateAverages(data.SplitResult[0]);
  }

  // Initial alignment
  EulerAnglesCalculator averageEuler(averageData);
  averageEuler.calculateMatrix();

  // Attitude initialization
  Eigen::MatrixXd att_k1Exa(3, 1), att_k1(3, 1);
  att_k1(0, 0) = averageEuler.getRoll();
  att_k1(1, 0) = averageEuler.getPitch();
  att_k1(2, 0) = averageEuler.getYaw();
  att_k1Exa(0, 0) = 0.0107951084511778 / 180.0 * pi;
  att_k1Exa(1, 0) = -2.14251290749072 / 180.0 * pi;
  att_k1Exa(2, 0) = -75.7498049314083 / 180.0 * pi;

  // Velocity initialization
  Eigen::MatrixXd v_k1Exa(3, 1), v_k0Exa(3, 1), v_k1(3, 1), v_k0(3, 1);
  v_k1Exa.fill(0);
  v_k0Exa = v_k1Exa;
  v_k1.fill(0);
  v_k0 = v_k1;

  // Position initialization
  Eigen::MatrixXd pos_k1Exa(3, 1), pos_k0Exa(3, 1), pos_k1(3, 1), pos_k0(3, 1);
  pos_k1(0, 0) = 30.5278800644;
  pos_k1(1, 0) = 114.3556717508;
  pos_k1(2, 0) = 21.895;
  pos_k0 = pos_k1;
  pos_k1Exa(0, 0) = 23.1373950708;
  pos_k1Exa(1, 0) = 113.3713651222;
  pos_k1Exa(2, 0) = 2.175;
  pos_k0Exa = pos_k1Exa;

  /*************************Epoch-wise computation*************************/
  std::vector<std::vector<double>> posExa, velExa, attExa, pos, vel, att;

  // Sample data computation
  for (int i = 0; i < dataExa.rawData.size() - 1; ++i) {
    if (dataExa.rawData[i][0]>=91620) {
      Updating updateExa(dataExa.rawData[i], dataExa.rawData[i + 1], v_k1Exa, pos_k1Exa, att_k1Exa, v_k0Exa, pos_k0Exa);
      updateExa.calculate();
      v_k0Exa = v_k1Exa;
      pos_k0Exa = pos_k1Exa;
      v_k1Exa = updateExa.v_k2;
      pos_k1Exa = updateExa.pos_k2;
      att_k1Exa = updateExa.att_k2;
      posExa.push_back({dataExa.rawData[i][0], pos_k1Exa(0, 0), pos_k1Exa(1, 0), pos_k1Exa(2, 0)});
      velExa.push_back({dataExa.rawData[i][0], v_k1Exa(0, 0), v_k1Exa(1, 0), v_k1Exa(2, 0)});
      attExa.push_back({dataExa.rawData[i][0], att_k1Exa(0, 0) * 180.0 / pi, att_k1Exa(1, 0) * 180.0 / pi, att_k1Exa(2, 0) * 180.0 / pi});
    }
    else {
      continue;
    }
  }

  // Measured data computation
  for (int j = 1; j < data.SplitResult.size(); ++j) {
    if(j % 2 == 1) {
        for (int i = 0; i < data.SplitResult[j].size() - 1; ++i) {
          Updating update(data.SplitResult[j][i], data.SplitResult[j][i + 1], v_k1, pos_k1, att_k1, v_k0, pos_k0);
          update.calculate();
          v_k0 = v_k1;
          pos_k0 = pos_k1;
          v_k1 = update.v_k2;
          pos_k1 = update.pos_k2;
          att_k1 = update.att_k2;
          pos.push_back({data.SplitResult[j][i + 1][0], pos_k1(0, 0), pos_k1(1, 0), pos_k1(2, 0)});
          vel.push_back({data.SplitResult[j][i + 1][0], v_k1(0, 0), v_k1(1, 0), v_k1(2, 0)});
          att.push_back({data.SplitResult[j][i + 1][0], att_k1(0, 0) * 180.0 / pi, att_k1(1, 0) * 180.0 / pi, att_k1(2, 0) * 180.0 / pi});
        }
    } else {
      averageData = data.calculateAverages(data.SplitResult[j]);
      EulerAnglesCalculator alignment(averageData);
      alignment.calculateMatrix();
      att_k1(0, 0) = alignment.getRoll();
      att_k1(1, 0) = alignment.getPitch();
      // att_k1(2, 0) = alignment.getYaw();

      v_k1(0, 0) = 0;
      v_k1(1, 0) = 0;
      v_k1(2, 0) = 0;
      v_k0 = v_k1;

      pos_k0 = pos_k1;
    }
  }

  /****************************Result analysis****************************/
  // Sample data
  std::vector<std::vector<double>> pos_difExa(posExa.size(), std::vector<double>(4)), vel_difExa(velExa.size(), std::vector<double>(4)), att_difExa(attExa.size(), std::vector<double>(4));
  for (int i = 0; i < posExa.size(); ++i) {
      pos_difExa[i][0] = posExa[i][0];
      vel_difExa[i][0] = velExa[i][0];
      att_difExa[i][0] = attExa[i][0];
      for (int j = 1; j < posExa[0].size(); ++j) {
        pos_difExa[i][j] = posExa[i][j] - resultExa.rawData[i][j];
        vel_difExa[i][j] = velExa[i][j] - resultExa.rawData[i][j + 3];
        att_difExa[i][j] = attExa[i][j] - resultExa.rawData[i][j + 6];
      }
  }

  // Plotting images
  Plotter plotExaPosDif(pos_difExa);
  plotExaPosDif.plotGraph(0, 1, "'Latitude Discrepancy of Sample Data'", "'time(s)'", "'latitude discrepancy(degree)'", "'latitude discrepancy'");
  plotExaPosDif.plotGraph(0, 2, "'Longitude Discrepancy of Sample Data'", "'time(s)'", "'longitude discrepancy(degree)'", "'longitude discrepancy'");
  plotExaPosDif.plotGraph(0, 3, "'Elevation Discrepancy of Sample Data'", "'time(s)'", "'elevation discrepancy(m)'", "'elevation discrepancy'");

  Plotter plotExaVelDif(vel_difExa);
  plotExaVelDif.plotGraph(0, 1, "'Northward Velocity Discrepancy of Sample Data'", "'time(s)'", "'northward velocity discrepancy(m/s)'", "'northward velocity discrepancy'");
  plotExaVelDif.plotGraph(0, 2, "'Eastward Velocity Discrepancy of Sample Data'", "'time(s)'", "'eastward velocity discrepancy(m/s)'", "'eastward velocity discrepancy'");
  plotExaVelDif.plotGraph(0, 3, "'Vertical Velocity Discrepancy of Sample Data'", "'time(s)'", "'vertical velocity discrepancy(m/s)'", "'vertical velocity discrepancy'");

  Plotter plotExaAttDif(att_difExa);
  plotExaAttDif.plotGraph(0, 1, "'Roll Discrepancy of Sample Data'", "'time(s)'", "'roll discrepancy(degree)'", "'roll discrepancy'");
  plotExaAttDif.plotGraph(0, 2, "'Pitch Discrepancy of Sample Data'", "'time(s)'", "'pitch discrepancy(degree)'", "'pitch discrepancy'");
  plotExaAttDif.plotGraph(0, 3, "'Yaw Discrepancy of Sample Data'", "'time(s)'", "'yaw discrepancy(degree)'", "'yaw discrepancy'");

  // Measured data
  std::vector<std::vector<double>> pos_dif(pos.size(), std::vector<double>(4)), vel_dif(vel.size(), std::vector<double>(4)), att_dif(att.size(), std::vector<double>(4));
  for (int i = 0; i < pos.size(); ++i) {
      pos_dif[i][0] = pos[i][0];
      pos_dif[i][1] = pos[i][1] - result.rawData[i][2];
      pos_dif[i][2] = pos[i][2] - result.rawData[i][1];
      pos_dif[i][3] = pos[i][3] - result.rawData[i][3];
      vel_dif[i][0] = vel[i][0];
      att_dif[i][0] = att[i][0];
      for (int j = 1; j < vel[0].size(); ++j) {
        vel_dif[i][j] = vel[i][j] - result.rawData[i][j + 6];
        att_dif[i][j] = att[i][j] - result.rawData[i][j + 9];
        if(att_dif[i][j]<-300) {
          att_dif[i][j] += 360;
        }
        if(att_dif[i][j]>300) {
          att_dif[i][j] -= 360;
        }
      }
  }

  // Plotting images
  Plotter plotPosDif(pos_dif);
  plotPosDif.plotGraph(0, 1, "'Latitude Discrepancy of Measured Data'", "'time(s)'", "'latitude discrepancy(degree)'", "'latitude discrepancy'");
  plotPosDif.plotGraph(0, 2, "'Longitude Discrepancy of Measured Data'", "'time(s)'", "'longitude discrepancy(degree)'", "'longitude discrepancy'");
  plotPosDif.plotGraph(0, 3, "'Elevation Discrepancy of Measured Data'", "'time(s)'", "'elevation discrepancy(m)'", "'elevation discrepancy'");

  Plotter plotVelDif(vel_dif);
  plotVelDif.plotGraph(0, 1, "'Northward Velocity Discrepancy of Measured Data'", "'time(s)'", "'northward velocity discrepancy(m/s)'", "'northward velocity discrepancy'");
  plotVelDif.plotGraph(0, 2, "'Eastward Velocity Discrepancy of Measured Data'", "'time(s)'", "'eastward velocity discrepancy(m/s)'", "'eastward velocity discrepancy'");
  plotVelDif.plotGraph(0, 3, "'Vertical Velocity Discrepancy of Measured Data'", "'time(s)'", "'vertical velocity discrepancy(m/s)'", "'vertical velocity discrepancy'");

  Plotter plotAttDif(att_dif);
  plotAttDif.plotGraph(0, 1, "'Roll Discrepancy of Measured Data'", "'time(s)'", "'roll discrepancy(degree)'", "'roll discrepancy'");
  plotAttDif.plotGraph(0, 2, "'Pitch Discrepancy of Measured Data'", "'time(s)'", "'pitch discrepancy(degree)'", "'pitch discrepancy'");
  plotAttDif.plotGraph(0, 3, "'Yaw Discrepancy of Measured Data'", "'time(s)'", "'yaw discrepancy(degree)'", "'yaw discrepancy'");

  pos = BLtoNE(pos);
  Plotter plot(pos);
  plot.plotGraph(2, 1, "'The Planar Trajectory Curve of Measured Data'", "'east(m)'", "'north(m)'", "'planar trajectory'");
  plot.plotGraph(0, 3, "'Elevation Curve of Measured Data'", "'time(s)'", "'elevation(m)'", "'elevation'");

  Plotter plotRe(result.rawData);
  plotRe.plotGraph(1, 2, "'The Planar Trajectory Curve of Reference Result'", "'east(m)'", "'north(m)'", "'planar trajectory'");

  return 0;
}
