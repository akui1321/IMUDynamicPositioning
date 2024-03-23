#ifndef IMU_DATA_READER_H
#define IMU_DATA_READER_H

#include <string>
#include <vector>

class IMUDataReader {
public:
  IMUDataReader(const std::string &filename);

  bool readData(int num);

  std::vector<double> calculateAverages(std::vector<std::vector<double>> alignment);

  void SplitData();
  
  std::vector<std::vector<double>> rawData;

  std::vector<std::vector<std::vector<double>>> SplitResult;

  void readTxtFile();

private:
  std::string filename;

  std::vector<double> averages;
};

#endif 
