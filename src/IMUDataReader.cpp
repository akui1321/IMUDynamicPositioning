#include "IMUDataReader.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>

IMUDataReader::IMUDataReader(const std::string &filename) : filename(filename) {}

// Read the file
bool IMUDataReader::readData(int num) {
  std::ifstream inputFile(filename, std::ios::binary);

  if (!inputFile.is_open()) {
    return false;
  }

  while (true) {
    double data[num];
    inputFile.read(reinterpret_cast<char *>(&data), sizeof(data));

    if (inputFile.eof()) {
      break;
    }

    std::vector<double> row(data, data + num);
    rawData.push_back(row);
  }

  inputFile.close();
  return true;
}

// Calculate the average of the static data
std::vector<double> IMUDataReader::calculateAverages(std::vector<std::vector<double>> alignment) {
  for (auto &entry : alignment) {
    for (int i = 1; i < 7; ++i) {
      entry[i] *= 200.0;
    }
  }

  int numValues = alignment[0].size();
  averages.resize(numValues, 0.0);
  int numEntries = alignment.size();

  int bias = 0;

  for (const auto &entry : alignment) {
    if (entry[1] > (0.00002*200.0)) {
      bias++;
      continue;
    }

    for (int i = 0; i < numValues; ++i) {
      averages[i] += entry[i];
    }
  }

  for (double &average : averages) {
    average /= (numEntries-bias);
  }

  return averages;
}

// Split data
void IMUDataReader::SplitData() {
  std::vector<std::vector<double>> currentArray;

  for (const auto &row : rawData) {
    if (fabs(row[0] - 97704.998) < 0.001 || fabs(row[0] - 97936.998) < 0.001 || fabs(row[0] - 98005.998) < 0.001 || fabs(row[0] - 98267.998) < 0.001 || fabs(row[0] - 98312.998) < 0.001 || fabs(row[0] - 98579.998) < 0.001 || fabs(row[0] - 98638.998) < 0.001 || fabs(row[0] - 98900.998) < 0.001 || fabs(row[0] - 98961.998) < 0.001 || fabs(row[0] - 99234.998) < 0.001) {
      SplitResult.push_back(currentArray);
      currentArray.clear();
    }

      currentArray.push_back(row);
    }

  if (!currentArray.empty()) {
    SplitResult.push_back(currentArray);
  }
}

// Read the txt file
void IMUDataReader::readTxtFile() {
  std::ifstream inputFile(filename);

  if (!inputFile.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return;
  }

  std::string line;
  int lineNumber = 0;
  bool dynamic = false;
  bool err = false;

  while (std::getline(inputFile, line)) {
    ++lineNumber;

    if (lineNumber < 53) {
      continue;
    }

    std::stringstream ss(line);
    std::vector<double> row;
    double value;

    ss >> value;

    if (fabs(value - 97705.003) <= 0.002 || fabs(value - 98006.003) <= 0.002 ||  fabs(value - 98313.003) <= 0.002 || fabs(value - 98639.003) <= 0.002 || fabs(value - 98962.003) <= 0.002) {
      dynamic = true;
    }

    if (fabs(value - 97936.998) <= 0.002 || fabs(value - 98267.998) <= 0.002 || fabs(value - 98579.998) <= 0.002 || fabs(value -98900.998) <= 0.002 || fabs(value - 99234.998) <= 0.002) {
      dynamic = false;
    }

    if (fabs(std::fmod(value, 1.0)) < 0.001) {
      err = true;
    } 
    else {
      err = false;
    }

    if (!err) {
      if (dynamic) {
        row.push_back(value);
        for (int i = 1; i < 19; ++i) {
          ss >> value;
          row.push_back(value);
        }
        rawData.push_back(row);
      }
    }
  }

  inputFile.close();
}
