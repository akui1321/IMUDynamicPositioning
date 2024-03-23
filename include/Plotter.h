#ifndef PLOTTER_H
#define PLOTTER_H

#include <vector>
#include <string>

class Plotter {
public:
  Plotter(const std::vector<std::vector<double>> &data);
  void plotGraph(int x, int y, std::string title, std::string xlabel, std::string ylabel, std::string legend);

private:
  std::vector<std::vector<double>> data;
};

#endif 
