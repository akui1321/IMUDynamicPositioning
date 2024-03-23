#include "Plotter.h"
#include <fstream>
#include <iomanip>
#include <iostream>

Plotter::Plotter(const std::vector<std::vector<double>> &data) : data(data) {}

// Plot the graph
void Plotter::plotGraph(int x, int y, std::string title, std::string xlabel, std::string ylabel, std::string legend) {
  // Write data to a temporary file
  std::ofstream tempDataFile("temp_data.txt");
  if (!tempDataFile.is_open()) {
    std::cerr << "Unable to open temporary data file!" << std::endl;
    return;
  }

  for (const auto &row : data) {
    tempDataFile << std::fixed << std::setprecision(32) <<row[x] << ' ' << row[y] << std::endl;
  }

  tempDataFile.close();

  // Using a gnuplot script
  std::ofstream gpFile("plot_script.plt");
  if (!gpFile.is_open()) {
    std::cerr << "Unable to create gnuplot script file!" << std::endl;
    return;
  }

  gpFile << "set term x11 persist\n";
  gpFile << "set title " << title << "\n";
  gpFile << "set xlabel " << xlabel << "\n";
  gpFile << "set ylabel " << ylabel << "\n";
  gpFile << "plot 'temp_data.txt' with lines title " << legend << "\n";

  gpFile.close();

  // Execute the gnuplot script
  system("gnuplot plot_script.plt");

  // Delete temporary files
  remove("temp_data.txt");
  remove("plot_script.plt");

  std::cout << "Press enter to continue..." << std::endl;
  std::cin.get();
}
