#ifndef __CSV_LOADER_H__
#define __CSV_LOADER_H__
#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace ucf {
std::vector<std::vector<std::string>> loadCSV(const std::string &filename) {
  std::vector<std::vector<std::string>> data; // Stores the parsed data

  // Open the CSV file
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("File could not be opened!");
  }

  std::string line;
  while (std::getline(file, line)) {
    std::vector<std::string> row;
    std::stringstream line_stream(line);
    std::string cell;
    // Split the line into cells by commas
    while (std::getline(line_stream, cell, ',')) {
      row.push_back(cell);
    }

    data.push_back(row);
  }

  file.close();
  return data;
}
} // namespace ucf
#endif // __CSV_LOADER_H__