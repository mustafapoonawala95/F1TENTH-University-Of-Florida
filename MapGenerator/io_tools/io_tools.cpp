

#include "io_tools.hpp"
#include <bits/stdint-uintn.h>
#include <fstream>
#include <ios>
#include <iostream>
#include <string>

namespace igg::io_tools {

  ipb::hw6::image ReadFromPgm(const std::string& file_name) {
    std::ifstream in(file_name, std::ios_base::in);
    if (!in) {
      ipb::hw6::image val;
      std::cout << "Could not find file to open! \n";
      return val;
    }


    std::string type;
    int rows = 0;
    int cols = 0;
    int max_val = 0;
    std::vector<int> data;
    in >> type >> rows >> cols >> max_val;
    std::cout << "This is rows: " << rows << "\n";
    std::cout << "This is cols: " << cols << "\n";
    std::cout << "This is max_val: " << max_val << "\n";
    std::cout << "This is type: " << type << "\n";

    data.resize(rows*cols);
    int byte = 50;                              // THIS WAS int
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        in >> byte;
        data[r*cols + c] = byte;
        //std::cout << "This is byte: " << byte << "\n";
      }
    }
    ipb::hw6::image val2;
    val2.rows(rows);
    val2.cols(cols);
    val2.max_val(max_val);
    val2.data(data);

    return val2;//{rows, cols, static_cast<uint8_t>(max_val), data};
}

bool WriteToPgm(const ipb::hw6::image& image_data, const std::string& file_name) {
  std::ofstream out(file_name);
  if (!out) {
    return false;
  }

  out << "P2" << std::endl
      << image_data.cols() << " " << image_data.rows() << std::endl
      << image_data.max_val() << std::endl;
  for (int r = 0; r < image_data.rows(); ++r) {
    for (int c = 0; c < image_data.cols(); ++c) {
      out << image_data.data()[r*image_data.cols() + c] << " ";
    }
    out << std::endl;
  }
  return true;
}

}  // namespace igg::io_tools
