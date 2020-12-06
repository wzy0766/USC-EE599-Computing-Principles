#include <iostream>
#include "src/lib/trojanmap.h"

int main() {

  TrojanMap x;
  x.CreateGraphFromCSVFile();
  x.PrintMenu();

  // std::vector<std::string> input{"123120189", "4011837229", "4011837224", "2514542032", "2514541020", "1931345270", "4015477529", "214470792", "63068532", "6807909279"}; // Input location ids 
  // auto result = x.TravellingTrojan(input);
  // std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  // auto res = result.second;
  // for (const auto &i : res) {
  //   for (const auto &j : i) {
  // std::cout << "My path: "  << j << " " << std::endl; // Print the result path lengths
  //  }
  // }
  // std::vector<std::string> gt{"1759017530", "122814237", "7864610981", "3403035586", "1759017530"}; // Expected order
  // std::cout << "GT path length: "  << x.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  return 0;
}