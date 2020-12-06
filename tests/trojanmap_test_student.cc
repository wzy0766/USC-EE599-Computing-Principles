#include "src/lib/trojanmap.h"

#include <map>
#include <vector>
#include <algorithm>

#include "gtest/gtest.h"

// Test AutoComplete Function
TEST(TrojanMapTest, TrojanMapTest) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test the simple case
  auto names = m.Autocomplete("Ra");
  std::vector<std::string> gt_1 = {"Ralphs", "Rance39s Chicago Pizza", "Ramen KenJo"};
  EXPECT_EQ(names, gt_1);
  // Test the lower case
  names = m.Autocomplete("ra");
  std::vector<std::string> gt_2 = {"Ralphs", "Rance39s Chicago Pizza", "Ramen KenJo"};
  EXPECT_EQ(names, gt_2);
  // Test the lower and upper case
  names = m.Autocomplete("rA");
  std::vector<std::string> gt_3 = {"Ralphs", "Rance39s Chicago Pizza", "Ramen KenJo"};
  EXPECT_EQ(names, gt_3);
}

// Test FindPosition Function
TEST(TrojanMapTest, FindPositionTest) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test Trader Joe39s
  auto position = m.GetPosition("Trader Joe39s");
  std::pair<double, double> gt_1(34.0257159, -118.2848075);
  EXPECT_EQ(position, gt_1);
  // Test Bank of America
  position = m.GetPosition("Bank of America");
  std::pair<double, double> gt_2(34.025187, -118.2841713);
  EXPECT_EQ(position, gt_2);
  // Test Popeyes Louisiana Kitchen
  position = m.GetPosition("Popeyes Louisiana Kitchen");
  std::pair<double, double> gt_3(34.0282131, -118.2756114);
  EXPECT_EQ(position, gt_3);
}

// Test CalculateShortestPath function
TEST(TrojanMapTest, CalculateShortestPathTest1) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test from Ralphs to Target
  auto path = m.CalculateShortestPath("Ralphs", "Target");
  std::vector<std::string> gt{
      "2578244375", "5559640911", "6787470571", "6808093910", "6808093913", "6808093919", "6816831441", 
      "6813405269", "6816193784", "6389467806", "6816193783", "123178876", "2613117895", "122719259", 
      "6807243574", "6807243576", "213332111", "441895337", "441895335", "122719255", "2613117893", 
      "6813405231", "122719216", "6813405232", "4015372486", "7071032399", "4015372485", "6813379479", 
      "5237417650"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to Target
  path = m.CalculateShortestPath("Target", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath function 2
TEST(TrojanMapTest, CalculateShortestPathTest2) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto path = m.CalculateShortestPath("CVS", "Kentucky Fried Chicken");
  // Test from CVS to Kentucky Fried Chicken
  std::vector<std::string> gt{
      "3088548446", "6813379501", "123241965", "5690152761", "6457457928", "6813379532", "6457457923", 
      "4835551238", "4012848858", "7735888673", "4835551106", "6813565297", "122719205", "6813565294", 
      "4835551232", "4835551104", "4012842272", "4835551103", "123178841", "6813565313", "122814435", 
      "6813565312", "4835551101", "4835551096", "3088547686"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);

  // Reverse the input from Kentucky Fried Chicken to CVS
  path = m.CalculateShortestPath("Kentucky Fried Chicken", "CVS");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test TSP function
TEST(TrojanMapTest, TSP_Test1) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();// # of places: 3
  std::vector<std::string> input{"6788499810", "4012726936", "6788499810"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6788499810", "4012726936", "6788499810", "6788499810"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

// Test TSP function 2
TEST(TrojanMapTest, TSP_Test2) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();// # of places: 4
  std::vector<std::string> input{"6820972476", "6816305554", "1773954300", "122684563", "6820972476"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6820972476", "6816305554", "1773954300", "122684563", "6820972476", "6820972476"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  
  EXPECT_EQ(flag, true);
}

// Test TSP function 3
TEST(TrojanMapTest, TSP_Test3) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();// # of places: 5
  std::vector<std::string> input{"7771782317", "6987230639", "6816193696", "5514004014", "1862312572", "7771782317"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: " <<result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"7771782317", "6987230639", "6816193696", "5514004014", "1862312572", "7771782317", "7771782317"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  
  EXPECT_EQ(flag, true);
}

// Test TSP_2opt function 1
TEST(TrojanMapTest, TSP2_2opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1862312636", "7424270441", "67666219", "4015405548", "4015203110", "6807439002"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"1862312636", "4015405548", "4015203110", "6807439002", "7424270441", "67666219", "1862312636"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  
  EXPECT_EQ(flag, true);
}

// Test TSP_2opt function 2
TEST(TrojanMapTest, TSP_2opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1873056015", "6905329551", "213332060", "1931345270"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"1873056015", "213332060", "1931345270", "6905329551", "1873056015"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}
/* Tht correction of test cases are NOT always guranteed
// Test TSP_2opt function 3
TEST(TrojanMapTest, TSP3) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"123120189", "4011837229", "4011837224", "2514542032", "2514541020", "1931345270", "4015477529", "214470792", "63068532", "6807909279"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: " <<result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"123120189", "1931345270", "4011837224", "4011837229", "2514542032", "2514541020", "6807909279", "63068532", "214470792", "4015477529", "123120189"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  
  EXPECT_EQ(flag, true);
}
*/