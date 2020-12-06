#include "trojanmap.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <fstream>
#include <locale>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <utility>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#define pi 3.1415926535897932384626433832795
#define EARTH_RADIUS 6378.137 //KM

//-----------------------------------------------------
// TODO (Students): You do not and should not change the following functions:
//-----------------------------------------------------

/**
 * PrintMenu: Create the menu
 * 
 */
void TrojanMap::PrintMenu() {

  std::string menu =
      "**************************************************************\n"
      "* Select the function you want to execute.                    \n"
      "* 1. Autocomplete                                             \n"
      "* 2. Find the position                                        \n"
      "* 3. CalculateShortestPath                                    \n"
      "* 4. Travelling salesman problem                              \n"
      "* 5. Exit                                                     \n"
      "**************************************************************\n";
  std::cout << menu << std::endl;
  std::string input;
  getline(std::cin, input);
  char number = input[0];
  switch (number)
  {
  case '1':
  {
    menu =
        "**************************************************************\n"
        "* 1. Autocomplete                                             \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a partial location:";
    std::cout << menu;
    getline(std::cin, input);
    auto results = Autocomplete(input);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '2':
  {
    menu =
        "**************************************************************\n"
        "* 2. Find the position                                        \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a location:";
    std::cout << menu;
    getline(std::cin, input);
    auto results = GetPosition(input);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.first != -1) {
      std::cout << "Latitude: " << results.first
                << " Longitude: " << results.second << std::endl;
      PlotPoint(results.first, results.second);
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '3':
  {
    menu =
        "**************************************************************\n"
        "* 3. CalculateShortestPath                                            "
        "      \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the start location:";
    std::cout << menu;
    std::string input1;
    getline(std::cin, input1);
    menu = "Please input the destination:";
    std::cout << menu;
    std::string input2;
    getline(std::cin, input2);
    auto results = CalculateShortestPath(input1, input2);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
      PlotPath(results);
    } else {
      std::cout << "No route from the start point to the destination."
                << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '4':
  {
    menu =
        "**************************************************************\n"
        "* 4. Travelling salesman problem                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
    std::cout << menu << std::endl << std::endl;
    menu = "Please input the number of the places:";
    std::cout << menu;
    getline(std::cin, input);
    int num = std::stoi(input);
    std::vector<std::string> keys;
    for (auto x : data) {
      keys.push_back(x.first);
    }
    std::vector<std::string> locations;
    srand(time(NULL));
    for (int i = 0; i < num; i++)
      locations.push_back(keys[rand() % keys.size()]);
    PlotPoints(locations);
    std::cout << "Calculating ..." << std::endl;
    auto results = TravellingTrojan(locations);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    CreateAnimation(results.second);
    if (results.second.size() != 0) {
      for (auto x : results.second[results.second.size()-1]) std::cout << x << std::endl;
      menu = "**************************************************************\n";
      std::cout << menu;
      std::cout << "The distance of the path is:" << results.first << std::endl;
      PlotPath(results.second[results.second.size()-1]);
    } else {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
           "You could find your animation at src/lib/output.avi.          \n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '5':
    break;
  default:
    std::cout << "Please select 1 - 5." << std::endl;
    PrintMenu();
    break;
  }
}


/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  std::fstream fin;
  fin.open("src/lib/map.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '['), word.end());
      word.erase(std::remove(word.begin(), word.end(), ']'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

/**
 * PlotPoint: Given a location id, plot the point on the map
 * 
 * @param  {std::string} id : location id
 */
void TrojanMap::PlotPoint(std::string id) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(data[id].lat, data[id].lon);
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}
/**
 * PlotPoint: Given a lat and a lon, plot the point on the map
 * 
 * @param  {double} lat : latitude
 * @param  {double} lon : longitude
 */
void TrojanMap::PlotPoint(double lat, double lon) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(lat, lon);
  cv::circle(img, cv::Point(int(result.first), int(result.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPath: Given a vector of location ids draws the path (connects the points)
 * 
 * @param  {std::vector<std::string>} location_ids : path
 */
void TrojanMap::PlotPath(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < int(location_ids.size()); i++) {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::line(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPoints(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}


/**
 * CreateAnimation: Create the videos of the progress to get the path
 * 
 * @param  {std::vector<std::vector<std::string>>} path_progress : the progress to get the path
 */
void TrojanMap::CreateAnimation(std::vector<std::vector<std::string>> path_progress){
  cv::VideoWriter video("src/lib/output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(1248,992));
  for(auto location_ids: path_progress) {
    std::string image_path = cv::samples::findFile("src/lib/input.jpg");
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
    cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
              cv::Scalar(0, 0, 255), cv::FILLED);
    for (auto i = 1; i < int(location_ids.size()); i++) {
      auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
      auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
      cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
                cv::Scalar(0, 0, 255), cv::FILLED);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
    video.write(img);
    cv::imshow("TrojanMap", img);
    cv::waitKey(1);
  }
	video.release();
}
/**
 * GetPlotLocation: Transform the location to the position on the map
 * 
 * @param  {double} lat         : latitude 
 * @param  {double} lon         : longitude
 * @return {std::pair<double, double>}  : position on the map
 */
std::pair<double, double> TrojanMap::GetPlotLocation(double lat, double lon) {
  std::pair<double, double> bottomLeft(34.01001, -118.30000);
  std::pair<double, double> upperRight(34.03302, -118.26502);
  double h = upperRight.first - bottomLeft.first;
  double w = upperRight.second - bottomLeft.second;
  std::pair<double, double> result((lon - bottomLeft.second) / w * 1248,
                                   (1 - (lat - bottomLeft.first) / h) * 992);
  return result;
}

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(std::string id) { return data[id].lat; }

/**
 * GetLon: Get the longitude of a Node given its id. 
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(std::string id) { return data[id].lon; }

/**
 * GetName: Get the name of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(std::string id) { return data[id].name; }

/**
 * GetNeighborIDs: Get the neighbor ids of a Node.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(std::string id) {
    std::vector<std::string> result;
    result = data[id].neighbors;
    return result;
}


/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {Node} a  : node a
 * @param  {Node} b  : node b
 * @return {double}  : distance in mile
 */
//helper functions for rad & RealDistance
double rad(double d)
{
    return d * pi /180.0;
}
double RealDistance(double lat1,double lon1,double lat2,double lon2)
{

	double a;
   	double b;
   	double radLat1 = rad(lat1);
   double radLat2 = rad(lat2);
   a = radLat1 - radLat2;
   b = rad(lon1) - rad(lon2);
   double s = 2 * asin(sqrt(pow(sin(a/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
    s = s * EARTH_RADIUS;
    s = s * 1000*0.62137;
    return s;
}
double TrojanMap::CalculateDistance(const Node &a, const Node &b) {
  // TODO: Use Haversine Formula: 
  //Refer to: https://blog.csdn.net/weixin_41519463/article/details/88999339
  // dlon = lon2 - lon1;
  // dlat = lat2 - lat1;
  // a = (sin(dlat / 2)) ^ 2 + cos(lat1) * cos(lat2) * (sin(dlon / 2)) ^ 2;
  // c = 2 * arcsin(min(1, sqrt(a)));
  // distances = 3961 * c;
  // where 3961 is the approximate radius of the earth at the latitude of
  // Washington, D.C., in miles
 
  return RealDistance(GetLat(a.id), GetLon(a.id), GetLat(b.id), GetLon(b.id));
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  double sum = 0;
  while (path.size() < 2) {
    return sum;
  }
  for (int i = 0; i < int(path.size())-1; i++) {
    sum += CalculateDistance(data[path[i]], data[path[i+1]]);
  }
  return sum;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name) {
  std::vector<std::string> results;
  std::string ids; //make a copy of ids
  std::transform(name.begin(), name.end(), name.begin(), ::tolower);
  //Refer to: https://www.tutorialspoint.com/how-to-convert-std-string-to-lower-case-in-cplusplus
  for (auto id : data) { //for each name in data
  ids = id.second.name;
  std::transform(ids.begin(), ids.end(), ids.begin(), ::tolower);
  //covert input data into the lowercase
    if (name == ids.substr(0, name.size())) {
    //check if the input name match with the data name.
    //refre to: https://www.geeksforgeeks.org/substring-in-cpp/
      results.push_back(id.second.name);
      //add original name to the results
    }
  }
  return results;
}

/**
 * GetPosition: Given a location name, return the position.
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1); // location doesn't exisit
  for (auto id : data) { //for each original name in database
    if (name == id.second.name) { //check if the input name is in the database
      results.first = GetLat(id.first);//Insert Latitude as first param
      results.second = GetLon(id.first);//Insert Longitude as second param
      break;
    }
  }
  return results; // result <Lat, Lon>
}

/**
 * CalculateShortestPath: Given 2 locations, return the shortest path which is a
 * list of id.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
// Dijkstra's Implementation (Distance Vector SPT)
std::vector<std::string> TrojanMap::CalculateShortestPath(
    std::string location1_name, std::string location2_name) {
  //Refer to: https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/?ref=lbp
  double INF = 99999999999999L;
  std::vector<std::string> x; //the path of visited nodes
  std::map<const Node*, double> costs; //store the distance between src node & current node 
  std::map<const Node*, std::string> from; //track where the current node comes from
  std::map<const Node*, bool> visited; //Node status: visited/unvisited

  std::string src_id, dest_id; //store the location id of src & dest node

  for (auto it = data.begin(); it != data.end(); it++) { //traversal all nodes in the map, "data"
    costs[&it->second] = INF; //Initial distance of all nodes as infinite
    if (it->second.name == location1_name) { //check if string of inputs match with data
      src_id = it->second.id; //store the location id of src node
    }
    if (it->second.name == location2_name) {
      dest_id = it->second.id;
    }
  }

  auto compare = [&costs](const Node *a, const Node *b) { //Define the rule to compare two nodes for queue
    return costs[a] > costs[b];//lambda expression
  }; //std::priority_queue <weight(a,k.a distance), vertex>
  std::priority_queue<const Node*, std::vector<const Node*>, decltype(compare)> queue(compare);
  //Refer to: https://en.cppreference.com/w/cpp/language/decltype
  const Node *curr_node = &data[src_id]; //Extract the curr_node from map, "data"
  costs[curr_node] = 0; //Make the distance from src node as zero
  queue.push(curr_node);//Insert src node into the queue

  while (!queue.empty()) { //Looping till priority queue becomes empty (or all distances are not finalized)
    curr_node = queue.top();//The first node in pair is the minimum distance 
    queue.pop();  //Extract the minimum distance node from queue

    if (curr_node->id == dest_id) { //check if it matches with dest_id (found it or not)
      std::string temp = dest_id;
      while (temp != src_id) { //backtracking all prev_nodes of current dest_node
        x.insert(x.begin(), temp);
        temp = from[&data[temp]];
      }
      x.insert(x.begin(), src_id);//add those parent nodes in the front of x vector (a.k.a src_node position)
      return x;
    }
    //if we didn't find it
    visited[curr_node] = true; //assume curr_node has been visited

    for (const std::string &adj_node : curr_node->neighbors) { //loop through and find all adjacent nodes of curr_node
      Node *dest_node = &data[adj_node]; //Extract every nodes that points to the location of curr_node from the map, "data"
      // dest_node --> adj_node
      if (visited[dest_node]) { //If current node is visited
        continue; //skip the visited node and continue
      }
      //Apply formula: dist[v] > dist[u] + weight(u, v)
      double weight = CalculateDistance(*curr_node, *dest_node); //weight(curr_node, dest_node)
      if (costs[dest_node] > costs[curr_node] + weight) { //If there is a shorter path to dest_node through curr_node
        from[dest_node] = curr_node->id;//updating its prev_nodes of shortest path (a.k.a minimum costs)
        //Apply formula: dist[v] = dist[u] + weight(u, v)
        costs[dest_node] = costs[curr_node] + weight; //Updating current distance from curr_node to dest_node
        queue.push(dest_node); //push it into the queue
      }
    }
  }
  
  return x; //print in test cases
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
void TrojanMap::dfs(std::vector<std::string> &ids, std::pair<double, std::vector<std::vector<std::string>>> &results,
		bool visited[], std::vector<std::string> &path, double &min_dist, double dist) {
	//Check if all locations in the input list "ids" has been added to the path
  if (path.size() == ids.size()) { 
		double last_dist = CalculateDistance(data[path[path.size()-1]], data[ids[0]]);
    //Calculate the distance between the last location and the penultimate(last but one) location
		dist += last_dist; //Total distance
		path.push_back(ids[0]);//Record location id into the path
		if (dist < min_dist) { //Begin with the dist < INF
      //check if total distance is shorter than minimum distance
			results.first = dist; //Current Maximum distance
			results.second.push_back(path);//Recored current location id
			min_dist = dist; //Update current minimum distance
		} //Check if there any other shorter dist between locations
		path.erase(path.end() - 1); //Exit the last location
		dist -= last_dist; //Also, substract the dist that are added previously
		return;
	}

	for (size_t i = 1; i < ids.size(); i++) { //Traverased all elements of location id list
		if (!visited[i]) { //Check if current location is in the path or not
			double last_dist = CalculateDistance(data[path[path.size()-1]], data[ids[i]]);
			if (dist + last_dist < min_dist) {
				path.push_back(ids[i]);
				dist += last_dist;
				visited[i] = true;
				dfs(ids, results, visited, path, min_dist, dist);
				visited[i] = false;
				path.erase(path.end() - 1);
				dist -= last_dist;
			}
		}
	}
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan(
                                    std::vector<std::string> &location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> results;
  //Refer to: https://www.geeksforgeeks.org/travelling-salesman-problem-implementation-using-backtracking/
	std::vector<std::string> path;//the path of visited places
  double INF = 99999999999999L;
	bool *visited = new bool[location_ids.size()];//array of true/false
  //We assume all places are connected, so it's best to calculate their distance directly
	for (size_t i = 0; i < location_ids.size(); i++) {
		visited[i] = false;//Assume i-th location is unvisted by default
	}
	visited[0] = true; //Inital location is visited by default
	path.push_back(location_ids[0]);
	dfs(location_ids, results, visited, path, INF, 0);//Recursively call dfs when curr_locations are visited

	delete[] visited; //delete vistied bool value & save the space complexity
  return results;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
                                    std::vector<std::string> &location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> results;
  //Refer to: https://www.youtube.com/watch?v=dYEWqrp-mho&feature=youtu.be
	std::vector<std::string> path;

	for (auto id : location_ids) {
		path.push_back(id);
	}
	path.push_back(location_ids[0]);
	results.second.push_back(path);
	results.first = CalculatePathLength(path);
	int noimprovement = 0;
	while (noimprovement < 10000) {
		int a = rand() % (location_ids.size() - 1) + 1;
		int b = rand() % (location_ids.size() - 1) + 1;
		noimprovement++;
		if (a == b) {
			continue;
		}

		std::string temp = path[a];
		path[a] = path[b];
		path[b] = temp;
		double dist = CalculatePathLength(path);
		if (dist < results.first) {
			results.first = dist;
			results.second.push_back(path);
			noimprovement = 0;
		}
	}
	return results;
}                                    
