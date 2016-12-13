#pragma once

#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include "glm/glm.hpp"
//#include "utilityCore.hpp"

using namespace std;

class Pointcloud {
private:
    ifstream fp_in;
public:
	Pointcloud(string filename);
	~Pointcloud();

	std::vector<glm::vec4>	points;
};
