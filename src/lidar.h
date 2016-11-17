#pragma once

#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include "glm/glm.hpp"
#include "utilities.h"
#include "sceneStructs.h"

using namespace std;

class Lidar {
public:
    Lidar(string filename);
	~Lidar();

	std::vector<std::vector<float>> scans;
};
