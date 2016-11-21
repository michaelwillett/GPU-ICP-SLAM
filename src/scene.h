#pragma once

#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include "glm/glm.hpp"
#include "utilities.h"
#include "sceneStructs.h"

using namespace std;

class Scene {
private:
    ifstream fp_in;
    int loadGeom();
    int loadCamera();
public:
    Scene(string filename);
    ~Scene();

    std::vector<Patch> maps;
    RenderState state;
};
