#pragma once

#include <string>
#include <vector>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include "glm/glm.hpp"

#define BACKGROUND_COLOR (glm::vec3(0.0f))
#define MAP_TYPE char

struct Camera {
    glm::ivec2 resolution;
    glm::vec3 position;
    glm::vec3 lookAt;
    glm::vec3 view;
    glm::vec3 up;
    glm::vec3 right;
    glm::vec2 fov;
    glm::vec2 pixelLength;
};

struct RenderState {
    Camera camera;
    std::vector<glm::vec3> image;
    std::string imageName;
};

struct ParticleHistory {
	std::vector<glm::vec3> patchPos;
};

struct Particle {
	glm::vec3 pos;
	float w;
	unsigned char cluster;
	ParticleHistory *map;
};

struct Patch {
	glm::vec3 scale;
	glm::vec3 resolution;
	MAP_TYPE *grid;
	unsigned char uid;
};

struct Node
{
	glm::vec2 pos;
	float dist;
};

struct Cluster {
	unsigned char id;
	unsigned int nodeIdx;
	std::vector<unsigned int> patchList;

	std::vector<Node> nodes;
	std::vector<std::vector<unsigned int>> edges;
};

class Map {
public:
	void UpdateMap();
	int mapCorrelation(int N, const MAP_TYPE *map, glm::ivec2 dim, const glm::vec2 *points);
private:
	MAP_TYPE map;
};

class PointCloud : Map {
public:

private:
	glm::vec3 points;
};

class OccupancyGrid : Map {
private:
	MAP_TYPE map;
};