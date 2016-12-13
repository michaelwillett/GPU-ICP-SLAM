#pragma once
#include "scene.h"

void drawAll(
	uchar4					*pbo, 
	unsigned int			n, 
	Scene					*hst_scene, 
	glm::vec3				*dev_image, 
	glm::vec3				robotPos,
	Particle				*dev_particles, 
	MAP_TYPE				*dev_occupancyGrid,
	Patch					*dev_maps,
	std::vector<Cluster>	clusters
);