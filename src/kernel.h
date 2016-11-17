#pragma once

#include <vector>
#include "scene.h"
#include "lidar.h"

void particleFilterInit(Scene *scene);
void particleFilterFree();
void particleFilter(uchar4 *pbo, int frame, Lidar *lidar);
void drawMap(uchar4 *pbo);

inline int ilog2(int x) {
	int lg = 0;
	while (x >>= 1) {
		++lg;
	}
	return lg;
}

inline int ilog2ceil(int x) {
	return ilog2(x - 1) + 1;
}