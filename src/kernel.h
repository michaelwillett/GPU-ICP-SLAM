#pragma once

#include <vector>
#include "scene.h"
#include "lidar.h"
#include "kdtree.hpp"

// Function Device Control
#define GPU_MOTION		1
#define GPU_MEASUREMENT 1
#define GPU_MAP			1
#define GPU_RESAMPLE	1

void particleFilterInit(Scene *scene);
void particleFilterFree();
void particleFilter(uchar4 *pbo, int frame, Lidar *lidar);
void drawMap(uchar4 *pbo);
void checkCUDAErrorFn(const char *msg, const char *file, int line);
void getPCData(Particle **ptrParticles, MAP_TYPE **ptrMap, KDTree::Node **ptrKD, int *nParticles, int *nKD, glm::vec3 &pos);


void particleFilterInitPC();
void particleFilterFreePC();
void particleFilterPC(int frame, Lidar *lidar);

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

#define ERRORCHECK 1
#define FILENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define checkCUDAError(msg) checkCUDAErrorFn(msg, FILENAME, __LINE__)

void inline checkCUDAErrorFn(const char *msg, const char *file, int line) {
#if ERRORCHECK
	cudaDeviceSynchronize();
	cudaError_t err = cudaGetLastError();
	if (cudaSuccess == err) {
		return;
	}

	fprintf(stderr, "CUDA error");
	if (file) {
		fprintf(stderr, " (%s:%d)", file, line);
	}
	fprintf(stderr, ": %s: %s\n", msg, cudaGetErrorString(err));
#  ifdef _WIN32
	getchar();
#  endif
	exit(EXIT_FAILURE);
#endif
}