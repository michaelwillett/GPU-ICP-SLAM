#include <cstdio>
#include <cuda.h>
#include <cmath>
#include <chrono>
#include <random>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/remove.h>
#include <thrust/copy.h>
#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/partition.h>
#include <thrust/reduce.h>
#include <thrust/gather.h>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "sceneStructs.h"
#include "scene.h"
#include "svd3.h"
#include "kdtree.hpp"
#include "utilities.h"
#include "draw.h"
#include "kernel.h"


// Particle Filter Controls
#define PARTICLE_COUNT 1000
#define EFFECTIVE_PARTICLES .7
#define FREE_WEIGHT -1
#define OCCUPIED_WEIGHT 4
#define MAX_NODE_DIST 2.5f
#define MIN_NODE_DIST .5f
#define WALL_CONFIDENCE 30
#define MIN_WALL_COUNT  2
#define CLOSURE_MAP_DIST 6.0f
#define CLOSURE_GRAPH_DIST 20.0f

// Sensor Configuration
#define LIDAR_ANGLE(i) (-135.0f + i * .25f) * PI / 180
#define LIDAR_SIZE 1081
#define LIDAR_RANGE 20.0f
#define COV {0.015, 0.015, .01}

// GPU calculations
#define BLOCK_SIZE 128

// Helper Functions
#define CLAMP(a, lo, hi) (a < lo) ? lo : (a > hi) ? hi : a
#define ROUND_FRAC(a,frac) round((a/frac))*frac;


static Scene * hst_scene = NULL;
static glm::vec3 * dev_image = NULL;
static Patch * dev_maps = NULL;

// host variables
static MAP_TYPE *occupancyGrid = NULL;
static Particle particles[PARTICLE_COUNT];
static glm::ivec2 map_dim;
static Patch map_params;
static glm::vec3 robotPos;
static std::vector<Cluster> clusters;

// device variable
static MAP_TYPE *dev_occupancyGrid = NULL;
static Particle *dev_particles = NULL;
static int *dev_fit = NULL;
static float *dev_lidar = NULL;
static float *dev_weights = NULL;
static bool *dev_freeCells = NULL;
static bool *dev_wallCells = NULL;

// KD tree variables
#define KD_MAX_SIZE 10000000
static KDTree::Node *dev_kd = NULL;
static KDTree::Node kd[KD_MAX_SIZE];
static int kdSize = 0;
static float *dev_dist = NULL;
static int *dev_pair = NULL;
static float *dev_fitf = NULL;


/**
* Handy-dandy hash function that provides seeds for random number generation.
*/
__host__ __device__ unsigned int utilhash(unsigned int a) {
	a = (a + 0x7ed55d16) + (a << 12);
	a = (a ^ 0xc761c23c) ^ (a >> 19);
	a = (a + 0x165667b1) + (a << 5);
	a = (a + 0xd3a2646c) ^ (a << 9);
	a = (a + 0xfd7046c5) + (a << 3);
	a = (a ^ 0xb55a4f09) ^ (a >> 16);
	return a;
}

__host__ __device__ thrust::default_random_engine makeSeededRandomEngine(int iter, int index, int depth) {
	int h = utilhash((1 << 31) | (depth << 22) | iter) ^ utilhash(index);
	return thrust::default_random_engine(h);
}

// timers
float  avg_motion = 0.0f, avg_measurement = 0.0f, avg_map = 0.0f, avg_sample = 0.0f;

void particleFilterInit(Scene *scene) {
	hst_scene = scene;

	const Camera &cam = hst_scene->state.camera;
	const int pixelcount = cam.resolution.x * cam.resolution.y;

	cudaMalloc(&dev_image, pixelcount * sizeof(glm::vec3));
	cudaMemset(dev_image, 0, pixelcount * sizeof(glm::vec3));

	cudaMalloc(&dev_maps, scene->maps.size() * sizeof(Patch));
	cudaMemcpy(dev_maps, scene->maps.data(), scene->maps.size() * sizeof(Patch), cudaMemcpyHostToDevice);

	map_params = scene->maps[0];
	map_dim = glm::ivec2(map_params.scale.x / map_params.resolution.x, map_params.scale.y / map_params.resolution.y);

	occupancyGrid = new MAP_TYPE[map_dim.x*map_dim.y];

	memset(occupancyGrid, -100, map_dim.x*map_dim.y*sizeof(MAP_TYPE));

	for (int i = 0; i < PARTICLE_COUNT; i++) {
		particles[i].pos = glm::vec3(0.0f, 0.0f, 0.0f);
		particles[i].w = 1.0f;
		particles[i].cluster = 0;
	}

	robotPos = glm::vec3(0.0f);

	cudaMalloc((void**)&dev_occupancyGrid, map_dim.x*map_dim.y * sizeof(MAP_TYPE));
	cudaMemcpy(dev_occupancyGrid, occupancyGrid, map_dim.x*map_dim.y * sizeof(MAP_TYPE), cudaMemcpyHostToDevice);

	cudaMalloc((void**)&dev_particles, PARTICLE_COUNT * sizeof(Particle));
	cudaMemcpy(dev_particles, particles, PARTICLE_COUNT * sizeof(Particle), cudaMemcpyHostToDevice);

	cudaMalloc((void**)&dev_fit, PARTICLE_COUNT * sizeof(int));
	cudaMalloc((void**)&dev_weights, PARTICLE_COUNT * sizeof(float));
	cudaMalloc((void**)&dev_lidar, LIDAR_SIZE * sizeof(float));
	cudaMalloc((void**)&dev_freeCells, map_dim.x * map_dim.y * sizeof(bool));
	cudaMalloc((void**)&dev_wallCells, map_dim.x * map_dim.y * sizeof(bool));

	// initialize default cluster
	Cluster	group1;
	Node n0;
	n0.pos = glm::vec2(0.0f);
	n0.dist = 0.0f;
	group1.id = 0;
	group1.nodeIdx = 0;
	group1.patchList.push_back(0);
	group1.nodes.push_back(n0);
	std::vector<unsigned int> empty;
	group1.edges.push_back(empty);
	clusters.push_back(group1);

    checkCUDAError("particleFilterInit");
	particleFilterInitPC();
}

void particleFilterFree() {
    cudaFree(dev_image);  // no-op if dev_image is null
	cudaFree(dev_maps);
	cudaFree(dev_occupancyGrid);
	cudaFree(dev_particles);
	cudaFree(dev_lidar);
	cudaFree(dev_fit);
	cudaFree(dev_weights);
	cudaFree(dev_freeCells);
	cudaFree(dev_wallCells);

	delete occupancyGrid;
    checkCUDAError("particleFilterFree");

	particleFilterFreePC();
}


// rotates generates 2d point for lidar reading
__device__ __host__ void CleanLidarScan(int n, const float scan, const float theta, glm::vec2 &intersection) {
	float rot = LIDAR_ANGLE(n) + theta;
	
	intersection.x = scan * std::cos(rot);
	intersection.y = scan * std::sin(rot);
}

//Bresenham's line algorithm for integer grid
__device__ __host__ void traceRay(glm::ivec2 start, glm::ivec2 end, glm::ivec2 map_dim, bool *out){
	glm::ivec2 delta = end - start;

	// swap to the right octant
	bool steep = abs(delta.y) > abs(delta.x);
	if (steep) { // check slope
		int temp = start.x;
		start.x = start.y;
		start.y = temp;

		temp = end.x;
		end.x = end.y;
		end.y = temp;
	}

	if (start.x > end.x){
		int temp = start.x;
		start.x = end.x;
		end.x = temp;

		temp = start.y;
		start.y = end.y;
		end.y = temp;
	}

	int deltax = end.x - start.x;
	int deltay = abs(end.y - start.y);
	float error = deltax / 2;
	int y = start.y;
	int ystep = (end.y > start.y) ? 1 : -1;

	// build line
	for (int x = start.x; x < end.x; x++){
		int idx = 0;
		if (steep)
			idx  = y*map_dim.x + x;
		else
			idx = x*map_dim.x + y;

		if (x < map_dim.x && y < map_dim.y && x >= 0 && y >= 0 && idx < map_dim.x * map_dim.y) { // assume square maps
			out[idx] = 1;
		}
		
		error -= deltay;
		if (error < 0){
			y += ystep;
			error += deltax;
		}
	}

}

// sum the value of specified points in a 2d map
__device__ __host__ int mapCorrelation(int N, const MAP_TYPE *map, glm::ivec2 dim, const glm::vec2 *points)
{
	int retv = 0;

	for (int i = 0; i < N; i++) {
		if (points[i].x >= 0 && points[i].x < dim.x && points[i].y >= 0 && points[i].y < dim.y) {
			int idx = (int)points[i].x * dim.x + (int)points[i].y;
			retv += map[idx];
		}
	}

	return retv;
}

__device__ __host__ int EvaluateParticle(MAP_TYPE *map, glm::ivec2 map_dim, Patch map_params, Particle &particle, glm::vec3 pos, float *lidar)
{
	// get walls relative to robot position, add particle position
	glm::vec2 walls[LIDAR_SIZE];

	for (int j = 0; j < LIDAR_SIZE; j++) {
		CleanLidarScan(j, lidar[j], particle.pos.z, walls[j]);
		walls[j].x += particle.pos.x;
		walls[j].y += particle.pos.y;

		// convert to grid idx
		walls[j].x = round(0.5f * map_params.scale.x / map_params.resolution.x + walls[j].x / map_params.resolution.x);
		walls[j].y = round(0.5f * map_params.scale.y / map_params.resolution.y + walls[j].y / map_params.resolution.y);
	}

	// test the map correlation between global map and walls
	return mapCorrelation(LIDAR_SIZE, map, map_dim, walls);
}

// kernel wrapper for calling Evaluate Particle
__global__ void kernEvaluateParticles(MAP_TYPE *map, glm::ivec2 map_dim, Patch map_params, Particle *particles, glm::vec3 pos, float *lidar, int *fit)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < PARTICLE_COUNT) {
		fit[i] = EvaluateParticle(map, map_dim, map_params, particles[i], pos, lidar);
	}
}

// simple inplace multiplication kernel
__global__ void kernUpdateWeights(int N, Particle *a, int *b, float c, int min)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < N) {
		a[i].w = a[i].w * ((float)b[i] - min) * c;
	}
}

// simple inplace multiplication kernel
__global__ void kernUpdateWeights(int N, Particle *a, float *b, float c, int min)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < N) {
		a[i].w = a[i].w * (b[i] - min) * c;
	}
}

// update particle cloud weights from measurement
glm::vec3 PFMeasurementUpdate(std::vector<float> lidar) {  
	glm::vec3 retv(0.0f);
	if (GPU_MEASUREMENT) {
		// 1D block for particles
		const int blockSize1d = 128;
		const dim3 blocksPerGrid1d((PARTICLE_COUNT + blockSize1d - 1) / blockSize1d);

		// create device copy of fit array and lidar
		cudaMemcpy(dev_lidar, &lidar[0], LIDAR_SIZE * sizeof(float), cudaMemcpyHostToDevice);
		cudaMemset(dev_fit, 0, PARTICLE_COUNT * sizeof(int));
		cudaDeviceSynchronize();

		kernEvaluateParticles << <blocksPerGrid1d, blockSize1d >> >(dev_occupancyGrid, map_dim, map_params, dev_particles, robotPos, dev_lidar, dev_fit);
		cudaDeviceSynchronize();
		checkCUDAError("particle measurement update error");

		thrust::device_vector<int> vFit(dev_fit, dev_fit + PARTICLE_COUNT);
		thrust::pair<thrust::device_vector<int>::iterator, thrust::device_vector<int>::iterator> result = thrust::minmax_element(vFit.begin(), vFit.end());
		int rng = *result.second - *result.first;
		int best = result.second - vFit.begin();

		// rescale all weights
		if (rng > 0) {
			float f = 1 / (float)(rng);
			kernUpdateWeights << <blocksPerGrid1d, blockSize1d >> >(PARTICLE_COUNT, dev_particles, dev_fit, f, *result.first);
			cudaDeviceSynchronize();
			checkCUDAError("particle weight update error");
		}

		// only use best point for return
		cudaMemcpy(particles, dev_particles, PARTICLE_COUNT * sizeof(glm::vec4), cudaMemcpyDeviceToHost);
		retv = (glm::vec3) particles[best].pos;
	}
	else {
		int best = -128 * LIDAR_SIZE;
		int worst = 128 * LIDAR_SIZE;
		int iBest = 0;

		int fit[PARTICLE_COUNT] = { 0 };
		for (int i = 0; i < PARTICLE_COUNT; i++) {
			fit[i] = EvaluateParticle(occupancyGrid, map_dim, map_params, particles[i], robotPos, &lidar[0]);

			// track correlation maximums
			if (fit[i] > best) {
				best = fit[i];
				iBest = i;
			}

			if (fit[i] < worst)
				worst = fit[i];
		}

		// rescale all weights
		if ((best - worst) > 0) {
			float f = 1.0f;
			for (int i = 0; i < PARTICLE_COUNT; i++) {
				f = (float)(fit[i] - worst) / (float)(best - worst);
				particles[i].w *= f;
			}
		}

		retv = (glm::vec3) particles[iBest].pos;
	}

	return retv;
}

// add noise to a single particle
__device__ __host__ void ParticleAddNoise(Particle &particle, int frame, int idx)
{
	float mean[3] = { 0 };
	float cov[3] = COV;		// covariance: x y theta

	thrust::default_random_engine e2 = makeSeededRandomEngine(frame, idx, 0);
	thrust::random::normal_distribution<float> distx(mean[0], cov[0]);
	thrust::random::normal_distribution<float> disty(mean[1], cov[1]);
	thrust::random::normal_distribution<float> distt(mean[2], cov[2]);

	glm::vec3 noise(distx(e2), disty(e2), distt(e2));
	particle.pos += noise;
}

// kernel wrapper for adding noise to a particle
__global__ void kernAddNoise(Particle *particles, int frame)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < PARTICLE_COUNT) {
		ParticleAddNoise(particles[i], frame, i);
	}
}

// perform a motion update on the particle cloud, adding in gaussian noise
void PFMotionUpdate(int frame) {

	if (GPU_MOTION) {
		// 1D block for particles
		const int blockSize1d = 128;
		const dim3 blocksPerGrid1d((PARTICLE_COUNT + blockSize1d - 1) / blockSize1d);

		// sync up host and device arrays for now...
		cudaMemcpy(dev_particles, particles, PARTICLE_COUNT * sizeof(Particle), cudaMemcpyHostToDevice);
		kernAddNoise << <blocksPerGrid1d, blockSize1d >> >(dev_particles, frame);
		cudaMemcpy(particles, dev_particles, PARTICLE_COUNT * sizeof(Particle), cudaMemcpyDeviceToHost);
		cudaDeviceSynchronize(); 
		
		checkCUDAError("particle motion update error");
	} else {
		for (int i = 0; i < PARTICLE_COUNT; i++)
			ParticleAddNoise(particles[i], frame, i);
	}
}

__global__ void kernCopyWeights(Particle *particles, float *weights, bool squared)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < PARTICLE_COUNT) {
		weights[i] = (squared) ? particles[i].w * particles[i].w : particles[i].w;
	}
}

__global__ void kernWeightedSample(Particle *particles, float *weights, float max, float Neff, int frame)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < PARTICLE_COUNT) {
		thrust::default_random_engine gen = makeSeededRandomEngine(Neff, frame, i);
		thrust::random::uniform_real_distribution<float> dist(0, max);

		int idx = 0;
		float rnd = dist(gen);
		while (idx < PARTICLE_COUNT && rnd > weights[idx]) idx++;

		particles[i] = particles[idx];
		particles[i].w = 1.0f;
	}
}

// check if particles need to be resampled
void PFResample(int frame) {
	// 1D block for particles
	const int blockSize1d = 128;
	const dim3 blocksPerGrid1d((PARTICLE_COUNT + blockSize1d - 1) / blockSize1d);

	float r = 0, r2 = 0;

	if (GPU_RESAMPLE) {

		kernCopyWeights << <blocksPerGrid1d, blockSize1d >> >(dev_particles, dev_weights, true);
		cudaDeviceSynchronize();
		thrust::device_ptr<float> pWeights = thrust::device_pointer_cast(dev_weights);
		r2 = thrust::reduce(pWeights, pWeights + PARTICLE_COUNT);

		kernCopyWeights << <blocksPerGrid1d, blockSize1d >> >(dev_particles, dev_weights, false);
		cudaDeviceSynchronize();
		r = thrust::reduce(pWeights, pWeights + PARTICLE_COUNT);
	}
	else {
		for (int i = 0; i < PARTICLE_COUNT; i++) {
			r += particles[i].w;
			r2 += (particles[i].w) * (particles[i].w);
		}
	}

	float Neff = r * r / r2;

	if (Neff < EFFECTIVE_PARTICLES*PARTICLE_COUNT) {

		if (GPU_RESAMPLE) {
			thrust::device_ptr<float> pWeights = thrust::device_pointer_cast(dev_weights);
			thrust::inclusive_scan(pWeights, pWeights + PARTICLE_COUNT, pWeights);

			float max;
			cudaMemcpy(&max, &dev_weights[PARTICLE_COUNT - 1], sizeof(float), cudaMemcpyDeviceToHost);
			kernWeightedSample << <blocksPerGrid1d, blockSize1d >> >(dev_particles, dev_weights, max, Neff, frame);
			cudaMemcpy(particles, dev_particles, PARTICLE_COUNT * sizeof(Particle), cudaMemcpyDeviceToHost);

			checkCUDAError("resample error");
		}
		else {
			float weightsum[PARTICLE_COUNT];
			weightsum[0] = particles[0].w;
			for (int i = 1; i < PARTICLE_COUNT; i++) {
				weightsum[i] = weightsum[i - 1] + particles[i].w;
			}

			thrust::default_random_engine gen = makeSeededRandomEngine(Neff, frame, 0);
			thrust::random::uniform_real_distribution<float> dist(0, weightsum[PARTICLE_COUNT - 1]);

			for (int i = 0; i < PARTICLE_COUNT; i++) {
				int idx = 0;
				float rnd = dist(gen);
				while (idx < PARTICLE_COUNT && rnd > weightsum[idx]) idx++;

				particles[i] = particles[idx];
				particles[i].w = 1.0f;
			}

			// push particles to GPU to draw
			cudaMemcpy(dev_particles, particles, PARTICLE_COUNT * sizeof(Particle), cudaMemcpyHostToDevice);
		}
	} 

}

__global__ void kernUpdateMap(int N, MAP_TYPE *map, bool *mask, int val)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < N) {
		long clamp_val = (1 << (sizeof(MAP_TYPE)* 8 - 1)) - 15;
		if (mask[i])
			map[i] = CLAMP(map[i] + val, -clamp_val, clamp_val);
	}
}

__global__ void kernGetWalls(float *lidar, glm::ivec2 center, float theta, bool *freeCells, bool *wallCells, glm::ivec2 map_dim, Patch map_params)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < LIDAR_SIZE) {
		glm::vec2 walls;

		//ego centric scan
		CleanLidarScan(i, lidar[i], theta, walls);

		// this will discard random bad data from sensor that was causing overflow errors
		if (abs(walls.x) < LIDAR_RANGE && abs(walls.y) < LIDAR_RANGE) {
			walls.x = round(walls.x / map_params.resolution.x);
			walls.y = round(walls.y / map_params.resolution.y);

			// center to robot pos in current map
			walls += (glm::vec2) center;

			// from here we need to check the wall bounds, determine if it needs to update multiple maps, and create a new patch if necessary.
			traceRay(center, walls, map_dim, freeCells);
			if (walls.x >= 0 && walls.x < map_dim.x && walls.y >= 0 && walls.y < map_dim.y) {
				wallCells[(int)(walls.x * map_dim.x + walls.y)] = true;
			}
		}
	}
}

void PFUpdateMap(std::vector<float> lidar) {
	glm::ivec2 center_idx(
		round(0.5f * map_dim.x + robotPos.x / map_params.resolution.x + map_params.resolution.x / 2),
		round(0.5f * map_dim.y + robotPos.y / map_params.resolution.y + map_params.resolution.y / 2)
	);

	long clamp_val = (1 << (sizeof(MAP_TYPE)* 8 - 1)) - 15;

	if (GPU_MAP) {		

		// 1D block for particles
		const int blockSize1d = 128;
		const dim3 blocksPerGridLidar((LIDAR_SIZE + blockSize1d - 1) / blockSize1d);
		const dim3 blocksPerGridMap((map_dim.x * map_dim.y + blockSize1d - 1) / blockSize1d);

		// find occupancy grid cells from translated lidar
		cudaMemset(dev_freeCells, 0, map_dim.x * map_dim.y*sizeof(bool));
		cudaMemset(dev_wallCells, 0, map_dim.x * map_dim.y*sizeof(bool));
		cudaMemcpy(dev_lidar, &lidar[0], LIDAR_SIZE * sizeof(float), cudaMemcpyHostToDevice);

		// find intersections from lidar scan
		kernGetWalls << <blocksPerGridLidar, blockSize1d >> >(dev_lidar, center_idx, robotPos.z, dev_freeCells, dev_wallCells, map_dim, map_params);

		// Update free/occupied weights
		kernUpdateMap << <blocksPerGridMap, blockSize1d >> >(map_dim.x * map_dim.y, dev_occupancyGrid, dev_freeCells, FREE_WEIGHT);
		kernUpdateMap << <blocksPerGridMap, blockSize1d >> >(map_dim.x * map_dim.y, dev_occupancyGrid, dev_wallCells, OCCUPIED_WEIGHT);
	}
	else {
		// find occupancy grid cells from translated lidar
		bool *freeCells = new bool[map_dim.x * map_dim.y];
		memset(freeCells, 0, map_dim.x * map_dim.y*sizeof(bool));

		// find intersections from lidar scan
		glm::vec2 walls[LIDAR_SIZE];
		for (int i = 0; i < LIDAR_SIZE; i++) {
			CleanLidarScan(i, lidar[i], robotPos.z, walls[i]);
			walls[i].x = round(walls[i].x / map_params.resolution.x);
			walls[i].y = round(walls[i].y / map_params.resolution.y);
			walls[i] += center_idx;

			if (walls[i].x >= 0 && walls[i].x < map_dim.x && walls[i].y >= 0 && walls[i].y < map_dim.y) {
				traceRay(center_idx, walls[i], map_dim, freeCells);
			}
		}

		// downweight free cells
		for (int i = 0; i < map_dim.x; i++) {
			for (int j = 0; j < map_dim.y; j++) {
				int idx = i*map_dim.x + j;
				if (freeCells[idx]) {
					occupancyGrid[idx] += FREE_WEIGHT;
					occupancyGrid[idx] = CLAMP(occupancyGrid[idx], -clamp_val, clamp_val);
				}
			}
		}

		// upweight occupied cells
		for (int i = 0; i < LIDAR_SIZE; i++) {
			if (walls[i].x >= 0 && walls[i].x < map_dim.x && walls[i].y >= 0 && walls[i].y < map_dim.y) {
				int idx = (int)walls[i].x * map_dim.x + (int)walls[i].y;
				occupancyGrid[idx] += OCCUPIED_WEIGHT;
				occupancyGrid[idx] = CLAMP(occupancyGrid[idx], -clamp_val, clamp_val);
			}
		}

		// push grid to GPU to draw
		cudaMemcpy(dev_occupancyGrid, occupancyGrid, map_dim.x*map_dim.y * sizeof(char), cudaMemcpyHostToDevice);

		delete freeCells;
	}
}

void CreateNode(unsigned int i) {
	// create node at current position
	Node temp;
	temp.pos = (glm::vec2) robotPos;
	temp.dist = 0.0f;
	float edgeLen = glm::distance(temp.pos, clusters[i].nodes[clusters[i].nodeIdx].pos);

	// update distance for all current nodes
	for (int j = 0; j < clusters[i].nodes.size(); j++) {
		clusters[i].nodes[j].dist += edgeLen;
	}

	clusters[i].nodes.push_back(temp);

	// add edge from new node to last node
	std::vector<unsigned int> edge;
	edge.push_back(clusters[i].nodeIdx);
	clusters[i].edges.push_back(edge);

	// add edge from last node to new node
	clusters[i].edges[clusters[i].nodeIdx].push_back(clusters[i].nodes.size() - 1);

	// update current node
	clusters[i].nodeIdx = clusters[i].nodes.size() - 1;

}

// This returns true if it can see any walls
__global__ void CheckVisibility(int N, MAP_TYPE *map, bool *mask, unsigned int *retv)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < N) {
		if (mask[i] )
			*retv += (map[i] > WALL_CONFIDENCE) ? 1 : 0;
	}
}

int FindWalls(int clusterID, int nodeID) {
	unsigned int *dev_retv;
	bool *freeCells = new bool[map_dim.x * map_dim.y];
	cudaMalloc((void**)&dev_retv, sizeof(unsigned int));
	cudaMemset(dev_retv, 0, sizeof(unsigned int));
	memset(freeCells, 0, map_dim.x * map_dim.y*sizeof(bool));

	glm::ivec2 ai(
		round(0.5f * map_dim.x + robotPos.x / map_params.resolution.x + map_params.resolution.x / 2),
		round(0.5f * map_dim.y + robotPos.y / map_params.resolution.y + map_params.resolution.y / 2)
		);

	glm::ivec2 bi(
		round(0.5f * map_dim.x + clusters[clusterID].nodes[nodeID].pos.x / map_params.resolution.x + map_params.resolution.x / 2),
		round(0.5f * map_dim.y + clusters[clusterID].nodes[nodeID].pos.y / map_params.resolution.y + map_params.resolution.y / 2)
		);

	traceRay(ai, bi, map_dim, freeCells);
	cudaMemcpy(dev_freeCells, freeCells, map_dim.x * map_dim.y*sizeof(bool), cudaMemcpyHostToDevice);

	const int blockSize1d = 128;
	const dim3 blocksPerGridMap((map_dim.x * map_dim.y + blockSize1d - 1) / blockSize1d);

	CheckVisibility << <blocksPerGridMap, blockSize1d >> >(map_dim.x * map_dim.y, dev_occupancyGrid, dev_freeCells, dev_retv);
	cudaDeviceSynchronize();

	int nWalls;
	cudaMemcpy(&nWalls, dev_retv, sizeof(int), cudaMemcpyDeviceToHost);

	cudaFree(dev_retv);
	delete freeCells;

	return nWalls;
}

void UpdateTopology() {
	for (int i = 0; i < clusters.size(); i++) {
		bool newNode = true;
		// check if we need a new node on topology graph for each cluster (this is fast on CPU)
		// we could posibly improve performance here by not recalculating the distance every step, only checking relative to the distance the robot has moved.
		for (int j = 0; j < clusters[i].nodes.size(); j++)
			newNode &= (glm::distance((glm::vec2) robotPos, clusters[i].nodes[j].pos) > MAX_NODE_DIST);

		if (newNode) {
			CreateNode(i);
			printf("new node from distance. number of graph nodes: %i\n", clusters[i].nodes.size());
		}

		// if we don't need a new node for distance, check if we need one from visibility
		//if (!newNode) { // run this on GPU to prevent sending the maps back and forth, this operation can be slow even for a small graph
		//	newNode = true;

		//	// 1D block for particles
		//	for (int j = 0; j < clusters[i].nodes.size(); j++) {
		//		int nWalls = FindWalls(i, j);
		//		//if (nWalls > 0) printf("found %i walls for node %i\n", nWalls, j);
		//		newNode &= (nWalls >= MIN_WALL_COUNT);// && (glm::distance((glm::vec2) robotPos, clusters[i].nodes[j].pos) > MIN_NODE_DIST);
		//	}
		//	
		//	if (newNode) {
		//		CreateNode(i);
		//		printf("new node from visibility. number of graph nodes: %i\n", clusters[i].nodes.size());
		//	}
		//}
	}
}

__global__ void AssignParticlesToCluster(int N, Particle *particles) {

	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < N) {
		// find the closest visible node

	}
}

void CheckLoopClosure() {
	for (int i = 0; i < clusters.size(); i++) {
		for (int j = 0; j < clusters[i].nodes.size(); j++) { // check each node for loop closure conditions

			if (glm::distance((glm::vec2) robotPos, clusters[i].nodes[j].pos) < CLOSURE_MAP_DIST) {
				float edgeLen = glm::distance((glm::vec2) robotPos, clusters[i].nodes[clusters[i].nodeIdx].pos);

				if (edgeLen + clusters[i].nodes[j].dist > CLOSURE_GRAPH_DIST) {
					//printf("potential loop closure with node %i\n", j);

					// find all nodes that could separate clusters
					// 1D block for particles
					std::vector<int> visibleNodes;
					for (int k = 0; k < clusters[i].nodes.size(); k++) {
						int nWalls = FindWalls(i, k);

						if (nWalls < MIN_WALL_COUNT) {
							visibleNodes.push_back(k);
						}
					}

					// create new clusters for each group of visible nodes
					for (int k = 0; k < visibleNodes.size(); k++) {
						for (int l = 0; l < clusters.size(); l++) {
							std::vector<unsigned int> v = clusters[l].edges[visibleNodes[k]];		// only create new cluster if no clusters have an edge between visible and current nodes
							bool createCluster = (std::find(v.begin(), v.end(), clusters[i].nodeIdx) != v.end());

							if (createCluster) {		
								// copy cluster and get a new ID for it
								Cluster newCluster = clusters[i];
								newCluster.id = clusters.size();	// this will be wrong when we start deleting obsolete clusters

								// add edges
								newCluster.edges[clusters[i].nodeIdx].push_back(visibleNodes[k]);
								newCluster.edges[visibleNodes[k]].push_back(clusters[i].nodeIdx);

								// update graph distances for all nodes in cluster

								//clusters.push_back(newCluster);
							}

						}
					}
					

					// parse all particles into correct cluster
					//const int blockSize1d = 128;
					//const dim3 blocksPerGrid1d((PARTICLE_COUNT + blockSize1d - 1) / blockSize1d);
					//AssignParticlesToCluster << <blocksPerGrid1d, blockSize1d >> >(PARTICLE_COUNT, dev_particles);

					// prune unused clusters
					printf("now contains %i clusters\n", clusters.size());
				}

			}
		}
	}
}


void drawMap(uchar4 *pbo) {
	drawAll(pbo, PARTICLE_COUNT, hst_scene, dev_image, robotPos, dev_particles, dev_occupancyGrid, dev_maps, clusters);
	checkCUDAError("draw screen error");
}

void getPCData(Particle **ptrParticles, MAP_TYPE **ptrMap, KDTree::Node **ptrKD, int *nParticles, int *nKD, glm::vec3 &pos) {
	// copy map to host so PCL can draw it
	cudaMemcpy(occupancyGrid, dev_occupancyGrid, map_dim.x*map_dim.y * sizeof(MAP_TYPE), cudaMemcpyDeviceToHost);
	*ptrParticles = particles;
	*ptrMap = occupancyGrid;
	*nParticles = PARTICLE_COUNT;

	*ptrKD = kd;
	*nKD = kdSize;
	pos = robotPos;
}


/**
* Begin ICP code.
*/


__host__ __device__ bool sortFuncX(const glm::vec4 &p1, const glm::vec4 &p2)
{
	return p1.x < p2.x;
}
__host__ __device__ bool sortFuncY(const glm::vec4 &p1, const glm::vec4 &p2)
{
	return p1.y < p2.y;
}
__host__ __device__ bool sortFuncZ(const glm::vec4 &p1, const glm::vec4 &p2)
{
	return p1.z < p2.z;
}

__global__ void transformPoint(int N, glm::vec4 *points, glm::mat4 transform) {
	int index = threadIdx.x + (blockIdx.x * blockDim.x);
	if (index >= N) {
		return;
	}

	points[index] = glm::vec4(glm::vec3(transform * glm::vec4(glm::vec3(points[index]), 1)), 1);
}

__device__ float getHyperplaneDist(const glm::vec4 *pt1, const glm::vec4 *pt2, int axis, bool *branch)
{
	float retv = 0.0f;
	if (axis == 0) {
		*branch = sortFuncX(*pt1, *pt2);
		retv = abs(pt1->x - pt2->x);
	}
	if (axis == 1) {
		*branch = sortFuncY(*pt1, *pt2);
		retv = abs(pt1->y - pt2->y);
	}
	if (axis == 2) {
		*branch = sortFuncZ(*pt1, *pt2);
		retv = abs(pt1->z - pt2->z);
	}

	return retv;
}

__global__ void outerProduct(int N, const glm::vec4 *vec1, const glm::vec4 *vec2, glm::mat3 *out)
{
	int i = threadIdx.x + (blockIdx.x * blockDim.x);
	if (i >= N) {
		return;
	}

	out[i] = glm::mat3(glm::vec3(vec1[i]) * vec2[i].x,
		glm::vec3(vec1[i]) * vec2[i].y,
		glm::vec3(vec1[i] * vec2[i].z));
}

__global__ void findCorrespondenceKD(int N, glm::vec4 *cor, const glm::vec4 *points, const KDTree::Node* tree)
{
	int i = threadIdx.x + (blockIdx.x * blockDim.x);
	if (i >= N) {
		return;
	}

	glm::vec4 pt = points[i];
	float bestDist = glm::distance(glm::vec3(pt), glm::vec3(tree[0].value));
	int bestIdx = 0;
	int head = 0;
	bool done = false;
	bool branch = false;
	bool nodeFullyExplored = false;

	while (!done) {
		// depth first on current branch
		while (head >= 0) {
			// check the current node
			const KDTree::Node test = tree[head];
			float d = glm::distance(glm::vec3(pt), glm::vec3(test.value));
			if (d < bestDist) {
				bestDist = d;
				bestIdx = head;
				nodeFullyExplored = false;
			}

			// find branch path
			getHyperplaneDist(&pt, &test.value, test.axis, &branch);
			head = branch ? test.left : test.right;
		}

		if (nodeFullyExplored) {
			done = true;
		}
		else {
			// check if parent of best node could have better values on other branch
			const KDTree::Node parent = tree[tree[bestIdx].parent];
			if (getHyperplaneDist(&pt, &parent.value, parent.axis, &branch) < bestDist) {
				head = !branch ? parent.left : parent.right;
				nodeFullyExplored = true;
			}
			else
				done = true;
		}
	}

	cor[i] = tree[bestIdx].value;
}

__global__ void findCorrespondenceIndexKD(int N, int *cor, const glm::vec4 *points, const KDTree::Node* tree)
{
	int i = threadIdx.x + (blockIdx.x * blockDim.x);
	if (i >= N) {
		return;
	}

	glm::vec4 pt = points[i];
	float bestDist = glm::distance(glm::vec3(pt), glm::vec3(tree[0].value));
	int bestIdx = 0;
	int head = 0;
	bool done = false;
	bool branch = false;
	bool nodeFullyExplored = false;

	while (!done) {
		// depth first on current branch
		while (head >= 0) {
			// check the current node
			const KDTree::Node test = tree[head];
			float d = glm::distance(glm::vec3(pt), glm::vec3(test.value));
			if (d < bestDist) {
				bestDist = d;
				bestIdx = head;
				nodeFullyExplored = false;
			}

			// find branch path
			getHyperplaneDist(&pt, &test.value, test.axis, &branch);
			head = branch ? test.left : test.right;
		}

		if (nodeFullyExplored) {
			done = true;
		}
		else {
			// check if parent of best node could have better values on other branch
			const KDTree::Node parent = tree[tree[bestIdx].parent];
			if (getHyperplaneDist(&pt, &parent.value, parent.axis, &branch) < bestDist) {
				head = !branch ? parent.left : parent.right;
				nodeFullyExplored = true;
			}
			else
				done = true;
		}
	}

	cor[i] = bestIdx;
}

__global__ void kernGetWallsKD(float *lidar, glm::vec3 robotPos, glm::vec4 *nodeVal, Patch map_params)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < LIDAR_SIZE) {
		//ego centric scan
		glm::vec2 walls;
		CleanLidarScan(i, lidar[i], robotPos.z, walls);

		// this will discard random bad data from sensor that was causing overflow errors
		if (abs(walls.x) < LIDAR_RANGE && abs(walls.y) < LIDAR_RANGE) {
			nodeVal[i].x = robotPos.x + walls.x, map_params.resolution.x;
			nodeVal[i].y = robotPos.y + walls.y, map_params.resolution.y;
			nodeVal[i].z = 0.0f;
			nodeVal[i].w = OCCUPIED_WEIGHT;
		}
	}
}

glm::vec3 transformPointICP(glm::vec3 start, std::vector<float> lidar) {
	int sizeTarget = LIDAR_SIZE;
	glm::vec3 retv = start;
	
	dim3 fullBlocksPerGrid((LIDAR_SIZE + BLOCK_SIZE - 1) / BLOCK_SIZE);
	// find the closest point in the scene for each point in the target

	glm::vec4 *dev_cor, *tar_c, *cor_c, *dev_target;
	glm::mat3 *dev_W;

	cudaMalloc((void**)&dev_cor, sizeTarget*sizeof(glm::vec4));
	cudaMalloc((void**)&tar_c, sizeTarget*sizeof(glm::vec4));
	cudaMalloc((void**)&cor_c, sizeTarget*sizeof(glm::vec4));
	cudaMalloc((void**)&dev_target, sizeTarget*sizeof(glm::vec4));
	cudaMalloc((void**)&dev_W, sizeTarget * sizeof(glm::mat3));
	cudaMemset(dev_W, 0, sizeTarget * sizeof(glm::mat3));

	// find intersections from lidar scan
	cudaMemcpy(dev_lidar, &lidar[0], LIDAR_SIZE * sizeof(float), cudaMemcpyHostToDevice);
	//kernGetWalls << <fullBlocksPerGrid, BLOCK_SIZE >> >(dev_lidar, center_idx, robotPos.z, dev_freeCells, dev_wallCells, map_dim, map_params);
	kernGetWallsKD << <fullBlocksPerGrid, BLOCK_SIZE >> >(dev_lidar, robotPos, dev_target, map_params);

	findCorrespondenceKD << <fullBlocksPerGrid, BLOCK_SIZE >> >(sizeTarget, dev_cor, dev_target, dev_kd);
	cudaThreadSynchronize();

	// Calculate mean centered correspondenses 
	glm::vec3 mu_tar(0, 0, 0), mu_cor(0, 0, 0);

	thrust::device_ptr<glm::vec4> ptr_target(dev_target);
	thrust::device_ptr<glm::vec4> ptr_scene(dev_target);
	thrust::device_ptr<glm::vec4> ptr_cor(dev_cor);

	mu_tar = glm::vec3(thrust::reduce(ptr_target, ptr_target + sizeTarget, glm::vec4(0, 0, 0, 0)));
	mu_cor = glm::vec3(thrust::reduce(ptr_cor, ptr_cor + sizeTarget, glm::vec4(0, 0, 0, 0)));

	mu_tar /= sizeTarget;
	mu_cor /= sizeTarget;

	cudaMemcpy(tar_c, dev_target, sizeTarget*sizeof(glm::vec4), cudaMemcpyDeviceToDevice);
	cudaMemcpy(cor_c, dev_cor, sizeTarget*sizeof(glm::vec4), cudaMemcpyDeviceToDevice);
	checkCUDAError("mean centered calculation failed!");

	// move the point cloud with translation
	glm::vec3 r(0, 0, 0);
	glm::vec3 s(1, 1, 1);
	glm::mat4 center_tar = utilityCore::buildTransformationMatrix(-mu_tar, r, s);
	glm::mat4 center_cor = utilityCore::buildTransformationMatrix(-mu_cor, r, s);

	transformPoint << <fullBlocksPerGrid, BLOCK_SIZE >> >(sizeTarget, tar_c, center_tar);
	transformPoint << <fullBlocksPerGrid, BLOCK_SIZE >> >(sizeTarget, cor_c, center_cor);

	checkCUDAError("mean centered transformation failed!");
	cudaThreadSynchronize();

	// Calculate W
	outerProduct << <fullBlocksPerGrid, BLOCK_SIZE >> >(sizeTarget, tar_c, cor_c, dev_W);
	thrust::device_ptr<glm::mat3> ptr_W(dev_W);
	glm::mat3 W = thrust::reduce(ptr_W, ptr_W + sizeTarget, glm::mat3(0));

	checkCUDAError("outer product failed!");
	cudaThreadSynchronize();

	// calculate SVD of W
	glm::mat3 U, S, V;

	svd(W[0][0], W[0][1], W[0][2], W[1][0], W[1][1], W[1][2], W[2][0], W[2][1], W[2][2],
		U[0][0], U[0][1], U[0][2], U[1][0], U[1][1], U[1][2], U[2][0], U[2][1], U[2][2],
		S[0][0], S[0][1], S[0][2], S[1][0], S[1][1], S[1][2], S[2][0], S[2][1], S[2][2],
		V[0][0], V[0][1], V[0][2], V[1][0], V[1][1], V[1][2], V[2][0], V[2][1], V[2][2]
		);


	glm::mat3 g_U(glm::vec3(U[0][0], U[1][0], U[2][0]), glm::vec3(U[0][1], U[1][1], U[2][1]), glm::vec3(U[0][2], U[1][2], U[2][2]));
	glm::mat3 g_Vt(glm::vec3(V[0][0], V[0][1], V[0][2]), glm::vec3(V[1][0], V[1][1], V[1][2]), glm::vec3(V[2][0], V[2][1], V[2][2]));

	// Get transformation from SVD
	glm::mat3 R = g_U * g_Vt;
	glm::vec3 t = glm::vec3(mu_cor) - R*glm::vec3(mu_tar);

	// update target points
	//glm::mat4 transform = glm::translate(glm::mat4(), t) * glm::mat4(R);
	//transformPoint << <fullBlocksPerGrid, BLOCK_SIZE >> >(sizeTarget, dev_target, transform);

	// make a massive assumption that the SVD will already result in a 2d rotation around the z-axis
	//glm::vec4 newPoint(start.x, start.y, 0.0f, 0.0f);
	//newPoint = transform*newPoint;
	float theta = asin(R[0][1]);


	retv.x += t.x;
	retv.y += t.y;
	retv.z += theta;

	cudaFree(dev_cor);
	cudaFree(tar_c);
	cudaFree(cor_c);
	cudaFree(dev_W);
	cudaFree(dev_target);

	return retv;
}


void particleFilterInitPC() {
	// KD tree data
	cudaMalloc((void**)&dev_dist, LIDAR_SIZE * sizeof(float));
	checkCUDAError("cudaMalloc dev_dist failed!");

	cudaMalloc((void**)&dev_pair, LIDAR_SIZE * sizeof(int));
	checkCUDAError("cudaMalloc dev_pair failed!");

	cudaMalloc((void**)&dev_kd, KD_MAX_SIZE * sizeof(KDTree::Node));
	checkCUDAError("cudaMalloc dev_kd failed!");

	cudaMalloc((void**)&dev_fitf, PARTICLE_COUNT * sizeof(float));
	checkCUDAError("cudaMalloc dev_fitf failed!");


	cudaThreadSynchronize();
	checkCUDAError("particleFilterInitPC");
}

void particleFilterFreePC() {
	cudaFree(dev_dist);
	cudaFree(dev_pair);
	cudaFree(dev_kd);
	cudaFree(dev_fitf);

	checkCUDAError("particleFilterFreePC");
}

__device__ float weight[LIDAR_SIZE];
__device__ float particlePos[3];
__device__ float atmWeight[PARTICLE_COUNT*LIDAR_SIZE];



__global__ void kernFindWallCorrespondance(float *lidar, KDTree::Node *kdTree, int nParticle) {
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < LIDAR_SIZE) {
		glm::vec2 walls;
		glm::vec4 pt;
		CleanLidarScan(i, lidar[i], particlePos[2], walls);

		if (abs(walls.x) < LIDAR_RANGE && abs(walls.y) < LIDAR_RANGE) {
			walls.x += particlePos[0];
			walls.y += particlePos[1];

			pt.x = walls.x;
			pt.y = walls.y;
			pt.z = 0.0f;
			pt.w = 0.0f;

			float bestDist = glm::distance(glm::vec3(pt), glm::vec3(kdTree[0].value));
			int bestIdx = 0;
			int head = 0;
			bool done = false;
			bool branch = false;
			bool nodeFullyExplored = false;

			while (!done) {
				// depth first on current branch
				while (head >= 0) {
					// check the current node
					const KDTree::Node test = kdTree[head];
					float d = glm::distance(glm::vec3(pt), glm::vec3(test.value));
					if (d < bestDist) {
						bestDist = d;
						bestIdx = head;
						nodeFullyExplored = false;
					}

					// find branch path
					getHyperplaneDist(&pt, &test.value, test.axis, &branch);
					head = branch ? test.left : test.right;
				}

				if (nodeFullyExplored) {
					done = true;
				}
				else {
					// check if parent of best node could have better values on other branch
					const KDTree::Node parent = kdTree[kdTree[bestIdx].parent];
					if (getHyperplaneDist(&pt, &parent.value, parent.axis, &branch) < bestDist) {
						head = !branch ? parent.left : parent.right;
						nodeFullyExplored = true;
					}
					else
						done = true;
				}
			}

			atmWeight[LIDAR_SIZE*nParticle + i] = kdTree[bestIdx].value.w;

			//atomicAdd(&atmWeight[nParticle], kdTree[bestIdx].value.w);
		}
		else {
			atmWeight[LIDAR_SIZE*nParticle + i] = 0;
		}
	}
}

#define DYNAMIC_KERN 0

__device__ float EvaluateParticleKD(MAP_TYPE *map, glm::ivec2 map_dim, Patch map_params, Particle &particle, glm::vec3 pos, float *lidar, KDTree::Node *kdTree, int kdSize, int nParticle)
{
	// get walls relative to robot position, add particle position
	glm::vec2 walls;
	glm::vec4 pt;
	
	int nValid = 0;
	glm::vec3 mu_tar(0.0f);
	glm::vec3 mu_cor(0.0f);
	float retv = 0.0f;

#if (DYNAMIC_KERN == 1)
	// try launching dynamic kernel
	// 1D block for LIDAR
	const int blockSize1d = 128;
	const dim3 blocksPerGrid1d((LIDAR_SIZE + blockSize1d - 1) / blockSize1d);

	particlePos[0] = particle.pos.x;
	particlePos[1] = particle.pos.y;
	particlePos[2] = particle.pos.z;
	atmWeight[nParticle] = 0.0f;

	kernFindWallCorrespondance << <blocksPerGrid1d, blockSize1d >> >(lidar, kdTree, nParticle);
	__syncthreads();
#endif

	// get walls and find correspondence
	for (int j = 0; j < LIDAR_SIZE; j++) {

#if (DYNAMIC_KERN == 0)
		CleanLidarScan(j, lidar[j], particle.pos.z, walls);

		if (abs(walls.x) < LIDAR_RANGE && abs(walls.y) < LIDAR_RANGE) {
			walls.x += particle.pos.x;
			walls.y += particle.pos.y;

			pt.x = walls.x;
			pt.y = walls.y;
			pt.z = 0.0f;
			pt.w = 0.0f;

			float bestDist = glm::distance(glm::vec3(pt), glm::vec3(kdTree[0].value));
			int bestIdx = 0;
			int head = 0;
			bool done = false;
			bool branch = false;
			bool nodeFullyExplored = false;

			while (!done) {
				// depth first on current branch
				while (head >= 0) {
					// check the current node
					const KDTree::Node test = kdTree[head];
					float d = glm::distance(glm::vec3(pt), glm::vec3(test.value));
					if (d < bestDist) {
						bestDist = d;
						bestIdx = head;
						nodeFullyExplored = false;
					}

					// find branch path
					getHyperplaneDist(&pt, &test.value, test.axis, &branch);
					head = branch ? test.left : test.right;
				}

				if (nodeFullyExplored) {
					done = true;
				}
				else {
					// check if parent of best node could have better values on other branch
					const KDTree::Node parent = kdTree[kdTree[bestIdx].parent];
					if (getHyperplaneDist(&pt, &parent.value, parent.axis, &branch) < bestDist) {
						head = !branch ? parent.left : parent.right;
						nodeFullyExplored = true;
					}
					else
						done = true;
				}
			}

			mu_tar += (glm::vec3) pt;
			mu_cor += (glm::vec3) kdTree[bestIdx].value;

			float minDist = sqrt(map_params.resolution.x*map_params.resolution.x + map_params.resolution.y*map_params.resolution.y) * 2;

			//if (glm::distance((glm::vec3) pt, (glm::vec3) kdTree[bestIdx].value) < minDist) {
				retv += kdTree[bestIdx].value.w;
		//}
		} 
#else
		retv += atmWeight[LIDAR_SIZE*nParticle + j];
#endif
	}

	//printf("matches found: %i %.4f\n", nValid, retv);

	//mu_tar /= nValid;
	//mu_cor /= nValid;

	return retv;
}

// kernel wrapper for calling Evaluate Particle
__global__ void kernEvaluateParticlesKD(MAP_TYPE *map, glm::ivec2 map_dim, Patch map_params, Particle *particles, glm::vec3 pos, float *lidar, float *fit, KDTree::Node *kdTree, int kdSize)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < PARTICLE_COUNT) {
		fit[i] = EvaluateParticleKD(map, map_dim, map_params, particles[i], pos, lidar, kdTree, kdSize, i);
	}
}

// update particle cloud weights from measurement
glm::vec3 PFMeasurementUpdateKD(std::vector<float> lidar) {
	glm::vec3 retv(0.0f);

	// 1D block for particles
	const int blockSize1d = 128;
	const dim3 blocksPerGrid1d((PARTICLE_COUNT + blockSize1d - 1) / blockSize1d);

	// create device copy of fit array and lidar
	cudaMemcpy(dev_lidar, &lidar[0], LIDAR_SIZE * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemset(dev_fitf, 0, PARTICLE_COUNT * sizeof(float));
	cudaDeviceSynchronize();

	kernEvaluateParticlesKD << <blocksPerGrid1d, blockSize1d >> >(dev_occupancyGrid, map_dim, map_params, dev_particles, robotPos, dev_lidar, dev_fitf, dev_kd, kdSize);
	cudaDeviceSynchronize();
	checkCUDAError("particle measurement kd tree update error");

	thrust::device_vector<float> vFit(dev_fitf, dev_fitf + PARTICLE_COUNT);
	thrust::pair<thrust::device_vector<float>::iterator, thrust::device_vector<float>::iterator> result = thrust::minmax_element(vFit.begin(), vFit.end());
	float rng = *result.second - *result.first;
	int best = result.second - vFit.begin();

	// rescale all weights
	if (rng > 0.0f) {
		float f = 1 / rng;
		kernUpdateWeights << <blocksPerGrid1d, blockSize1d >> >(PARTICLE_COUNT, dev_particles, dev_fitf, f, *result.first);
		cudaDeviceSynchronize();
		checkCUDAError("particle weight kdtree update error");
	}

	// only use best point for return
	cudaMemcpy(particles, dev_particles, PARTICLE_COUNT * sizeof(glm::vec4), cudaMemcpyDeviceToHost);
	//retv = (glm::vec3) particles[best].pos;

	// run ICP on final point only
	retv = transformPointICP((glm::vec3) particles[best].pos, lidar);

	return retv;
}

__global__ void kernUpdateMapKD(int N, KDTree::Node* tree, glm::vec4 *target, int *indexList, int val, Patch map_params)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < N) {
		long clamp_val = (1 << (sizeof(MAP_TYPE) * 8 - 1)) - 15;
		glm::vec3 a = (glm::vec3) target[i];
		glm::vec3 b = (glm::vec3) tree[indexList[i]].value;
		float minDist = sqrt(map_params.resolution.x*map_params.resolution.x + map_params.resolution.y*map_params.resolution.y);

		if (glm::distance(a, b) < minDist) {
			tree[indexList[i]].value.w = CLAMP(tree[indexList[i]].value.w + val, -clamp_val, clamp_val);
		}
	}
}


__global__ void kernTestCorrespondance(int N, KDTree::Node* tree, glm::vec4 *target, int *indexList, bool *diff, Patch map_params)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < N) {
		glm::vec3 a = (glm::vec3) target[i];
		glm::vec3 b = (glm::vec3) tree[indexList[i]].value;
		float minDist = sqrt(map_params.resolution.x*map_params.resolution.x + map_params.resolution.y*map_params.resolution.y) / 2.0f;


		diff[i] = (glm::distance(a, b) > minDist);
	}
}


__global__ void kernGeneratePosArray(glm::vec4 *out, glm::vec3 pos, glm::ivec2 dim, Patch params)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < dim.x*dim.y) {
		int x = i / dim.x;
		int y = i % dim.x;
		out[i].x = x * params.resolution.x - params.scale.x / 2.0f + pos.x;
		out[i].y = y * params.resolution.y - params.scale.y / 2.0f + pos.y;
		out[i].x = ROUND_FRAC(out[i].x, params.resolution.x);
		out[i].y = ROUND_FRAC(out[i].y, params.resolution.y);
		out[i].z = 0.0f;
	}
}

struct is_true
{
	__host__ __device__
		bool operator()(const bool x)
	{
		return x == true;
	}
};

void PFUpdateMapKD(std::vector<float> lidar) {
	// get local free and occupied grid cells here. Use these values to update pointcloud
	glm::ivec2 center_idx(
		round(0.5f * map_dim.x + map_params.resolution.x / 2),
		round(0.5f * map_dim.y + map_params.resolution.y / 2)
		);
	
	// 1D block for particles
	const int blockSize1d = 128;
	const dim3 blocksPerGridLidar((LIDAR_SIZE + blockSize1d - 1) / blockSize1d);

	// find occupancy grid cells from translated lidar
	cudaMemset(dev_freeCells, 0, map_dim.x * map_dim.y*sizeof(bool));
	cudaMemset(dev_wallCells, 0, map_dim.x * map_dim.y*sizeof(bool));
	cudaMemcpy(dev_lidar, &lidar[0], LIDAR_SIZE * sizeof(float), cudaMemcpyHostToDevice);

	// find intersections from lidar scan
	kernGetWalls << <blocksPerGridLidar, blockSize1d >> >(dev_lidar, center_idx, robotPos.z, dev_freeCells, dev_wallCells, map_dim, map_params);

	bool *wallCells = new bool[map_dim.x * map_dim.y];
	bool *freeCells = new bool[map_dim.x * map_dim.y];
	std::vector<glm::vec4> wallPC;
	std::vector<glm::vec4> freePC;

	cudaMemcpy(wallCells, dev_wallCells, map_dim.x * map_dim.y * sizeof(bool), cudaMemcpyDeviceToHost);
	cudaMemcpy(freeCells, dev_freeCells, map_dim.x * map_dim.y * sizeof(bool), cudaMemcpyDeviceToHost);
	
	// Create Pointclouds here
	// parallelize through compactions and summation
	for (int x = 0; x < map_dim.x; x++) {
		for (int y = 0; y < map_dim.y; y++) {
			int idx = (x * map_dim.x) + y;

			if (wallCells[idx]) {
				glm::vec4 point;

				point.x = x * map_params.resolution.x - map_params.scale.x / 2.0f + robotPos.x;
				point.y = y * map_params.resolution.y - map_params.scale.y / 2.0f + robotPos.y;
				point.x = ROUND_FRAC(point.x, map_params.resolution.x);
				point.y = ROUND_FRAC(point.y, map_params.resolution.y);
				point.z = 0.0f;
				wallPC.push_back(point);
			}

			if (freeCells[idx]) {
				glm::vec4 point;

				point.x = x * map_params.resolution.x - map_params.scale.x / 2.0f + robotPos.x;
				point.y = y * map_params.resolution.y - map_params.scale.y / 2.0f + robotPos.y;
				point.x = ROUND_FRAC(point.x, map_params.resolution.x);
				point.y = ROUND_FRAC(point.y, map_params.resolution.y);
				point.z = 0.0f;
				freePC.push_back(point);
			}
		}
	}

	if (kdSize > 0) {
		// downweight existing wall cells if in freeCells
		const dim3 blocksPerGridFree((freePC.size() + blockSize1d - 1) / blockSize1d);
		const dim3 blocksPerGridWall((wallPC.size() + blockSize1d - 1) / blockSize1d);

		glm::vec4 *dev_walls, *dev_free;
		int *dev_walls_c, *dev_free_c;
		cudaMalloc((void**)&dev_walls, wallPC.size()*sizeof(glm::vec4));
		cudaMalloc((void**)&dev_walls_c, wallPC.size()*sizeof(int));
		cudaMalloc((void**)&dev_free, freePC.size()*sizeof(glm::vec4));
		cudaMalloc((void**)&dev_free_c, freePC.size()*sizeof(int));

		cudaMemcpy(dev_free, &freePC[0], wallPC.size()*sizeof(glm::vec4), cudaMemcpyHostToDevice);
		cudaMemcpy(dev_walls, &wallPC[0], wallPC.size()*sizeof(glm::vec4), cudaMemcpyHostToDevice);

		findCorrespondenceIndexKD << <blocksPerGridFree, BLOCK_SIZE >> >(freePC.size(), dev_free_c, dev_free, dev_kd);
		findCorrespondenceIndexKD << <blocksPerGridWall, BLOCK_SIZE >> >(wallPC.size(), dev_walls_c, dev_walls, dev_kd);

		cudaDeviceSynchronize();
		checkCUDAError("map update - correspondance failure");

		bool *wallsCreate = new bool[wallPC.size()];
		bool *dev_wallsCreate = NULL;
		cudaMalloc((void**)&dev_wallsCreate, wallPC.size()*sizeof(bool));

		//cudaMemcpy(free_c, &freePC[0], freePC.size()*sizeof(int), cudaMemcpyHostToDevice);
		//cudaMemcpy(walls_c, &wallPC[0], wallPC.size()*sizeof(int), cudaMemcpyHostToDevice);

		// downweight free cells
		kernUpdateMapKD << <blocksPerGridFree, BLOCK_SIZE >> >(freePC.size(), dev_kd, dev_free, dev_free_c, FREE_WEIGHT, map_params);

		// upweight existing wall cells
		kernUpdateMapKD << <blocksPerGridWall, BLOCK_SIZE >> >(wallPC.size(), dev_kd, dev_walls, dev_walls_c, OCCUPIED_WEIGHT, map_params);
		cudaDeviceSynchronize();

		// insert any new wall cells
		kernTestCorrespondance << <blocksPerGridWall, BLOCK_SIZE >> >(wallPC.size(), dev_kd, dev_walls, dev_walls_c, dev_wallsCreate, map_params);

		cudaMemcpy(wallsCreate, dev_wallsCreate, wallPC.size()*sizeof(bool), cudaMemcpyDeviceToHost);
		cudaDeviceSynchronize();
		
		int nInsert = 0; 
		for (int i = 0; i < wallPC.size(); i++) {
			if (wallsCreate[i]) nInsert++;
		}

		// we dont want to recreate the kd tree every time, we just want to maintain copies on both the host and device...
		if (nInsert > 0) {
			cudaMemcpy(kd, dev_kd, kdSize*sizeof(KDTree::Node), cudaMemcpyDeviceToHost); // make sure to copy new weight values
			for (int i = 0; i < wallPC.size(); i++) {
				if (wallsCreate[i]) {
					wallPC[i].w = -100;
					KDTree::InsertNode(wallPC[i], kd, kdSize++);
				}
			}
			//printf("new pointcloud size: %i\n", kdSize);
			cudaMemcpy(dev_kd, kd, kdSize*sizeof(KDTree::Node), cudaMemcpyHostToDevice);
		}

		checkCUDAError("map update - insert failure");

		cudaFree(dev_walls);
		cudaFree(dev_walls_c);
		cudaFree(dev_free);
		cudaFree(dev_free_c);
		cudaFree(dev_wallsCreate);

		delete wallsCreate;

	} else { // create a new kd tree from first scan
		KDTree::Create(wallPC, kd);
		cudaMemcpy(dev_kd, kd, wallPC.size()*sizeof(KDTree::Node), cudaMemcpyHostToDevice);
		kdSize += wallPC.size();
	}

	delete[] wallCells;
	delete[] freeCells;
}

void PFUpdateMapKD_SLOW(std::vector<float> lidar) {
	// get local free and occupied grid cells here. Use these values to update pointcloud
	glm::ivec2 center_idx(
		round(0.5f * map_dim.x + map_params.resolution.x / 2),
		round(0.5f * map_dim.y + map_params.resolution.y / 2)
		);
	
	// 1D block for particles
	const int blockSize1d = 128;
	const dim3 blocksPerGridLidar((LIDAR_SIZE + blockSize1d - 1) / blockSize1d);

	// find occupancy grid cells from translated lidar
	cudaMemset(dev_freeCells, 0, map_dim.x * map_dim.y*sizeof(bool));
	cudaMemset(dev_wallCells, 0, map_dim.x * map_dim.y*sizeof(bool));
	cudaMemcpy(dev_lidar, &lidar[0], LIDAR_SIZE * sizeof(float), cudaMemcpyHostToDevice);

	// find intersections from lidar scan
	kernGetWalls << <blocksPerGridLidar, blockSize1d >> >(dev_lidar, center_idx, robotPos.z, dev_freeCells, dev_wallCells, map_dim, map_params);

	//// create index list
	//glm::vec4 *dev_pos, *dev_walls_pc, *dev_free_pc;
	//cudaMalloc((void**)&dev_pos, map_dim.x*map_dim.y*sizeof(glm::vec4));
	//cudaMalloc((void**)&dev_walls_pc, map_dim.x*map_dim.y*sizeof(glm::vec4));
	//cudaMalloc((void**)&dev_free_pc, map_dim.x*map_dim.y*sizeof(glm::vec4));
	//const dim3 blocksPerGridMap((map_dim.x*map_dim.y + blockSize1d - 1) / blockSize1d);
	//kernGeneratePosArray << < blocksPerGridMap, blockSize1d >> >(dev_pos, robotPos, map_dim, map_params);

	//thrust::device_ptr<glm::vec4> ptr_pos = thrust::device_pointer_cast(dev_pos);
	//int nWalls = thrust::copy_if(thrust::device, ptr_pos, ptr_pos + map_dim.x*map_dim.y, dev_wallCells, dev_walls_pc, is_true()) - dev_walls_pc;
	//int nFree = thrust::copy_if(thrust::device, ptr_pos, ptr_pos + map_dim.x*map_dim.y, dev_freeCells, dev_free_pc, is_true()) - dev_free_pc;

	bool *wallCells = new bool[map_dim.x * map_dim.y];
	bool *freeCells = new bool[map_dim.x * map_dim.y];
	std::vector<glm::vec4> wallPC;
	std::vector<glm::vec4> freePC;

	cudaMemcpy(wallCells, dev_wallCells, map_dim.x * map_dim.y * sizeof(bool), cudaMemcpyDeviceToHost);
	cudaMemcpy(freeCells, dev_freeCells, map_dim.x * map_dim.y * sizeof(bool), cudaMemcpyDeviceToHost);

	// Create Pointclouds here
	// parallelize through compactions and summation
	for (int x = 0; x < map_dim.x; x++) {
		for (int y = 0; y < map_dim.y; y++) {
			int idx = (x * map_dim.x) + y;

			if (wallCells[idx]) {
				glm::vec4 point;

				point.x = x * map_params.resolution.x - map_params.scale.x / 2.0f + robotPos.x;
				point.y = y * map_params.resolution.y - map_params.scale.y / 2.0f + robotPos.y;
				point.x = ROUND_FRAC(point.x, map_params.resolution.x);
				point.y = ROUND_FRAC(point.y, map_params.resolution.y);
				point.z = 0.0f;
				wallPC.push_back(point);
			}

			if (freeCells[idx]) {
				glm::vec4 point;

				point.x = x * map_params.resolution.x - map_params.scale.x / 2.0f + robotPos.x;
				point.y = y * map_params.resolution.y - map_params.scale.y / 2.0f + robotPos.y;
				point.x = ROUND_FRAC(point.x, map_params.resolution.x);
				point.y = ROUND_FRAC(point.y, map_params.resolution.y);
				point.z = 0.0f;
				freePC.push_back(point);
			}
		}
	}
	int nFree = freePC.size();
	int nWalls = wallPC.size();
	

	if (kdSize > 0) {
		// downweight existing wall cells if in freeCells
		const dim3 blocksPerGridFree((nFree + blockSize1d - 1) / blockSize1d);
		const dim3 blocksPerGridWall((nWalls + blockSize1d - 1) / blockSize1d);

		glm::vec4 *dev_walls_pc, *dev_free_pc;
		cudaMalloc((void**)&dev_walls_pc, nWalls*sizeof(glm::vec4));
		cudaMalloc((void**)&dev_free_pc, nFree*sizeof(glm::vec4));

		int *dev_walls_c, *dev_free_c;
		cudaMalloc((void**)&dev_walls_c, nWalls*sizeof(int));
		cudaMalloc((void**)&dev_free_c, nFree*sizeof(int));

		findCorrespondenceIndexKD << <blocksPerGridFree, BLOCK_SIZE >> >(nFree, dev_free_c, dev_free_pc, dev_kd);
		findCorrespondenceIndexKD << <blocksPerGridWall, BLOCK_SIZE >> >(nWalls, dev_walls_c, dev_walls_pc, dev_kd);

		cudaDeviceSynchronize();
		checkCUDAError("map update - correspondance failure");

		bool *wallsCreate = new bool[nWalls];
		bool *dev_wallsCreate = NULL;
		cudaMalloc((void**)&dev_wallsCreate, nWalls*sizeof(bool));


		// downweight free cells
		kernUpdateMapKD << <blocksPerGridFree, BLOCK_SIZE >> >(nFree, dev_kd, dev_free_pc, dev_free_c, FREE_WEIGHT, map_params);

		// upweight existing wall cells
		kernUpdateMapKD << <blocksPerGridWall, BLOCK_SIZE >> >(nWalls, dev_kd, dev_walls_pc, dev_walls_c, OCCUPIED_WEIGHT, map_params);
		cudaDeviceSynchronize();

		// insert any new wall cells
		kernTestCorrespondance << <blocksPerGridWall, BLOCK_SIZE >> >(nWalls, dev_kd, dev_walls_pc, dev_walls_c, dev_wallsCreate, map_params);

		cudaMemcpy(wallsCreate, dev_wallsCreate, nWalls*sizeof(bool), cudaMemcpyDeviceToHost);
		cudaDeviceSynchronize();
		
		int nInsert = thrust::count(thrust::device, dev_wallsCreate, dev_wallsCreate + nWalls, true);

		// we dont want to recreate the kd tree every time, we just want to maintain copies on both the host and device...
		if (nInsert > 0) {
			glm::vec4 *wallPC = new glm::vec4[nWalls];
			cudaMemcpy(kd, dev_kd, kdSize*sizeof(KDTree::Node), cudaMemcpyDeviceToHost); // make sure to copy new weight values
			cudaMemcpy(wallPC, dev_walls_pc, nWalls*sizeof(glm::vec4), cudaMemcpyDeviceToHost); 
			for (int i = 0; i < nWalls; i++) {
				if (wallsCreate[i]) {
					wallPC[i].w = -100;
					KDTree::InsertNode(wallPC[i], kd, kdSize++);
				}
			}
			//printf("new pointcloud size: %i\n", kdSize);
			cudaMemcpy(dev_kd, kd, kdSize*sizeof(KDTree::Node), cudaMemcpyHostToDevice);
			delete[] wallPC;
		}

		checkCUDAError("map update - insert failure");

		cudaFree(dev_walls_c);
		cudaFree(dev_free_c);
		cudaFree(dev_wallsCreate);
		cudaFree(dev_walls_pc);
		cudaFree(dev_free_pc);

		delete[] wallsCreate;

	} else { // create a new kd tree from first scan
		//std::vector<glm::vec4> wallPC;
		//wallPC.reserve(nWalls);
		//wallPC.push_back(glm::vec4(0.0f));
		//cudaMemcpy(&wallPC[0], dev_walls_pc, nWalls*sizeof(glm::vec4), cudaMemcpyDeviceToHost);

		KDTree::Create(wallPC, kd);
		cudaMemcpy(dev_kd, kd, nWalls*sizeof(KDTree::Node), cudaMemcpyHostToDevice);
		kdSize += wallPC.size();
	}

	//cudaFree(dev_pos);
	//cudaFree(dev_walls_pc);
	//cudaFree(dev_free_pc);
	delete[] wallCells;
	delete[] freeCells;
}


/**
* Wrapper for the __global__ call that sets up the kernel calls and does a ton
* of memory management
*/
void particleFilter(uchar4 *pbo, int frame, Lidar *lidar) {

	if (frame % 1000 == 0)
		printf("PC size: %i\n", kdSize);

	if (frame % 100 == 5) {
		cudaMemcpy(kd, dev_kd, kdSize*sizeof(KDTree::Node), cudaMemcpyDeviceToHost);
		KDTree::Balance(kd, kdSize);
		cudaMemcpy(dev_kd, kd, kdSize*sizeof(KDTree::Node), cudaMemcpyHostToDevice);
	}

	//special case for first scan
	if (kdSize == 0) {
		robotPos = glm::vec3(0.0f, 0.0f, 0.0f);
		PFUpdateMapKD(lidar->scans[frame]);
	}
	else {
		// timing metrics
		if (frame % 100 == 0) {
			avg_motion = 0.0f;
			avg_measurement = 0.0f;
			avg_map = 0.0f;
			avg_sample = 0.0f;
		}

		std::chrono::time_point<std::chrono::system_clock> start, end;
		
		start = std::chrono::system_clock::now();
		PFMotionUpdate(frame);
		end = std::chrono::system_clock::now();
		avg_motion += (std::chrono::duration_cast<std::chrono::microseconds> (end - start)).count();
		start = end;

		robotPos = PFMeasurementUpdateKD(lidar->scans[frame]);
		end = std::chrono::system_clock::now();
		avg_measurement += (std::chrono::duration_cast<std::chrono::microseconds> (end - start)).count();
		start = end;
		
		PFUpdateMapKD(lidar->scans[frame]);
		end = std::chrono::system_clock::now();
		avg_map += (std::chrono::duration_cast<std::chrono::microseconds> (end - start)).count();
		start = end;
		
		PFResample(frame);
		end = std::chrono::system_clock::now();
		avg_sample += (std::chrono::duration_cast<std::chrono::microseconds> (end - start)).count();
		start = end;

		//UpdateTopology();
		//CheckLoopClosure();
		
		// print timing metrics
		if (frame % 100 == 0) {
			cout << "Frame " << frame << ":" << endl;
			printf("    motion:      %3.2f\n", avg_motion / 100.0f);
			printf("    measurement: %3.2f\n", avg_measurement / 100.0f);
			printf("    map:         %3.2f\n", avg_map / 100.0f);
			printf("    resample:    %3.2f\n", avg_sample / 100.0f);
		}
	}






}