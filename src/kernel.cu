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

#include "sceneStructs.h"
#include "scene.h"
#include "glm/glm.hpp"
#include "glm/gtx/norm.hpp"
#include "utilities.h"
#include "draw.h"
#include "kernel.h"

// Function Device Control
#define GPU_MOTION		1
#define GPU_MEASUREMENT 1
#define GPU_MAP			1
#define GPU_RESAMPLE	1

// Particle Filter Controls
#define PARTICLE_COUNT 5000
#define EFFECTIVE_PARTICLES .7
#define FREE_WEIGHT -1
#define OCCUPIED_WEIGHT 3
#define TOPOLOGY_SPLIT 3.5f
#define WALL_CONFIDENCE 50

// Sensor Configuration
#define LIDAR_ANGLE(i) (-135.0f + i * .25f) * PI / 180
#define LIDAR_SIZE 1081
#define LIDAR_RANGE 20.0f
#define COV {0.015, 0.015, .015}

// GPU calculations
#define BLOCK_SIZE 128

// Helper Functions
#define CLAMP(a, lo, hi) (a < lo) ? lo : (a > hi) ? hi : a


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

	long max_val = 1 << (sizeof(MAP_TYPE) * 8 - 1);
	memset(occupancyGrid, -1*(max_val-1)*0, map_dim.x*map_dim.y*sizeof(MAP_TYPE));

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
	group1.id = 0;
	group1.nodeIdx = 0;
	group1.patchList.push_back(0);
	group1.nodes.push_back(glm::vec2(0.0f, 0.0f));
	std::vector<unsigned int> empty;
	group1.edges.push_back(empty);
	clusters.push_back(group1);

    checkCUDAError("particleFilterInit");
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
		}
	} 

	// push particles to GPU to draw
	cudaMemcpy(dev_particles, particles, PARTICLE_COUNT * sizeof(Particle), cudaMemcpyHostToDevice);
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
	clusters[i].nodes.push_back((glm::vec2) robotPos);

	// add edge from new node to last node
	std::vector<unsigned int> edge;
	edge.push_back(clusters[i].nodeIdx);
	clusters[i].edges.push_back(edge);

	// add edge from last node to new node
	clusters[i].edges[clusters[i].nodeIdx].push_back(clusters[i].nodes.size() - 1);

	// update current node
	clusters[i].nodeIdx = clusters[i].nodes.size() - 1;
	printf("number of graph nodes: %i\n", clusters[i].nodes.size());

}

__global__ void CastVisibilityRays(int N, glm::vec2 a, glm::vec2 *b, bool *rayCells, glm::ivec2 map_dim, Patch map_params) {
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < N) {	
		// convert from world coordinates to global map index
		glm::ivec2 ai(
			round(0.5f * map_dim.x + a.x / map_params.resolution.x + map_params.resolution.x / 2),
			round(0.5f * map_dim.y + a.y / map_params.resolution.y + map_params.resolution.y / 2)
			);

		glm::ivec2 bi(
			round(0.5f * map_dim.x + b[i].x / map_params.resolution.x + map_params.resolution.x / 2),
			round(0.5f * map_dim.y + b[i].y / map_params.resolution.y + map_params.resolution.y / 2)
			);

		traceRay(ai, bi, map_dim, rayCells);
	}
}

__global__ void CheckVisibility(int N, MAP_TYPE *map, bool *mask, bool *retv)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < N) {
		if (mask[i])
			*retv = *retv || (map[i] > WALL_CONFIDENCE);
	}
}

void UpdateTopology() {
	for (int i = 0; i < clusters.size(); i++) {
		bool newNode = true;
		// check if we need a new node on topology graph for each cluster (this is fast on CPU)
		for (int j = 0; j < clusters[i].nodes.size(); j++)
			newNode &= (glm::distance((glm::vec2) robotPos, clusters[i].nodes[j]) > TOPOLOGY_SPLIT);

		if (newNode) CreateNode(i);

		// if we don't need a new node for distance, check if we need one from visibility
		if (!newNode) { // run this on GPU to prevent sending the maps back and forth, this operation can be slow even for a small graph
			glm::vec2 *dev_nodes;
			bool *dev_retv;
			cudaMalloc((void**)&dev_nodes, clusters[i].nodes.size() * sizeof(glm::vec2));
			cudaMalloc((void**)&dev_retv, sizeof(bool));
			cudaMemcpy(dev_nodes, &clusters[i].nodes[0], clusters[i].nodes.size() * sizeof(glm::vec2), cudaMemcpyHostToDevice);
			cudaMemset(dev_retv, 0, sizeof(bool));
			cudaMemset(dev_freeCells, 0, map_dim.x * map_dim.y*sizeof(bool));

			// 1D block for particles
			const int blockSize1d = 128;
			const dim3 blocksPerGridCluster((clusters[i].nodes.size() + blockSize1d - 1) / blockSize1d);
			const dim3 blocksPerGridMap((map_dim.x * map_dim.y + blockSize1d - 1) / blockSize1d);

			CastVisibilityRays << <blocksPerGridCluster, blockSize1d >> >(clusters[i].nodes.size(), (glm::vec2) robotPos, dev_nodes, dev_freeCells, map_dim, map_params);
			cudaDeviceSynchronize();
			CheckVisibility << <blocksPerGridMap, blockSize1d >> >(map_dim.x * map_dim.y, dev_occupancyGrid, dev_freeCells, dev_retv);
			cudaDeviceSynchronize();

			//for (int j = 0; j < clusters[i].nodes.size(); j++)
			//	if (IsVisible((glm::vec2) robotPos, clusters[i].nodes[j])) obstructed = false;
			
			cudaFree(dev_nodes);
			cudaFree(dev_retv);
			
			//if (obstructed) CreateNode(i);
		}

	}
}

/**
 * Wrapper for the __global__ call that sets up the kernel calls and does a ton
 * of memory management
 */
void particleFilter(uchar4 *pbo, int frame, Lidar *lidar) {

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

	robotPos = PFMeasurementUpdate(lidar->scans[frame]);
	end = std::chrono::system_clock::now();
	avg_measurement += (std::chrono::duration_cast<std::chrono::microseconds> (end - start)).count();
	start = end;
	
	PFUpdateMap(lidar->scans[frame]);
	end = std::chrono::system_clock::now();
	avg_map += (std::chrono::duration_cast<std::chrono::microseconds> (end - start)).count();
	start = end;
	
	PFResample(frame);
	end = std::chrono::system_clock::now();
	avg_sample += (std::chrono::duration_cast<std::chrono::microseconds> (end - start)).count();
	start = end;

	UpdateTopology();

	// print timing metrics
	if (frame % 100 == -1) {
		cout << "Frame " << frame << ":" << endl;
		printf("    motion:      %3.2f\n", avg_motion / 100.0f);
		printf("    measurement: %3.2f\n", avg_measurement / 100.0f);
		printf("    map:         %3.2f\n", avg_map / 100.0f);
		printf("    resample:    %3.2f\n", avg_sample / 100.0f);
	}
}

void drawMap(uchar4 *pbo) {
	drawAll(pbo, PARTICLE_COUNT, hst_scene, dev_image, robotPos, dev_particles, dev_occupancyGrid, dev_maps, clusters);
	checkCUDAError("draw screen error");
}