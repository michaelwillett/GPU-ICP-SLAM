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

#include "intersections.h"
#include "sceneStructs.h"
#include "scene.h"
#include "glm/glm.hpp"
#include "glm/gtx/norm.hpp"
#include "utilities.h"
#include "kernel.h"


#define GPU_MOTION		1
#define GPU_MEASUREMENT 1
#define GPU_MAP			1
#define GPU_RESAMPLE	1

#define LIDAR_ANGLE(i) (-135.0f + i * .25f) * PI / 180
#define LIDAR_SIZE 1081
#define PARTICLE_COUNT 5000
#define COV {0.015, 0.015, .015}
#define EFFECTIVE_PARTICLES .7
#define MAP_TYPE char
#define FREE_WEIGHT -1
#define OCCUPIED_WEIGHT 3

#define BLOCK_SIZE 128
#define ERRORCHECK 1

#define CLAMP(a, lo, hi) (a < lo) ? lo : (a > hi) ? hi : a

#define FILENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define checkCUDAError(msg) checkCUDAErrorFn(msg, FILENAME, __LINE__)

void checkCUDAErrorFn(const char *msg, const char *file, int line) {
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

//Kernel that writes the image to the OpenGL PBO directly.
__global__ void sendImageToPBO(uchar4* pbo, glm::ivec2 resolution,
        int iter, glm::vec3* image) {
    int x = (blockIdx.x * blockDim.x) + threadIdx.x;
    int y = (blockIdx.y * blockDim.y) + threadIdx.y;

    if (x < resolution.x && y < resolution.y) {
        int index = x + (y * resolution.x);
        glm::vec3 pix = image[index];

        glm::ivec3 color;
        color.x = glm::clamp((int) (pix.x / iter * 255.0), 0, 255);
        color.y = glm::clamp((int) (pix.y / iter * 255.0), 0, 255);
        color.z = glm::clamp((int) (pix.z / iter * 255.0), 0, 255);

        // Each thread writes one pixel location in the texture (textel)
        pbo[index].w = 0;
        pbo[index].x = color.x;
        pbo[index].y = color.y;
        pbo[index].z = color.z;
    }
}

__host__ __device__ thrust::default_random_engine makeSeededRandomEngine(int iter, int index, int depth) {
	int h = utilhash((1 << 31) | (depth << 22) | iter) ^ utilhash(index);
	return thrust::default_random_engine(h);
}

static Scene * hst_scene = NULL;
static glm::vec3 * dev_image = NULL;
static Geom * dev_geoms = NULL;

// host variables
static MAP_TYPE *occupancyGrid = NULL;
static glm::vec4 particles[PARTICLE_COUNT];
static glm::ivec2 map_dim;
static Geom map_params;
static glm::vec3 robotPos;

// device variable
static MAP_TYPE *dev_occupancyGrid = NULL;
static glm::vec4 *dev_particles = NULL;
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

	cudaMalloc(&dev_geoms, scene->geoms.size() * sizeof(Geom));
	cudaMemcpy(dev_geoms, scene->geoms.data(), scene->geoms.size() * sizeof(Geom), cudaMemcpyHostToDevice);

	map_params = scene->geoms[0];
	map_dim = glm::ivec2(map_params.scale.x / map_params.resolution.x, map_params.scale.y / map_params.resolution.y);

	occupancyGrid = new MAP_TYPE[map_dim.x*map_dim.y];

	long max_val = 1 << (sizeof(MAP_TYPE) * 8 - 1);
	memset(occupancyGrid, -1*(max_val-1)*0, map_dim.x*map_dim.y*sizeof(MAP_TYPE));

	//particles = new glm::vec4[PARTICLE_COUNT]; 
	for (int i = 0; i < PARTICLE_COUNT; i++) {
		particles[i] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	}

	robotPos = glm::vec3(0.0f);

	cudaMalloc(&dev_occupancyGrid, map_dim.x*map_dim.y * sizeof(MAP_TYPE));
	cudaMemcpy(dev_occupancyGrid, occupancyGrid, map_dim.x*map_dim.y * sizeof(MAP_TYPE), cudaMemcpyHostToDevice);

	cudaMalloc(&dev_particles, PARTICLE_COUNT * sizeof(glm::vec4));
	cudaMemcpy(dev_particles, particles, PARTICLE_COUNT * sizeof(glm::vec4), cudaMemcpyHostToDevice);

	cudaMalloc((void**)&dev_fit, PARTICLE_COUNT * sizeof(int));
	cudaMalloc((void**)&dev_weights, PARTICLE_COUNT * sizeof(float));
	cudaMalloc((void**)&dev_lidar, LIDAR_SIZE * sizeof(float));
	cudaMalloc((void**)&dev_freeCells, map_dim.x * map_dim.y * sizeof(bool));
	cudaMalloc((void**)&dev_wallCells, map_dim.x * map_dim.y * sizeof(bool));


    checkCUDAError("particleFilterInit");
}

void particleFilterFree() {
    cudaFree(dev_image);  // no-op if dev_image is null
	cudaFree(dev_geoms);
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

// Display the occupancy grid
__global__ void drawMap(int nPixels, glm::vec3 * image, Geom *objects, Camera cam, MAP_TYPE *occupancyGrid, glm::vec3 scale, glm::vec3 res)
{
	int x = (blockIdx.x * blockDim.x) + threadIdx.x;
	int y = (blockIdx.y * blockDim.y) + threadIdx.y;

	if (x < cam.resolution.x && y < cam.resolution.y) {
		int index = x + (y * cam.resolution.x);
		Geom map = objects[0];

		// convert pixel coordinates to map coordates
		float zoom = cam.position.z;
		glm::vec2 mid((float)cam.resolution.x / 2.0f, (float) cam.resolution.y / 2.0f);
		float xc = (x - mid.x + cam.position.x) / zoom;
		float yc = (y - mid.y + cam.position.y) / zoom;

		// check if pixel is in the map
		if (xc < map.scale.x / 2 && xc > -map.scale.x / 2 && yc < map.scale.y / 2 && yc > -map.scale.y / 2) {
			glm::ivec2 idx(
				round(0.5f * scale.x / res.x + xc / res.x),
				round(0.5f * scale.y / res.y + yc / res.y)
			);

			long max_val = 1 << (sizeof(MAP_TYPE)* 8 - 1);
			float val = ((float)(occupancyGrid[idx.x * (int)(scale.x / res.x) + idx.y] + max_val)) / (max_val*2);
			image[index] = glm::vec3(val);
		}
		else
			image[index] = glm::vec3(1.0f);
	}
}

// Display particles on screen
__global__ void drawParticles(glm::vec3 * image, glm::vec4 *particles, Camera cam, glm::vec3 scale, glm::vec3 res) {
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < PARTICLE_COUNT) {
		// convert map coordinates to pixel coordinates
		float zoom = cam.position.z;
		glm::vec2 mid((float)cam.resolution.x / 2.0f, (float)cam.resolution.y / 2.0f);
		int x = particles[i].x * zoom + mid.x - cam.position.x;
		int y = particles[i].y * zoom + mid.y - cam.position.y;

		int l = cam.resolution.x;
		int index = x + (y * l);

		image[index] = glm::vec3(0.0f, 1.0f, 1.0f);
	}
}

// display a box for robot position
void drawRobot(glm::vec3 * image, glm::vec3 robot, Camera cam, glm::vec3 scale, glm::vec3 res) {
	// convert map coordinates to pixel coordinates
	float zoom = cam.position.z;
	glm::vec2 mid((float)cam.resolution.x / 2.0f, (float)cam.resolution.y / 2.0f);
	int x = robot.x * zoom + mid.x - cam.position.x;
	int y = robot.y * zoom + mid.y - cam.position.y;

	int l = cam.resolution.x;
	int index = x + (y * l);
	glm::vec3 color(1.0f, 0.0f, 0.0f);
	glm::vec3 row[3] = { color, color, color };

	cudaMemcpy(&image[index - 1], row, 3 * sizeof(glm::vec3), cudaMemcpyHostToDevice);
	cudaMemcpy(&image[index - 1 + l], row, 3 * sizeof(glm::vec3), cudaMemcpyHostToDevice);
	cudaMemcpy(&image[index - 1 - l], row, 3 * sizeof(glm::vec3), cudaMemcpyHostToDevice);
}

// rotates generates 2d point for lidar reading
__device__ __host__ void CleanLidarScan(int n, const float scan, const float theta, glm::vec2 &intersection) {
	float rot = LIDAR_ANGLE(n) + theta;
	
	intersection.x = scan * std::cos(rot);
	intersection.y = scan * std::sin(rot);
}

//Bresenham's line algorithm for integer grid
__device__ __host__ void traceRay(glm::ivec2 start, glm::ivec2 end, int rowLen, bool *out){
	glm::ivec2 delta = end - start;

	// swap for to the right octant
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
		if (steep) 
			out[y*rowLen + x] = 1;
		else 
			out[x*rowLen + y] = 1;
		
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

__device__ __host__ int EvaluateParticle(MAP_TYPE *map, glm::ivec2 map_dim, Geom map_params, glm::vec4 &particle, glm::vec3 pos, float *lidar)
{
	// get walls relative to robot position, add particle position
	glm::vec2 walls[LIDAR_SIZE];

	for (int j = 0; j < LIDAR_SIZE; j++) {
		CleanLidarScan(j, lidar[j], particle.z, walls[j]);
		walls[j].x += particle.x;
		walls[j].y += particle.y;

		// convert to grid idx
		walls[j].x = round(0.5f * map_params.scale.x / map_params.resolution.x + walls[j].x / map_params.resolution.x);
		walls[j].y = round(0.5f * map_params.scale.y / map_params.resolution.y + walls[j].y / map_params.resolution.y);
	}

	// test the map correlation between global map and walls
	return mapCorrelation(LIDAR_SIZE, map, map_dim, walls);
}

// kernel wrapper for calling Evaluate Particle
__global__ void kernEvaluateParticles(MAP_TYPE *map, glm::ivec2 map_dim, Geom map_params, glm::vec4 *particles, glm::vec3 pos, float *lidar, int *fit)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < PARTICLE_COUNT) {
		fit[i] = EvaluateParticle(map, map_dim, map_params, particles[i], pos, lidar);
	}
}

// simple inplace multiplication kernel
__global__ void kernUpdateWeights(int N, glm::vec4 *a, int *b, float c, int min)
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < N) {
		a[i].w = a[i].w * ((float) b[i] - min) * c;
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
		retv = (glm::vec3) particles[best];
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

		retv = (glm::vec3) particles[iBest];
	}

	return retv;
}

// add noise to a single particle
__device__ __host__ void ParticleAddNoise(glm::vec4 &particle, int frame, int idx)
{
	float mean[3] = { 0 };
	float cov[3] = COV;		// covariance: x y theta

	thrust::default_random_engine e2 = makeSeededRandomEngine(frame, idx, 0);
	thrust::random::normal_distribution<float> distx(mean[0], cov[0]);
	thrust::random::normal_distribution<float> disty(mean[1], cov[1]);
	thrust::random::normal_distribution<float> distt(mean[2], cov[2]);

	glm::vec4 noise(distx(e2), disty(e2), distt(e2), 0.0f);
	particle += noise;
}

// kernel wrapper for adding noise to a particle
__global__ void kernAddNoise(glm::vec4 *particles, int frame) 
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
		cudaMemcpy(dev_particles, particles, PARTICLE_COUNT * sizeof(glm::vec4), cudaMemcpyHostToDevice);
		kernAddNoise << <blocksPerGrid1d, blockSize1d >> >(dev_particles, frame);
		cudaMemcpy(particles, dev_particles, PARTICLE_COUNT * sizeof(glm::vec4), cudaMemcpyDeviceToHost);
		cudaDeviceSynchronize(); 
		
		checkCUDAError("particle motion update error");
	} else {
		for (int i = 0; i < PARTICLE_COUNT; i++)
			ParticleAddNoise(particles[i], frame, i);
	}
}

__global__ void kernCopyWeights(glm::vec4 *particles, float *weights, bool squared) 
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < PARTICLE_COUNT) {
		weights[i] = (squared) ? particles[i].w * particles[i].w : particles[i].w;
	}
}

__global__ void kernWeightedSample(glm::vec4 *particles, float *weights, float max, float Neff, int frame) 
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
			cudaMemcpy(particles, dev_particles, PARTICLE_COUNT * sizeof(glm::vec4), cudaMemcpyDeviceToHost);

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
	cudaMemcpy(dev_particles, particles, PARTICLE_COUNT * sizeof(glm::vec4), cudaMemcpyHostToDevice);
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

__global__ void kernGetWalls(float *lidar, glm::ivec2 center, float theta, bool *freeCells, bool *wallCells, glm::ivec2 map_dim, Geom map_params) 
{
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < LIDAR_SIZE) {
		glm::vec2 walls;

		CleanLidarScan(i, lidar[i], theta, walls);
		walls.x = round(walls.x / map_params.resolution.x);
		walls.y = round(walls.y / map_params.resolution.y);
		walls += center;

		if (walls.x >= 0 && walls.x < map_dim.x && walls.y >= 0 && walls.y < map_dim.y) {
			traceRay(center, walls, map_dim.x, freeCells);
			wallCells[(int) (walls.x * map_dim.x + walls.y)] = true;
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
				traceRay(center_idx, walls[i], map_dim.x, freeCells);
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

/**
 * Wrapper for the __global__ call that sets up the kernel calls and does a ton
 * of memory management
 */
void particleFilter(uchar4 *pbo, int frame, Lidar *lidar) {


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

	if (frame % 100 == -1) {
		cout << "Frame " << frame << ":" << endl;
		printf("    motion:      %3.2f\n", avg_motion / 100.0f);
		printf("    measurement: %3.2f\n", avg_measurement / 100.0f);
		printf("    map:         %3.2f\n", avg_map / 100.0f);
		printf("    resample:    %3.2f\n", avg_sample / 100.0f);
	}
}


void drawMap(uchar4 *pbo)
{
	const Camera &cam = hst_scene->state.camera;
	const int pixelcount = cam.resolution.x * cam.resolution.y;

	// 2D block for generating pixels in camera
	const dim3 blockSize2d(8, 8);
	const dim3 blocksPerGrid2d(
		(cam.resolution.x + blockSize2d.x - 1) / blockSize2d.x,
		(cam.resolution.y + blockSize2d.y - 1) / blockSize2d.y);

	const int blockSize1d = 128;
	const dim3 blocksPerGrid1d((PARTICLE_COUNT + blockSize1d - 1) / blockSize1d);

	drawMap << <blocksPerGrid2d, blockSize2d >> >(pixelcount, dev_image, dev_geoms, cam, dev_occupancyGrid, map_params.scale, map_params.resolution);
	drawParticles << <blocksPerGrid1d, blockSize1d >> > (dev_image, dev_particles, cam, map_params.scale, map_params.resolution);
	drawRobot(dev_image, robotPos, cam, map_params.scale, map_params.resolution);
	checkCUDAError("draw screen error");


	// Send results to OpenGL buffer for rendering
	sendImageToPBO << <blocksPerGrid2d, blockSize2d >> >(pbo, cam.resolution, 1, dev_image);

	// Retrieve image from GPU
	cudaMemcpy(hst_scene->state.image.data(), dev_image,
		pixelcount * sizeof(glm::vec3), cudaMemcpyDeviceToHost);
}