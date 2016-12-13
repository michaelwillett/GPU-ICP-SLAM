#include <cstdio>
#include <cuda.h>
#include "sceneStructs.h"
#include "utilities.h"
#include "draw.h"


//Bresenham's line algorithm for integer grid
__device__ __host__ void DrawRay(glm::ivec2 start, glm::ivec2 end, glm::ivec2 map_dim, glm::vec3 * image, glm::vec3 color){
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
			idx = y + x*map_dim.x;
		else
			idx = x + y*map_dim.y;

		if (x < map_dim.x && y < map_dim.y && x >= 0 && y >= 0 && idx < map_dim.x * map_dim.y) { // assume square maps
			cudaMemcpy(&image[idx], &color, sizeof(glm::vec3), cudaMemcpyHostToDevice);
		}

		error -= deltay;
		if (error < 0){
			y += ystep;
			error += deltax;
		}
	}

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
		color.x = glm::clamp((int)(pix.x / iter * 255.0), 0, 255);
		color.y = glm::clamp((int)(pix.y / iter * 255.0), 0, 255);
		color.z = glm::clamp((int)(pix.z / iter * 255.0), 0, 255);

		// Each thread writes one pixel location in the texture (textel)
		pbo[index].w = 0;
		pbo[index].x = color.x;
		pbo[index].y = color.y;
		pbo[index].z = color.z;
	}
}

// Display the occupancy grid
__global__ void drawMap(int nPixels, glm::vec3 * image, Patch *objects, Camera cam, MAP_TYPE *occupancyGrid)
{
	int x = (blockIdx.x * blockDim.x) + threadIdx.x;
	int y = (blockIdx.y * blockDim.y) + threadIdx.y;

	if (x < cam.resolution.x && y < cam.resolution.y) {
		int index = x + (y * cam.resolution.x);
		Patch map = objects[0];

		// convert pixel coordinates to map coordates
		float zoom = cam.position.z;
		glm::vec2 mid((float)cam.resolution.x / 2.0f, (float)cam.resolution.y / 2.0f);
		float xc = (x - mid.x + cam.position.x) / zoom;
		float yc = (y - mid.y + cam.position.y) / zoom;

		// check if pixel is in the map
		if (xc < map.scale.x / 2 && xc > -map.scale.x / 2 && yc < map.scale.y / 2 && yc > -map.scale.y / 2) {
			glm::ivec2 idx(
				round(0.5f * map.scale.x / map.resolution.x + xc / map.resolution.x),
				round(0.5f * map.scale.y / map.resolution.y + yc / map.resolution.y)
				);

			long max_val = 1 << (sizeof(MAP_TYPE)* 8 - 1);
			float val = ((float)(occupancyGrid[idx.x * (int)(map.scale.x / map.resolution.x) + idx.y] + max_val)) / (max_val * 2);
			image[index] = glm::vec3(1.0-val);
		}
		else
			image[index] = glm::vec3(1.0f);
	}
}

// Display particles on screen
__global__ void drawParticles(unsigned int n, glm::vec3 * image, Particle *particles, Camera cam) {
	int i = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (i < n) {
		// convert map coordinates to pixel coordinates
		float zoom = cam.position.z;
		glm::vec2 mid((float)cam.resolution.x / 2.0f, (float)cam.resolution.y / 2.0f);
		int x = particles[i].pos.x * zoom + mid.x - cam.position.x;
		int y = particles[i].pos.y * zoom + mid.y - cam.position.y;

		int l = cam.resolution.x;
		int index = x + (y * l);

		if (x >= 0 && y >= 0 && x < cam.resolution.x && y < cam.resolution.y)
			image[index] = glm::vec3(0.0f, 1.0f, 1.0f);
	}
}

// display a box for robot position
void drawRobot(glm::vec3 * image, glm::vec3 robot, Camera cam) {
	// convert map coordinates to pixel coordinates
	float zoom = cam.position.z;
	glm::vec2 mid((float)cam.resolution.x / 2.0f, (float)cam.resolution.y / 2.0f);
	int x = robot.x * zoom + mid.x - cam.position.x;
	int y = robot.y * zoom + mid.y - cam.position.y;

	int l = cam.resolution.x;
	int index = x + (y * l);
	glm::vec3 color(1.0f, 0.0f, 0.0f);
	glm::vec3 row[3] = { color, color, color };


	if (x >= 0 && y >= 0 && x < cam.resolution.x && y < cam.resolution.y) {
		cudaMemcpy(&image[index - 1], row, 3 * sizeof(glm::vec3), cudaMemcpyHostToDevice);
		cudaMemcpy(&image[index - 1 + l], row, 3 * sizeof(glm::vec3), cudaMemcpyHostToDevice);
		cudaMemcpy(&image[index - 1 - l], row, 3 * sizeof(glm::vec3), cudaMemcpyHostToDevice);
	}
}

void drawGraph(glm::vec3 *image, std::vector<Cluster> clusters, Camera cam) {
	for (int i = 0; i < clusters.size(); i++) {
		for (int j = 0; j < clusters[i].nodes.size(); j++) {
			// convert map coordinates to pixel coordinates
			float zoom = cam.position.z;
			glm::vec2 mid((float)cam.resolution.x / 2.0f, (float)cam.resolution.y / 2.0f);
			int x0 = clusters[i].nodes[j].pos.x * zoom + mid.x - cam.position.x;
			int y0 = clusters[i].nodes[j].pos.y * zoom + mid.y - cam.position.y;

			int l = cam.resolution.x;
			int index = x0 + (y0 * l);
			glm::vec3 color(1.0f, 0.5f, 0.0f);
			glm::vec3 row[3] = { color, color, color };


			if (x0 >= 0 && y0 >= 0 && x0 < cam.resolution.x && y0 < cam.resolution.y) {
				cudaMemcpy(&image[index - 1], row, 3 * sizeof(glm::vec3), cudaMemcpyHostToDevice);
				cudaMemcpy(&image[index - 1 + l], row, 3 * sizeof(glm::vec3), cudaMemcpyHostToDevice);
				cudaMemcpy(&image[index - 1 - l], row, 3 * sizeof(glm::vec3), cudaMemcpyHostToDevice);
			}

			// draw all edges
			std::vector<unsigned int> edges = clusters[i].edges[j];
			for (int k = 0; k < edges.size(); k++) {
				int x1 = clusters[i].nodes[edges[k]].pos.x * zoom + mid.x - cam.position.x;
				int y1 = clusters[i].nodes[edges[k]].pos.y * zoom + mid.y - cam.position.y;
				DrawRay(glm::ivec2(x0, y0), glm::ivec2(x1, y1), cam.resolution, image, color);
			}
		}
	}
}

void drawAll(uchar4 *pbo, unsigned int n, Scene *hst_scene, glm::vec3 *dev_image, glm::vec3 robotPos, Particle *dev_particles, MAP_TYPE *dev_occupancyGrid, Patch *dev_maps, std::vector<Cluster> clusters)
{
	const Camera &cam = hst_scene->state.camera;
	const int pixelcount = cam.resolution.x * cam.resolution.y;

	// 2D block for generating pixels in camera
	const dim3 blockSize2d(8, 8);
	const dim3 blocksPerGrid2d(
		(cam.resolution.x + blockSize2d.x - 1) / blockSize2d.x,
		(cam.resolution.y + blockSize2d.y - 1) / blockSize2d.y);

	const int blockSize1d = 128;
	const dim3 blocksPerGrid1d((n + blockSize1d - 1) / blockSize1d);

	drawMap << <blocksPerGrid2d, blockSize2d >> >(pixelcount, dev_image, dev_maps, cam, dev_occupancyGrid);
	drawParticles << <blocksPerGrid1d, blockSize1d >> > (n, dev_image, dev_particles, cam);
	drawRobot(dev_image, robotPos, cam);
	//drawGraph(dev_image, clusters, cam);

	// Send results to OpenGL buffer for rendering
	sendImageToPBO << <blocksPerGrid2d, blockSize2d >> >(pbo, cam.resolution, 1, dev_image);

	// Retrieve image from GPU
	cudaMemcpy(hst_scene->state.image.data(), dev_image,
		pixelcount * sizeof(glm::vec3), cudaMemcpyDeviceToHost);
}