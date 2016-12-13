#include "main.h"
#include "preview.h"
#include <cstring>

#include <boost/make_shared.hpp> 
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


#define PCL_ENABLE

static std::string startTimeString;

// For camera controls
static bool leftMousePressed = false;
static bool rightMousePressed = false;
static bool middleMousePressed = false;
static double lastX;
static double lastY;

static bool camchanged = true;
static float dtheta = 0, dphi = 0;
static glm::vec3 cammove;

float zoom, theta, phi;
float dx, dy;
glm::vec3 cameraPosition;
glm::vec3 ogLookAt; // for recentering the camera

Scene *scene;
Lidar *lidar;
RenderState *renderState;
float **map;
int iteration;

int width;
int height;

//-------------------------------
//-------------MAIN--------------
//-------------------------------

int main(int argc, char** argv) {
    startTimeString = currentTimeString();

    if (argc < 2) {
        printf("Usage: %s SCENEFILE.txt\n", argv[0]);
        return 1;
    }

	const char *sceneFile = argv[1];
	const char *lidarFile = argv[2];

    // Load scene file
    scene = new Scene(sceneFile);

	// Load data
	lidar = new Lidar(lidarFile);


    // Set up camera stuff from loaded path tracer settings
    iteration = 0;
    renderState = &scene->state;
    Camera &cam = renderState->camera;
    width = cam.resolution.x;
    height = cam.resolution.y;

    glm::vec3 view = cam.view;
    glm::vec3 up = cam.up;
    glm::vec3 right = glm::cross(view, up);
    up = glm::cross(right, view);

    cameraPosition = cam.position;

    // compute phi (horizontal) and theta (vertical) relative 3D axis
    // so, (0 0 1) is forward, (0 1 0) is up
    glm::vec3 viewXZ = glm::vec3(view.x, 0.0f, view.z);
    glm::vec3 viewZY = glm::vec3(0.0f, view.y, view.z);
    phi = glm::acos(glm::dot(glm::normalize(viewXZ), glm::vec3(0, 0, -1)));
    theta = glm::acos(glm::dot(glm::normalize(viewZY), glm::vec3(0, 1, 0)));
    ogLookAt = cam.lookAt;
    zoom = glm::length(cam.position - ogLookAt);

    // Initialize CUDA and GL components
    init();
	initPCL();

    // GLFW main loop
    mainLoop();

    return 0;
}

void saveImage() {
    float samples = iteration;
    // output image file
    image img(width, height);

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            int index = x + (y * width);
            glm::vec3 pix = renderState->image[index];
            img.setPixel(width - 1 - x, y, glm::vec3(pix));
        }
    }

    std::string filename = renderState->imageName;
    std::ostringstream ss;
    ss << filename << "." << startTimeString;
    filename = ss.str();

    // CHECKITOUT
    img.savePNG(filename);
    //img.saveHDR(filename);  // Save a Radiance HDR file
}


void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
      switch (key) {
      case GLFW_KEY_ESCAPE:
        saveImage();
        glfwSetWindowShouldClose(window, GL_TRUE);
        break;
      case GLFW_KEY_S:
        saveImage();
        break;
      case GLFW_KEY_SPACE:
        camchanged = true;
        renderState = &scene->state;
        Camera &cam = renderState->camera;
        cam.lookAt = ogLookAt;
        break;
      }
    }
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
  leftMousePressed = (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS);
  rightMousePressed = (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS);
  middleMousePressed = (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS);
}

void mousePositionCallback(GLFWwindow* window, double xpos, double ypos) {
	if (leftMousePressed) {
		dx += (xpos - lastX);
		dy -= (ypos - lastY);
		camchanged = true;
	}
	else if (rightMousePressed) {
		zoom *= 1.0f + (ypos - lastY) / height;
		zoom = std::fmax(0.1f, zoom);
		camchanged = true;
	}
	else if (middleMousePressed) {
	}
	lastX = xpos;
	lastY = ypos;
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

void initPCL() {
	viewer->setBackgroundColor(255, 255, 255);
	viewer->addCoordinateSystem(1.0);
	viewer->removeCoordinateSystem();
	viewer->initCameraParameters();
}

void runCuda(bool Visualize) {
	if (camchanged) {
		//iteration = 0;
		Camera &cam = renderState->camera;
		cameraPosition.x = dx;
		cameraPosition.y = dy;
		cameraPosition.z = zoom;

		cam.position = cameraPosition;
		cam.lookAt.x = cameraPosition.x;
		cam.lookAt.y = cameraPosition.y;
		cam.lookAt.z = 0;
		camchanged = false;
	}

	// Map OpenGL buffer object for writing from CUDA on a single GPU
	// No data is moved (Win & Linux). When mapped to CUDA, OpenGL should not use this buffer

	if (iteration == 0) {
		particleFilterFree();
		particleFilterInit(scene);
		//iteration = 10000;	// change start point here for debugging
	}

	bool done = (iteration >= lidar->scans.size() - 1);
	if (!done) {
		uchar4 *pbo_dptr = NULL;
		iteration++;
		cudaGLMapBufferObject((void**)&pbo_dptr, pbo);

		// execute the kernel
		particleFilter(pbo_dptr, iteration, lidar);

		// update display
		if (Visualize || iteration >= lidar->scans.size() - 2) {
			if (PCL_DRAW) {
				drawPCMap();
			}
			else {
				drawMap(pbo_dptr);
			}
		}
		else {
		}

		// unmap buffer object
		cudaGLUnmapBufferObject(pbo);
	}
	else {
		saveImage();
		particleFilterFree();
		cudaDeviceReset();
		exit(EXIT_SUCCESS);
	}
}

void drawPCMap() {
	Particle *ptrParticles = NULL;
	MAP_TYPE *ptrMap = NULL;
	KDTree::Node *ptrKD = NULL;
	int nParticles, nKD;
	glm::vec3 pos(0.0f);
	getPCData(&ptrParticles, &ptrMap, &ptrKD, &nParticles, &nKD, pos);


	// Occupancy grid walls
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr walls(new pcl::PointCloud<pcl::PointXYZRGB>);
	uint8_t r(0), g(0), b(0);
	glm::ivec2 map_dim = glm::ivec2(scene->maps[0].scale.x / scene->maps[0].resolution.x, scene->maps[0].scale.y / scene->maps[0].resolution.y);
	//for (int x = 0; x < map_dim.x; x++) {
	//	for (int y = 0; y < map_dim.y; y++) {
	//		int idx = (x * map_dim.x) + y;
	//		char test = ptrMap[idx];
	//		if (ptrMap[idx] > 30) {
	//			pcl::PointXYZRGB point;
	//			
	//			point.x = x * scene->maps[0].resolution.x - scene->maps[0].scale.x / 2.0f;
	//			point.y = y * scene->maps[0].resolution.y - scene->maps[0].scale.y / 2.0f;
	//			point.z = 0.0f;
	//			uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	//			point.rgb = *reinterpret_cast<float*>(&rgb);
	//			walls->points.push_back(point);
	//		}
	//	}
	//}
	walls->width = (int)walls->points.size();
	walls->height = 1;
	viewer->addPointCloud<pcl::PointXYZRGB>(walls, "walls", 0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "walls");


	// draw point cloud walls
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr kdField(new pcl::PointCloud<pcl::PointXYZRGB>);
	r = 0, g = 0, b = 0;
	for (int i = 0; i < nKD; i++) {
		if (ptrKD[i].value.w > -100) {

			r = 150 - ptrKD[i].value.w;
			g = 150 - ptrKD[i].value.w;
			b = 150 - ptrKD[i].value.w;

			pcl::PointXYZRGB point;
			point.x = ptrKD[i].value.x;
			point.y = ptrKD[i].value.y;
			point.z = ptrKD[i].value.z;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			kdField->points.push_back(point);
		}
	}
	kdField->width = (int)kdField->points.size();
	kdField->height = 1;
	viewer->addPointCloud<pcl::PointXYZRGB>(kdField, "kd", 0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "kd");



	// draw a pointcloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr particleField(new pcl::PointCloud<pcl::PointXYZRGB>);
	r = (255), g = (15), b = (15);
	for (int i = 0; i < nParticles; i++) {
		pcl::PointXYZRGB point;
		point.x = ptrParticles[i].pos.x;
		point.y = ptrParticles[i].pos.y;
		point.z = 0.01f;
		uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		point.rgb = *reinterpret_cast<float*>(&rgb);
		particleField->points.push_back(point);
	}
	particleField->width = (int)particleField->points.size();
	particleField->height = 1;
	viewer->addPointCloud<pcl::PointXYZRGB>(particleField, "particles", 0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "particles");

	//set robot pos
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr robot(new pcl::PointCloud<pcl::PointXYZRGB>);
	r = (15), g = (255), b = (15);
	pcl::PointXYZRGB point;
	point.x = pos.x;
	point.y = pos.y;
	point.z = 0.02f; // pos.z in 2d is heading angle
	uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	point.rgb = *reinterpret_cast<float*>(&rgb);
	robot->points.push_back(point);
	viewer->addPointCloud<pcl::PointXYZRGB>(robot, "robot", 0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "robot");

	// basic view
	viewer->spinOnce();
	viewer->removePointCloud("particles");
	viewer->removePointCloud("walls");
	viewer->removePointCloud("kd");
	viewer->removePointCloud("robot");
}