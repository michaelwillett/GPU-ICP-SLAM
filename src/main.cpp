#include "main.h"
#include "preview.h"
#include <cstring>

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

void runCuda(bool Visualize) {
    if (camchanged) {
        iteration = 0;
        Camera &cam = renderState->camera;
		cameraPosition.x = dx;
		cameraPosition.y = dy;
		cameraPosition.z = zoom;
		
		cam.position = cameraPosition;
		cam.lookAt.x = cameraPosition.x;
		cam.lookAt.y = cameraPosition.y;
		cam.lookAt.z = 0;
        camchanged = false;
		std::cout << cam.position.x << " " << cam.position.y << " " << cam.position.z << std::endl;
      }

    // Map OpenGL buffer object for writing from CUDA on a single GPU
    // No data is moved (Win & Linux). When mapped to CUDA, OpenGL should not use this buffer

    if (iteration == 0) {
        particleFilterFree();
		particleFilterInit(scene);
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
			drawMap(pbo_dptr);
		}

		// unmap buffer object
		cudaGLUnmapBufferObject(pbo);
    } else {
        saveImage();
		particleFilterFree();
        cudaDeviceReset();
        exit(EXIT_SUCCESS);
    }
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
		// compute new camera parameters
		//phi -= (xpos - lastX) / width;
		//theta -= (ypos - lastY) / height;
		//theta = std::fmax(0.001f, std::fmin(theta, PI));
		dx += (xpos - lastX);
		dy -= (ypos - lastY);
		//camchanged = true;
	}
	else if (rightMousePressed) {
		zoom += (ypos - lastY) / height;
		zoom = std::fmax(0.1f, zoom);
		//camchanged = true;
	}
	else if (middleMousePressed) {
		//renderState = &scene->state;
		//Camera &cam = renderState->camera;
		//glm::vec3 forward = cam.view;
		//forward.y = 0.0f;
		//forward = glm::normalize(forward);
		//glm::vec3 right = cam.right;
		//right.y = 0.0f;
		//right = glm::normalize(right);

		//cam.lookAt -= (float) (xpos - lastX) * right * 0.01f;
		//cam.lookAt += (float) (ypos - lastY) * forward * 0.01f;
		//camchanged = true;
	}
	lastX = xpos;
	lastY = ypos;
}
