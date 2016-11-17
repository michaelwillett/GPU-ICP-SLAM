set(SOURCE_FILES
    "stb.cpp"
    "image.cpp"
    "image.h"
    "glslUtility.hpp"
    "glslUtility.cpp"
    "kernel.cu"
    "kernel.h"
    "scene.cpp"
    "scene.h"
    "lidar.h"
    "lidar.cpp"
    "sceneStructs.h"
    "preview.h"
    "preview.cpp"
    "utilities.cpp"
    "utilities.h"
    "intersections.h"
    "mat.h"
    "matrix.h"
    "tmwtypes.h"
    )

cuda_add_library(src
    ${SOURCE_FILES}
    OPTIONS -arch=sm_50
    )
