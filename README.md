**University of Pennsylvania, CIS 565: GPU Programming and Architecture,
Final Project - GPU Rao-Blackwellized Particle Filter SLAM **

* Michael Willett
* Tested on: Windows 10, I5-4690k @ 3.50GHz 16.00GB, GTX 750-TI 2GB (Personal Computer)

## Contents
1. [Introduction](#intro)
2. [Particle Filter Algorithm](#part1)
3. [Performance Analysis](#part2)
6. [Build Instructions](#appendix)


<a name="intro"/>
## Introduction: Particle Filters
One of the biggest challenges in robotics operating in social environments is the task of building a map of
the region and accurating identifying where the robot is (known as Simultanious Localization and Mapping, or SLAM). 
Since the late 90's, one of the most successful algorithms for solving this problem is using a particle filter to 
estimate the robot position relative to its observations, and build the map successively as new measurements are made. 
A particle filter in its simplest form is a Monte Carlo estimation of the current robot state. By creating a large 
number of potential robot positions and checking each one relative to past knowledge, accurate estimates of true 
coordinates can be achieved without complex regression fitting of sensor data.

The biggest limitation of particle filters is that for a large amount of particles, CPU implementation is incredibly 
costly. To compensate for this, historically SLAM algorithms relied heavily on good robot odometry to create a prior
on the distribution using dead-reckoning. However, as mobile graphics processors become more viable for robotic applications
increasing the particle count becomes a great way to improve estimation accuracy without siginificant runtime increase.
In fact, the particle filter algorithm for SLAM can be considered embarassingly parallel in implementation, and accurate
results can be achieved from visual sensor data only, without any need to develop a prior with robot odometry.

<img src="images/timelapse.gif" width="800" alt="ICP On Cylinder"  style="margin: auto;display: block;">


<a name="part1"/>
## Section 1: Basic Particle Filter SLAM Algorithm

The Particle Filter algorithm can be broken into six basic steps:

1. Disperse all particles with gaussian random noise.
2. For each particle, calculate which grid cells the sensor detects an obsticle in, and sum these values.
3. Select the highest scoring particle as the new position for the next time step, and adjust all particle weights
relative to the distribution of scores calculated in step 2.
4. Update the map by increasing the value of detected collisions, and decreasing the value of cells between the current
position and each measured collision.
5. Resample the particles proportionally to their weight distribution.
6. Repeat steps 1 through 5 for each successive sensor input.


<a name="part2"/>
## Section 2: Improving the Algorithm
One major problem with simple SLAM algorithms is the issue of loop closure. This occurs when the sensor travels for a relatively
long distance into new environments, and slowly minor positional errors in the mapping will result in the final map not adjusting
for a known position when it revisits an area. Below are some examples of maps with good loop closure:

<img src="images/goal_maps.png" width="600" alt="final maps"  style="margin: auto;display: block;">

The most common method of active loop closure is to build a topology graph on top of the occupancy grid, and as new nodes are added,
try to merge nodes that are close in euclidean distance, but distance in graph distance.
<img src="images/graph_building.png" width="600" alt="final maps"  style="margin: auto;display: block;">

Here, we can see the current code inserting a new node onto the topology graph:
<img src="images/graph_building.png" width="600" alt="final maps"  style="margin: auto;display: block;">


<a name="appendix"/>
## Appendix: Build Instructions
* `src/` contains the source code.

This code requires the matlab runtime library for C++ to be installed on the machine. Make sure that the matlab library path
is included in Project>Properties>Configuration Properties>Debugging "Environment". E.G.:

> PATH=%PATH%;C:\Program Files\MATLAB\R2015a\bin\win64;

Once compiled, the executable requires two input files: a text file specifying camera data and a matlab file containing lidar scans.
See the 'data' folder for examples of both.

**CMake note:** Do not change any build settings or add any files to your
project directly (in Visual Studio, Nsight, etc.) Instead, edit the
`src/CMakeLists.txt` file. Any files you add must be added here. If you edit it,
just rebuild your VS/Nsight project to make it update itself.

**If you experience linker errors on build related to the compute capability during thrust calls, edit the project to include the CUDA
library 'cudadevrt.lib'**

#### Windows

1. In Git Bash, navigate to your cloned project directory.
2. Create a `build` directory: `mkdir build`
   * (This "out-of-source" build makes it easy to delete the `build` directory
     and try again if something goes wrong with the configuration.)
3. Navigate into that directory: `cd build`
4. Open the CMake GUI to configure the project:
   * `cmake-gui ..` or `"C:\Program Files (x86)\cmake\bin\cmake-gui.exe" ..`
     * Don't forget the `..` part!
   * Make sure that the "Source" directory is like
     `.../Project5-Particle-Filter-SLAM`.
   * Click *Configure*.  Select your version of Visual Studio, Win64.
     (**NOTE:** you must use Win64, as we don't provide libraries for Win32.)
   * If you see an error like `CUDA_SDK_ROOT_DIR-NOTFOUND`,
     set `CUDA_SDK_ROOT_DIR` to your CUDA install path. This will be something
     like: `C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5`
   * Click *Generate*.
5. If generation was successful, there should now be a Visual Studio solution
   (`.sln`) file in the `build` directory that you just created. Open this.
   (from the command line: `explorer *.sln`)
6. Build. (Note that there are Debug and Release configuration options.)
7. Run. Make sure you run the `cis565_` target (not `ALL_BUILD`) by
   right-clicking it and selecting "Set as StartUp Project".
   * If you have switchable graphics (NVIDIA Optimus), you may need to force
     your program to run with only the NVIDIA card. In NVIDIA Control Panel,
     under "Manage 3D Settings," set "Multi-display/Mixed GPU acceleration"
     to "Single display performance mode".

#### OS X & Linux

This build has not been tested on OS X or Linux. Since lidar data is loaded from a matlab file, the 
user is responsible for linking the appropriate libraries to compile.

It is recommended that you use Nsight.

1. Open Nsight. Set the workspace to the one *containing* your cloned repo.
2. *File->Import...->General->Existing Projects Into Workspace*.
   * Select the Project 0 repository as the *root directory*.
3. Select the *cis565-* project in the Project Explorer. From the *Project*
   menu, select *Build All*.
   * For later use, note that you can select various Debug and Release build
     configurations under *Project->Build Configurations->Set Active...*.
4. If you see an error like `CUDA_SDK_ROOT_DIR-NOTFOUND`:
   * In a terminal, navigate to the build directory, then run: `cmake-gui ..`
   * Set `CUDA_SDK_ROOT_DIR` to your CUDA install path.
     This will be something like: `/usr/local/cuda`
   * Click *Configure*, then *Generate*.
5. Right click and *Refresh* the project.
6. From the *Run* menu, *Run*. Select "Local C/C++ Application" and the
   `cis565_` binary.