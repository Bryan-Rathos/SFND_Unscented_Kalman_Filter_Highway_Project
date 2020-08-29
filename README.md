# SFND_Unscented_Kalman_Filter Highway Project

Below is the roadmap of the Unscented Kalman Filter

<img src="media/roadmap.png" width="700" height="400" />

In the `ukf.cpp` file the `ProcessMeasurement()` function calls the `Prediction()` fucntion to predict sigma point based on the Constant Turn Rate and Velocity (CTRV) motion model. It also calls the update step for the Lidar (`UpdateLidar()`) and Radar (`UpdateRadar()`) based on which sensor measurement it receives.

To check weather the UKF performs well and does not underestimate or overestimate the unceratainity of the measurement we run a Normalized Innovation Squared (NIS) test which is a difference between the predicted measurement and actual measurement which is normalized by the measurement covariance matrix `S`. The NIS is a scalar value and follows a Chi-Squared Distribution. Below is a part of a chi squared table.

<img src="media/chi_sq_dist.png" width="400" height="200" />

From the table, 'df' is the degrees of freedom of the sensor measurement space. Radar has 3 DOF and Lidar has 2 DOF. Consider the "df=3" row for Radar. In the last column, X^2 0.050 means that ststistically in 5% of cases the NIS will be more than 7.815. Similarly for Lidar in the "df=2" the 95% line is at 5.991. If there are few values above the 95% line it means that we do not overestimate or underestimate the uncertainity of the system. If we find that all of the NIS vlaues are far below the 95% line it means we understimate the uncertainity of the system. Similarly, if a lot of values are above the 95% line it means taht we overestimate teh uncertainity in or system i.e., our extimations are actually more precise than we think. If we find that we underestimate the uncertainity, we can reduce the process noise parameter in our system to have an acceotable NIS and vice versa for the overestimate case. In our case the process noise variables are the noise in longitudinal acceleration and noise is yaw acceleration. Below is the graph of the NIS for Lidar and Radar I plotted whuch is acceptable. The fine tuned process noise parameter values are (longitudinal acceleration) `std_a = 70.0` and (yaw acceleration) `std_yawdd = 15.0`. Below are the NIS plots for Lidar and Radar respectively.

<img src="media/nis_lidar.png" width="500" height="300" />

<img src="media/nis_radar.png" width="500" height="300" />

The final output of the multiple car position tracking using Unscented Kalman Filter is as follows:

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

In this project an Unscented Kalman Filter is used to estimate the state of multiple cars on a highway using noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ukf_highway

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, and src/ukf.h

The program main.cpp has already been filled out, but feel free to modify it.

<img src="media/ukf_highway.png" width="700" height="400" />

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. 
The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the 
other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has
it's own UKF object generated for it, and will update each indidual one during every time step. 

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so you are only tracking along the X/Y axis.

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
 * PCL 1.2

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ukf_highway`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar modify the code in `highway.h` to alter the cars. Also check out `tools.cpp` to
change how measurements are taken, for instance lidar markers could be the (x,y) center of bounding boxes by scanning the PCD environment
and performing clustering. This is similar to what was done in Sensor Fusion Lidar Obstacle Detection.
