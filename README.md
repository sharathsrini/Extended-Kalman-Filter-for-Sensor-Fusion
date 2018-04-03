
[//]: # (Image References)
[image1]: EKF.png

# Extended Kalman Filter Project

In this project I have utilized an Extended Kalman Filter algorithm to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project [rubric](https://review.udacity.com/#!/rubrics/748/view). 

---

## Important Dependencies

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

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

# Results

## Visualisation

The following graph compares real and estimated values for car coordinates using data from Dataset 1

![alt text][EKF]

## RMSE

The accuracy requirement is that the algortihm should perform with RMSE error lower than some threshold values. This shown in tables below for both datasets:

Dataset 1:

| Parameter | RMSE | RMSE threshold |
|:---------:|:----:|:--------------:|
|x          |0.0974| 0.11           |
|y          |0.0855| 0.11           |
|Vx         |0.4517| 0.52           |
|Vy         |0.4404| 0.52           |

Dataset 2:

| Parameter | RMSE | RMSE threshold |
|:---------:|:----:|:--------------:|
|x          |0.0726| -           |
|y          |0.0967| -           |
|Vx         |0.4579| -           |
|Vy         |0.4966| -           |




