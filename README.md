# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

## Background
A Kalman filter can be used to estimate the state of a system. In the case of Self-Driving cars, the use case for example is that of the Self Driving car tracking the state of another moving vehicle. The "State" of that moving vehicle can be denoted by px, py, vx, vy (Positions & Velocity in X & Y directions). These state variables may not be directly observable, so they need to be estimated via LIDAR & RADAR measurements taken from sensors on the Self-Driving Car. 

## Kalman Filter Intuition (From Udacity Lecture)
The Kalman equation contains many variables, so here is a high level overview to get some intuition about what the Kalman filter is doing.

Prediction
Let's say we know an object's current position and velocity , which we keep in the x variable. Now one second has passed. We can predict where the object will be one second later because we knew the object position and velocity one second ago; we'll just assume the object kept going at the same velocity.

The x' =Fx+ν equation does these prediction calculations for us.

But maybe the object didn't maintain the exact same velocity. Maybe the object changed direction, accelerated or decelerated. So when we predict the position one second later, our uncertainty increases. P' =FPFT+Q represents this increase in uncertainty.

Process noise refers to the uncertainty in the prediction step. We assume the object travels at a constant velocity, but in reality, the object might accelerate or decelerate. The notation \nu \sim N(0, Q)ν∼N(0,Q) defines the process noise as a gaussian distribution with mean zero and covariance Q.

Update
Now we get some sensor information that tells where the object is relative to the car. First we compare where we think we are with what the sensor data tells us y = z - Hx'y=z−Hx 

The K matrix, often called the Kalman filter gain, combines the uncertainty of where we think we are P with the uncertainty of our sensor measurement R. If our sensor measurements are very uncertain (R is high relative to P'), then the Kalman filter will give more weight to where we think we are: x'
′
If where we think we are is uncertain (P' is high relative to R), the Kalman filter will put more weight on the sensor measurement: z.

Measurement noise refers to uncertainty in sensor measurements. The notation \omega \sim N(0, R)ω∼N(0,R) defines the measurement noise as a gaussian distribution with mean zero and covariance R. Measurement noise comes from uncertainty in sensor measurements.


## Kalman Filter Algorithm Psuedo-Code:
Prediction:
"F" represents State transition matrix, which when multiplied with current X [position, velocity] returns new X.
For 1 dimension
F= [1 Δt
    0 1]
X=[px;vx]

This can be extended to more dimensions.
X=[px;py;vx;vy]


x0 = F x + u
"P" represents state uncertainty. Q represents process uncertainty. In this case, we're only modeling position & velocity, so Q is modeling acceleration as random noise.
P is diagonal, choose larger values for P if highly uncertain of initial position.

P0 = F P F T + Q

Measurement Update:
'y' represents the error
"H" represents the measurement function. When you measure LIDAR, you only have position info not velocity. H matrix helps zero out the velocity component.

y = z − Hx0

"S" represents measurement uncertainty, with R measurement noise
S = HP0HT + R
"K" represents Kalmann gain
K = P0HT S−1
"X" update - depending on whether measurement uncertainty versus a-prior uncertainty, x0 or y term will dominate the update
x = x0 + Ky
P = (I − KH)P0

Intuition: 
If measurement covariance [Q] is smaller than the A-prior estimate covariance P, then measurement will dominate. If Q is larger, then the A-priori estimate will dominate

LIDAR measurements provide (noisy) 
L 3.122427e-01  5.803398e-01  1477010443000000  6.000000e-01  6.000000e-01  5.199937e+00  0 0 6.911322e-03

For laser sensors, we have a 2D measurement vector. Each location component px, py are affected by a random noise.R is the measurement noise covariance matrix. Typically provided by manufacturer.

RADAR measurements provide (noisy)
R 1.014892e+00  5.543292e-01  4.892807e+00  1477010443050000  8.599968e-01  6.000449e-01  5.199747e+00  1.796856e-03  3.455661e-04  1.382155e-02

For radar measurements, we have a 3D vector representing radial distance, angle (between  vertical axis X and the heading vector) and radial velocity. 
The H matrix for Radar (similar to H matrix for LIDAR) is a function which maps radial to x,y coordinates.
The H matrix is non-linear. TO linearize, use Taylor expansion and take derivatives, expressed as Jacobian. The Jacobian calculates partial derivatives of rho, phi and radial velocity with respect to px,py,vx,vy.

The Kalman Filter equation is same for RADAR except, take the Jacobian fo F & H instead of F & H as in the LIDAR equations.

To check accuracy, use Root Mean Square Error cumulatively, but separately for px,py,vx,vy. Evaluate versus "Ground Truth " X,Y values, which are provided with each measurement.

## Input Data File Description
Each row represents a sensor measurement where the first column tells you if the measurement comes from radar (R) or lidar (L).

For a row containing radar data, the columns are: sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

For a row containing lidar data, the columns are: sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

Whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y).

first measurement - the filter will receive initial measurements of the bicycle's position relative to the car. These measurements will come from a radar or lidar sensor.
initialize state and covariance matrices - the filter will initialize the bicycle's position based on the first measurement.

then the car will receive another sensor measurement after a time period \Delta{t}Δt.

predict - the algorithm will predict where the bicycle will be after time \Delta{t}Δt. One basic way to predict the bicycle location after \Delta{t}Δt is to assume the bicycle's velocity is constant; thus the bicycle will have moved velocity * \Delta{t}Δt. In the extended Kalman filter lesson, we will assume the velocity is constant.

update - the filter compares the "predicted" location with what the sensor measurement says. The predicted location and the measured location are combined to give an updated location. The Kalman filter will put more weight on either the predicted location or the measured location depending on the uncertainty of each value.

then the car will receive another sensor measurement after a time period \Delta{t}Δt. The algorithm then does another predict and update step.

## Goals
px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt" which is the same data file the simulator uses for Dataset 1.

## Extended Kalman Filter Implementation Algorithm:
Here is the output video of the Kalman filter
![Kalman Video](https://raw.githubusercontent.com/eshnil2000/Udacity-CarND-Extended-Kalman-Filter-Project/master/images/kalman.gif)

# Initialization of Matrices : [FusionEKF.cpp](https://raw.githubusercontent.com/eshnil2000/Udacity-CarND-Extended-Kalman-Filter-Project/master/src/FusionEKF.cpp)

```c

//measurement covariance matrix - laser
R_laser_ << 0.0225, 0, 0, 0.0225;

//measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,0, 0.0009, 0,0, 0, 0.09;
//Measurement matrix - laser (shape: 2x4)
  H_laser_ << 1, 0, 0, 0,0, 1, 0, 0;

//x_ is state vector representing positionx, positiony, velocityx, velocityy
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0, 0;

    //Set first measurements to get started
    float first_measurement_x = measurement_pack.raw_measurements_[0];
    float first_measurement_y = measurement_pack.raw_measurements_[1];

/**
     *Initializing State uncertainity matrix. Set uncertainty for velocity about 10X that of position
     */
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 10, 0, 0, 0,0, 10, 0, 0,0, 0, 1000, 0, 0, 0, 0, 1000;

```

#General instructions
In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `


If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric




