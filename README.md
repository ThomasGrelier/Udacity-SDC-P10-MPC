# MPC project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview
This repository contains the work I did within **Project #10 of Udacity's Self-Driving Car Nanodegree Program** (fifth project of second term). Objective of this project is to use **Model Predictive Control** to have a car navigate on a simulator track. Implementation is done in **C++**. 

For this we use the car driving simulator that Udacity has developed and that we already used for Project #3 (Behavorial Cloning) and Project #9 (PID controller). For that project, the objective is still the same (drive the car around the car) but using Model Predictive Control approach. 

*Snapshot of Udacity's driving simulator*

![](./simulator.png)


## Project writeup

### Introduction 
In this project we implement MPC to maintain the car on the track. 

The simulator provides:
* position (x,y) in m and orientation (psi) in rad of the car in the map frame
* velocity (mph)
* waypoints of reference trajectory to follow (x1, .., xN, y1, .., yN)

These values are fed in the MPC algorithm to compute the appropriate steering angle and throttle, which are sent back to the simulator.

There is no requirement on car speed. I started with the speed equal to 40 mph while the maximum possible speed is 100 mph. The higher the speed the more difficult it is to remain on track! 

A latency of 100 ms is considered. It corresponds to the delay between the sending of the actuator commands and their application.

### MPC presentation
Model Predictive Control consists in solving an optimization problem. The idea is to find the actuator commands (steering angle, throttle) over a given time interval in the future that will best enable to follow a reference trajectory (which is computed and provided by the path planning module).
This optimization problem tries to minimize a cost function which depends on several parameters (cross track error (CTE), yaw error, speed error...), while coping with some constraints on actuator boundaries and actuator change rate. When the best combination of parameters is found, the first set of actuator commands are applied and the others are discarded. The optimization is repeated at each time step.
 
 ### MPC parameters
 The hyper parameters of the MPC algorithms are:
 - Duration of the trajectory
 - Time step
 - Vehicle evolution model
 - Model constraints
 - Cost function
 
Choice of horizon and sampling time is a tradeoff between computer time and accuracy. The number of variables of the optimization problem is proportional to the number of steps. So it is important to limit this value.
The vehicle model I chose it the one we studied in class: the bicycle kinematic model. It contains 6 parameters (x, y, velocity, yaw, CTE, yaw error).
Some constraints are put on the actuator values: +/- 25° on the steering angle and +/-1 on throttle value (that is full throttle and full brake)
As for the cost function, it the weighted square sum of the following terms:
* CTE
* Yaw error
* Throttle command
* Steering angle command
* Throttle difference between two consecutive time steps
* Steering angle difference between two consecutive time steps

The last two constraints are introduced so as to get a smooth driving: smooth turns, smooth acceleration and braking. 
 
### Coping with latency
In a real car, an actuation command won't execute instantly - there will be a delay, called latency, as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds. 
It's a difficult challenge for some controllers - like a PID controller - to overcome. But a Model Predictive Controller can adapt quite well because we can model this latency in the system.
One way to take into latency into account is to propagate the vehicle state  into the future (100 ms away) using the vehicle evolution model, then apply MPC from there. This is the solution I implemented.

### MPC tuning 
Tuning a controller consists in finding the values of the hyperparameters.
For horizon and time step, I started with 10 steps of 100ms, that is a horizon time of 1 second. I then tuned my cost function, looking for the weights that would make my car stay on the road.
Starting with all 6 weights to one, my car was wobbling a lot. To smooth the trajectory I increased the weights associated to yaw error, steer angle and steer angle difference to 10. It provided better results but the car still got out of the road at the first turn. I then significantly increased the weight associated to the steering angle difference, moving it to 100. The car was then able to drive round the track. There still was a lot of throttle/brake commands, so to reduce it I increased the weight associated to throttle command difference to 50.
This previous tuning was done for a speed of 40 mph. I tested how it behaved for 50 mph. Well, the car started to oscillate and soon crashed. After increasing the weight for the differential steering angle command to 200 and reducing the weight of the steering angle to 1 (so as to turn sharper), the car made it.
I finally tried 60 mph with this new tuning but it did not pass.

We can see that the tuning is very sensitive to weights of the cost function. Driving very fast indeed requires a very fine tuning.
It would certainly also require to play on horizon and time steps. I tried to modify these parameters and I could notice the car did not behave the same at all when increasing time step or horizon, requiring a new weight tuning!


## Repository content

The repository includes the following files:

 - source code in the */src* folder
	 - main.cpp : communicates with the Simulator, receiving car poise (position, yaw, speed) and reference trajectory (waypoints), runs the MPC and sends back actuator commands (steering angle and throttle). 
	 - MPC.cpp:  implements MPC. Returns MPC trajectory and actuator commands
 	 - json.hpp: JSON is used for communication with simulator
	 - Eigen3-3/: repository containing Eigen library used for matrix calculus

 - CMakeLists.txt: file that is used by CMAKE to build the project 

## Dependencies

* cmake >= 3.5
* make >= 4.1(mac, linux), 3.81(Windows)
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* Simulator: it can be download [here](https://github.com/udacity/self-driving-car-sim/releases).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`
 

