### Omar benzakour
---

#MPC controller

## Goals

The goals / steps of this project are the following:

* Implement a Model Predictive Controller to derive acceleration and steering which will allow a car to drive autonomously.

[//]: # (Image References)
[kinematic]: ./imgs/kinematic.png
[cte]: ./imgs/cte.png

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from [here](https://github.com/udacity/self-driving-car-sim/releases)
* * **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`. 



## Description of the MPC controller

The Model Predictive Control (MPC) is a method of process control used to control a process (here a car) while satisting a set of constraints. The MPC optimizes a finite time horizon that is constituted of N timeslots. After the optimization step, the MPC implements the first timeslot and then optimises the next horizon. In this project the MPC controller optimizes the control input (steering and acceleration) of the car

### The Horizon

As we said the MPC controller optimizes the overall cost over N timeslots. In our project, we define the duration of the optimized trajectory T by equaly spliting the time-horizon into N timeslots spaced by &delta;t

### The reference trajectory

In this project, we are given points that define the overall desired trajectory. At an instant t, we compute the equation of a polynom of degree 3 that fits this trajectory. we will refer the this polynom by f

### Model parameters

The state of the car at a timeslot t is defined by 6 parameters:

* **x**: x position of the car
* **y** : y position of the car
* **v** : velocity of the car
* **psi** : orientation of the car
*  the **Cross Track Error (CTE)**. For the timeslot t, it represents the distance between the vehicle and the desired position at time t. 
*  The **orientation error (epsi)** which represents the angel between the desired position and the current position

The Control input is also part of the state of the car, indeed at timeslot t, we keep track of:

* steering angle (**theta**)
* the throttle / brake ( **acceleration of the vehicle** )

Therefore for a Hoziron composed of N steps we have:
 6*N + (N-1)*2 parameters

### Model

The following equations describe our model. They allow us to
go from timeslot t to timeslot (t+1):

![alt text][kinematic]




The following equations allow us to compute the errors at each timeslot (with f the polynom that fits the desired trajectory):

	cte(t) = y(t) - f(x(t)) 
	epsi(t) = psi(t) - arctan(f'(x(t)))



## Finding appropriate hyperparameters

### Tunning N and &delta;t

The MPC controller optimizes the N timeslots that constitute the horizon. Here are some remarks regarding N and &delta;t.

* The higher N is, the more processing power we need to run the algorithm. Indeed the higher N is, the higher is the number of paramaters we have to optimize
* The MPC optimizes the model over the whole horizon. However we only implement the first step of the model. If we use a high N the optimization might under optimize the first step. An example of this phenomenon can be seen during turns. Indeed because we overoptimize the last parameters the car is sometimes taking cuts
* The higher the term N*&delta;t is, the longer is trajectory that we optimize. Again, this means that we might underoptimize the first step which leads the car to go out of the track during turns
* &delta;t represents the time difference between 2 successive points. The smaller &delta;t is the more chance we give to the car to adjust the car's position. A small &delta;t might lead the car to overshoot and diverge

I have chosen these parameters in a way to keep the trajectory N*&delta;t small compared to a turn. Since I have set the desired speed at 40mph I could visualize the trajectory and adjust both parameters. I have started with N=10 and &delta;t=0.15 and reduced both parameter to have N=7 and &delta;t=0.11. When I see that the car diverges I increase &delta;t. When I see than the trajectory is too big I decrease N. I have also tunned the hyperparameters of the cost function. More explanations on the next paragraph

### Cost function

The cost function is the function that the MPC controller minimizes. The cost function sums **for each timeslot** the following parameters:

The **cte**:

	cost += 1000*CppAD::pow(vars[cte_start + t], 2);
	
The **orientation error**:

	cost += 1000*CppAD::pow(vars[epsi_start + t], 2);

The **distance between the reference speed**:

	cost += 10*CppAD::pow(vars[v_start + t] - ref_v, 2);
	
The **use of the actuators**:

	cost += 10*CppAD::pow(vars[delta_start + t], 2);
    cost += 10*CppAD::pow(vars[a_start + t], 2);
	
The **the value gap between sequential actuations.**:

	cost += 30000*CppAD::pow(vars[delta_start + t + 1] - 
			vars[delta_start + t], 2);
    cost += 100*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);


### Polynomial Fitting and MPC Preprocessing

The backend provides us with some points that constitute the desired trajectory. Before fittting a polynom to these points we project them into the car coordinate system. Therefore at instant t, the car is at the origin of the coordinate system (0,0). The CTE is therefore equal to f(0) and psie is equal to -arctan(f'(0)). The computation is therefore easy

### Model Predictive Control with Latency

a time t, we are provided with the whole state of the car. We can therefore use the kinematic equations and compute the state of the car at time (t + latency). From there we just replace our state by the state that we have just computed. 
