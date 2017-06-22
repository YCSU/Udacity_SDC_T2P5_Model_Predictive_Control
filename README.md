# Model Predictive Control

Model Predictive Control (MPC) is implemented for controlling a car driving around in a simulator. The idea of MPC is to predict the states of the car by considering the equations of motions of the car and minimizing the defined cost. The challenging part of this project is that the car does not actuate the commands instantly but with a latency of 100ms. We need to deal with the latency of the actuator commands.

---
## Model

The state of the car can be described by the following four variables:
```
(x, y, psi, v)
```
where x and y is the position of the car. psi is the orientaion angle with respect to the x axis of the world coordinate. v is the velocity of the car. Consider the actuator commands, the throttle (a) and steering angle (delta), the equations of motions of the car is
```
x' = x + v * cos(psi) * dt
y' = y + v * sin(psi) * dt
psi' = psi + v / Lf * delta * dt
v' = v + a * dt
```
where Lf is the distance between the front of the vehicle and its center of gravity, and dt is the elapsed time duration.


## Fitting the waypoints

## Choosgin the time step and elapsed duration

## Dealing with latency

## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* [Ipopt](https://projects.coin-or.org/Ipopt)
* [CppAD](https://www.coin-or.org/CppAD/)
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
