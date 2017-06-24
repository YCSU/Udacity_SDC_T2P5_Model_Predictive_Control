# Model Predictive Control

Model Predictive Control (MPC) is implemented for controlling a car driving around in a simulator. The idea of MPC is to predict the states of the car by considering the equations of motions of the car and minimizing the defined cost. The challenging part of this project is that the car does not actuate the commands instantly but with a latency of 100ms. We need to deal with the latency of the actuator commands.

---
## Model

To model the state of the car, we need to know not only the position, heading and velocity of the car but also how far away the car is from the reference track. The state of the car can be described by the following six variables:
```
(x, y, psi, v, cte, e_psi)
```
where x and y is the position of the car. psi is the orientaion angle with respect to the x axis of the world coordinate. v is the velocity of the car. cte is the cross track error, which is the distance from the vehicle's center of gravity to the reference track. The last variable e_psi is the difference between the angle of the car's heading and the designated angle infered from the reference track.

Consider the actuator commands, the throttle (a) and steering angle (delta), the equations of motions of the car is
```
x' = x + v * cos(psi) * dt
y' = y + v * sin(psi) * dt
psi' = psi + v / Lf * delta * dt
v' = v + a * dt
cte' = cte + v * sin(e_psi) *dt
e_psi' = e_psi + v / Lf * delta * dt
```
where Lf is the distance between the front of the vehicle and its center of gravity, and dt is the elapsed time duration. If the reference track is fitted to a polynomial f, the last two equations can be written as
```
cte' = f(x)-y + v * sin(e_psi) *dt
e_psi' = psi - derivative(f)(x) + v / Lf * delta * dt
```

## Fitting the waypoints and dealing with latency

The waypoints of the road in the world coordinate are given. The waypoints are fitted with a third-order polynomial. Before fitting the polynomial, the waypoints are transformed into the vehicle coordinate (main.cpp, line 110-115). It is easier to caculate the cross track error  and psi error in the vehicle coordinate.

To deal with the latency of 100ms, the initial state (x', y', psi', v') to be fed into the optimizer is calculated by the above equations of motions with the current state (x, y, psi, v) and dt = latency = 100ms (main.cpp, line 104-108).  
 
## Choosing the time step and elapsed duration
Now we can define the cost function with respect to cross track error, psi error and the change rate of them, etc. The cost function is defined in MPC.cpp (line 53-72). After the cost function is defined, the whole problem becomes a nonlinear programming problem which we need to find a set of future states that can minimize the cost function and satisfies the constraints given by the equations of motions.

To calculate the future states, we need to choose the number of time steps and elapsed time duration. We experimented with different combinations of the time steps N = (8, 10, 12, 15) and elapsed time duration dt= (0.1s, 0.12s, 0.15s, 0.2s). If dt is large, we are apprximating the track with larger steps. This is bad for sharp turns.If dt is too small, we may overfit the actuator commands. A proper value of the time steps N is chosen based on dt. The combination of the time steps length = 15 and elapsed time duration = 0.1s is the most stable one. Here is a [link](https://youtu.be/30RuOa4Eyxo) to the video of driving successfully around the track.


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
