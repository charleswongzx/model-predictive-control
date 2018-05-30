# CarND-Controls-MPC
Adapted and Built for the Self-Driving Car Engineer Nanodegree Program

---
This is an implementation of a Model Predictive Controller for vehicle steering and throttle control at speed using C++.
Designed to run with the Udacity Simulator at a 100 ms actuator latency and broadcast latency over a websocket with 2 local processes.

## Results
The car kept on track quite well, with a fairly smooth driving style and a decent adherence to the ground truth.
It reached speeds of 90+ mph, which was the highest I'd ever achieved whether with behavioural cloning as in term 1, or the pid controller in an earlier project.

![udacity trial](media/result.png)

## Challenges
- Correcting for the added latency was fairly straightforward, and simply involved adding 100 ms to the predicted state.
- Tuning delta_t length and frequency was more involved, and required multiple runs before settling on optimum parameters.
I initially thought more sampling would be better as long as my PC could handle the calculations, and set N = 30, dt = 0.05.
This made the vehicle extremely twitchy however, and its amplitude of oscillation eventually caused it to veer off track.
- Tuning weights for FG_eval cost functions were the most tricky. I had a lot of help from the Udacity forum.
I found that penalising CTE and epsi heavily were necessary for keeping the car on track, as lower values meant the vehicle somtimes left the circuit entirely.
This introduced jerky directional change however, and it was necessary to penalise delta_change to ensure a more natural driving style.


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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.