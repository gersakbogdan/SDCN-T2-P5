# Self-Driving Car Engineer Nanodegree Program

## The model
Model Predictive Controller uses a Kinematic model that emulates the vehicle controls dynamics as closely as possible.
The state, actuators and update equations are described below:

 * State is given by: { px position, py position, orientation (psi), speed (v), cross track error (cte), orientation error (epsi)}
 * Actuators: { steering angle (delta), acceleration (a)}
 * Prediction equations:
   * x1 = x0 + v0 * cos(psi0) * dt
   * y1 = y0 + v0 * sin(psi0) * dt
   * psi1 = psi0 + v0 / Lf * delta0 * dt
   * v1 = v0 + a0 * dt

## Timestep Length and Elapsed Duration (N & dt)
First I started with the values used in the lesson,  `N = 25, dt = 0.05` but the vehicle was unstable and went off of track at the first turn.
Next I changed the reference speed to 50 mph and started to play with different values for `N` and `dt`.
After a few tries I saw that vehicle is quite stable for values of `N` which were between `10 - 12` and `dt = 0.1`
Using a reference speed of 80 mph the model performed with the following values: `N = 12, dt = 0.1` and this were the values I choosed to use in the end.

## Polynomial Fitting and MPC Preprocessing
Because `ptsx` and `ptsy` provieded by the simulator were in map coordinate, in order to use the values in our model
I had to tranform the points into vehicle coordinates using the following equations:
 * wptsx[i] = x_trans * cos(-psi) - y_trans * sin(-psi);
 * wptsy[i] = x_trans * sin(-psi) + y_trans * cos(-psi);

## Model Predictive Control with Latency
In order to test our model like on a real vehicle, where latency between the actuation command and the physical actuation exists, `latency` value was set up as `100ms`.
To handle this our vehicle coordinations, orientation and speed were calculated using the following equations:
 * px += v * cos(psi) * latency;
 * py += v * sin(psi) * latency;
 * psi -= v * delta / Lf * latency;
 * v += a * latency;

To account for this latency the car's state was estimated after the latency time before being evaluated by the MPC.
This resulted in the controller providing actuation command for a future time aligned with the latency.
This was done on lines 109-116 of main.cpp.
The same kinematic model was used for this prediction as is used in the MPC.



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

or

1. `./run.sh`
