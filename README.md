# CarND-Controls-MPC
Adapted and Built for the Self-Driving Car Engineer Nanodegree Program

---
This is an implementation of a Model Predictive Controller for vehicle steering and throttle control at speed using C++.
Designed to run with the Udacity Simulator at a 100 ms actuator latency and broadcast latency over a websocket with 2 local processes.

## Results
The car kept on track quite well, with a fairly smooth driving style and a decent adherence to the ground truth.
It reached speeds of 90+ mph, which was the highest I'd ever achieved whether with behavioural cloning as in term 1, or the pid controller in an earlier project.

![udacity trial](media/result.png)

## Implementation
### main.cpp
#### Coordinate Conversion
Given that the ground truth path was provided by the simulator in map coordinates, it was important to first convert the path into coordinates relative to the car. This is taken care of by this simple formula:
```cpp
// Transform the points to the vehicle's orientation
          for (int i = 0; i < ptsx.size(); i++) {
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;
            ptsx_car[i] = x * cos(-psi) - y * sin(-psi);
            ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
          }
```
#### Error Calculation
The vehicle-based points found from coordinate conversion need to first be fit to 3rd-degree polynomial, which is a good enough approximation for the line formed. Coefficients are evaulated and used to find cross-track error and orientation error:
```cpp
auto coeffs = polyfit(ptsx_car, ptsy_car, 3);  // 3rd-order polynomial fitting to car coords
double cte = polyeval(coeffs, 0);  // cross track error calculation
double epsi = -atan(coeffs[1]);  // orientation error
```
#### State Prediction (Incl. Latency)
Here begins the state prediction step, where the vehicle's future position, orientation, speed and error are calculated for the following timestep. Here is where I accounted for the 100ms actuator latency imposed by the simulator, which mimics the latency one might encounter for the controls in a real vehicle.
```cpp
// Lf and dt for future state prediction
const double Lf = 2.67;
const double dt = 0.1;  // 100 ms latency added to predicted state

// Predict state after latency
double pred_px = 0.0 + v * dt; // Since psi is zero, cos(0) = 1, can leave out
const double pred_py = 0.0; // Since sin(0) = 0, y stays as 0 (y + v * 0 * dt)
double pred_psi = 0.0 + v * -delta / Lf * dt;
double pred_v = v + a * dt;
double pred_cte = cte + v * sin(epsi) * dt;
double pred_epsi = epsi + v * -delta / Lf * dt;

Eigen::VectorXd state(6);
state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;


```
#### Steering and Throttle Calculation
This step invokes the solver used to optimise the cost function outlined in mpc.cpp. The values returned from the solver are sent to the simulator for actuation.

```cpp
// Solve for new actuations (and to show predicted x and y in the future)
auto vars = mpc.Solve(state, coeffs);

// Calculate steering and throttle
          double steer_value = vars[0] / (deg2rad(25) * Lf);
          double throttle_value = vars[1];
```
### MPC.cpp
#### Assigning Cost Weights
The MPC is all about optimising a cost function, which includes penalties for:
- Deviation from the recommended path (CTE)
- Orientation error (EPSI)
- Changes in speed (v)
- Steering (delta)
- Acceleration (a)
- Changes in steering and acceleration (delta and a change)
Tuning this was tricky. More elaboration under challenges below.

The associated costs were tabulated and added to fg[0], where the total cost of the model was stored. Here the CppAD library was used to compute derivatives.
```cpp
// Weights
const int cte_cost_weight = 2000;
const int epsi_cost_weight = 2000;
const int v_cost_weight = 1;
const int delta_cost_weight = 10;
const int a_cost_weight = 10;
const int delta_change_cost_weight = 100;
const int a_change_cost_weight = 10;

// Cost for CTE, psi error and velocity
for (int t = 0; t < N; t++) {
  fg[0] += cte_cost_weight * CppAD::pow(vars[cte_start + t], 2);
  fg[0] += epsi_cost_weight * CppAD::pow(vars[epsi_start + t], 2);
  fg[0] += v_cost_weight * CppAD::pow(vars[v_start + t] - ref_v, 2);
}

// Costs for steering (delta) and acceleration (a)
for (int t = 0; t < N-1; t++) {
  fg[0] += delta_cost_weight * CppAD::pow(vars[delta_start + t], 2);
  fg[0] += a_cost_weight * CppAD::pow(vars[a_start + t], 2);
}

// Costs related to the change in steering and acceleration (makes the ride smoother)
for (int t = 0; t < N-2; t++) {
  fg[0] += delta_change_cost_weight * pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  fg[0] += a_change_cost_weight * pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```

#### Setting Up Constraints
The solver model requires constraints, not least because of physical limitations (steering can only go 25 deg), but also because a solution must be converged upon within reasonable limits. Setting up constraints is performed like so:
```cpp
// Initial constraints
// We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
// This bumps up the position of all the other values.
fg[1 + x_start] = vars[x_start];
fg[1 + y_start] = vars[y_start];
fg[1 + psi_start] = vars[psi_start];
fg[1 + v_start] = vars[v_start];
fg[1 + cte_start] = vars[cte_start];
fg[1 + epsi_start] = vars[epsi_start];

// The rest of the constraints
for (int t = 1; t < N; t++) {
  // State at time t + 1
  AD<double> x1 = vars[x_start + t];
  AD<double> y1 = vars[y_start + t];
  AD<double> psi1 = vars[psi_start + t];
  AD<double> v1 = vars[v_start + t];
  AD<double> cte1 = vars[cte_start + t];
  AD<double> epsi1 = vars[epsi_start + t];

  // State at time t
  AD<double> x0 = vars[x_start + t - 1];
  AD<double> y0 = vars[y_start + t - 1];
  AD<double> psi0 = vars[psi_start + t - 1];
  AD<double> v0 = vars[v_start + t - 1];
  AD<double> cte0 = vars[cte_start + t - 1];
  AD<double> epsi0 = vars[epsi_start + t - 1];

  // Actuator constraints at time t only
  AD<double> delta0 = vars[delta_start + t - 1];
  AD<double> a0 = vars[a_start + t - 1];

  AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
  AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*pow(x0,2));

  // Setting up the rest of the model constraints
  fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
  fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
  fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
  fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
  fg[1 + cte_start + t] = cte1 - ((f0-y0) + (v0 * CppAD::sin(epsi0) * dt));
  fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_des0) - v0 * delta0 / Lf * dt);
}
```
#### Solve()
Finally, the actual solving of the optimsation problem. Here, we use the Ipopt library to find locally optimal values.
Ipopt treats the entire problem as a matrix of variables and constraints, so most of the work is setting up our number of vars and constraints, defining our upper/lower bounds. We call the FG_eval function built above to compute our objectives and constraints, before proceeding to actually run the solver. The solution is returned as a vector of predicted x and y values to be plotted into the simulator, as well as our first actuator values.
```cpp
vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Setting the number of model variables (includes both states and inputs).
  // N * state vector size + (N - 1) * 2 actuators (For steering & acceleration)
  size_t n_vars = N * state.size() + (N - 1) * 2;
  // Setting the number of constraints
  size_t n_constraints = N * state.size();

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Sets lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Start lower and upper limits at current values
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Returns the first actuator values, along with predicted x and y values to plot in the simulator
  vector<double> solved;
  solved.push_back(solution.x[delta_start]);
  solved.push_back(solution.x[a_start]);
  for (int i = 0; i < N; ++i) {
    solved.push_back(solution.x[x_start + i]);
    solved.push_back(solution.x[y_start + i]);
  }
  return solved;
}
```

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
