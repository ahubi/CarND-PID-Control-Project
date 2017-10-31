# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

[1]: http://www.ni.com/white-paper/3782/en/
[2]: https://en.wikipedia.org/wiki/PID_controller

### PID description

Proportional-Integral-Derivative (PID) control is the most common control algorithm used in industry and has been universally accepted in industrial control. The popularity of PID controllers can be attributed partly to their robust performance in a wide range of operating conditions and partly to their functional simplicity, which allows engineers to operate them in a simple, straightforward manner. [[1]]

PID algorithm consists of three basic parts:

* P - Proportional
* I - Integral
* D - Derivative

Each part can be controlled with a gain (K).

#### Proportional
This part is directly proportional to the error returned by the system.
Increasing the proportional gain will increase the speed of the system response.
However, if the proportional gain is too large, the process variable will begin to oscillate.
Increasing the gain further will make the system unstable and get out of control.
In this project inaccurately chosen value for proportional gain leads to quick overshoots of the car.

#### Integral
The integral component sums the error term over time.
The goal with integral part is to remove the Systematic Bias of a system.

#### Derivative
The derivative component considers the rate of change of error and is
trying to bring this rate to zero [[2]] In this project the rate of change is
calculated by error(t) - error(t-1). Time is assumed to be always constant and
therefore omitted.

### PID implementation
In this project the simulator returns a so called Cross Track Error (CTE) to
provide feedback about car position in relation to car desired Track position.
With provided CTE the PID controller calculates a new steering value and provides
it back to the simulator which corrects car's track position.
This steps are continuously repeated.

##### Update step
Here the CTE error is provided by the simulator to the PID controller.
It calculates each error part (P, I, D) to be ready for total error calculation.
```
void PID::UpdateError(double cte) {
  de_  = cte - pe_; //calculate derivative error
  pe_  = cte;       //set proportional error
  ie_ += cte;       //calculate integral error
}
```
##### Total error
Here the total error is calculated and returned to the simulator.
```
double PID::TotalError() {
  double ret = -Kp_ * pe_ - Kd_ * de_ - Ki_ * ie_;
  //Normalize to [-1,1]
  while (ret > 1) ret -=1;
  while (ret < -1) ret +=1;
  return ret;
}

```
##### Initialization step
This step is performed only once during setup of the PID controller.
The passed gains Kp, Ki and Kd are identified during Tuning process.
```
void PID::Init(const double& kp, const double& ki, const double& kd) {
  Kp_ = kp;
  Ki_ = ki;
  Kd_ = kd;
}
```

### PID Tuning
To achieve optimal car driving behavior the gains must be selected. There are
different tuning approaches to identify proper gains of a system.
In this project I used manual tuning to find out gains to keep the car on track.
After experiments with the simulator and trying on two different hardware machines
Quadcore desktop CPU and laptop dualcore CPU following gains were finally
chosen

```
pid.Init(0.1, 0.0001, 0.6);

```

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
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake ../src && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)
* This project can be used with Eclipse, see ide_profiles/Eclipse description.

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.
