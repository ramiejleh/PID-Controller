# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## Implementation

In PID.cpp:

The init method is implemented to initialize PID coefficients and errors.
```c++
void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  p_error = 0;
  i_error = 0;
}
```

The UpdateError method is implemented to update PID errors based on the crosstrack error.

```c++
void PID::UpdateError(double cte) {
  // d_error is difference between prev cte (p_error) and the current one
  d_error = (cte - p_error);
  // p_error is the previous error
  p_error = cte;
  // i_error is the sum of all CTEs until now
  i_error += cte;
}
```

The TotalError method is implemented to calculate and return the total error
```c++
double PID::TotalError() {
  return -Kp * p_error - Kd * d_error - Ki * i_error;  // Done: total error
}
```
## The Effects of the PID Coefficients
Each of these Coefficients is critical to make a versatile controller.  
`Kp` is a proportional factor to the cross track error. The higher the number, the faster the oscillation and the overshooting becomes larger.  
`Kd` is a proportional factor to the differnce between the previous CTE and the current one. It important to prevent overshooting. The higher the number, the slower it is to converge though it converges more smoothly. Too low of a number will not prevent overshooting anymore.  
`Ki` is a proportional factor to the sum of all CTEs until that point. This will help correct any syetematic bias. 

They all work as expected and have been chosen after some manual trial and error and tuning

