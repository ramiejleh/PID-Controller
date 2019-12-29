#include "PID.h"

/**
 * Done: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Done: Initialize PID coefficients (and errors, if needed)
   */
  
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  p_error = 0;
  i_error = 0;
}

void PID::UpdateError(double cte) {
  /**
   * Done: Update PID errors based on cte.
   */
  // d_error is difference between prev cte (p_error) and the current one
  d_error = (cte - p_error);
  // p_error is the previous error
  p_error = cte;
  // i_error is the sum of all CTEs until now
  i_error += cte;
}

double PID::TotalError() {
  /**
   * Done: Calculate and return the total error
   */
  return -Kp * p_error - Kd * d_error - Ki * i_error;  // Done: total error
}