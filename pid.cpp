#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include <Arduino.h>
#include "pid.h"

/**
 * PID Controller function using a fixed time step (dt).
 * 
 * This function implements a PID (Proportional-Integral-Derivative) controller 
 * to adjust a process variable toward a desired setpoint.
 *
 * @param setpoint Desired target value.
 * @param actual Current measured value.
 * @param pid PID_DT struct containing PID parameters and state variables.
 * @param dt Time step (seconds) for integration and differentiation.
 * @return The computed control output, constrained within defined limits.
 */
float pidController(float setpoint, float actual, PID_DT& pid, float dt) {
  float error = setpoint - actual; // Compute error between setpoint and actual value

  // Compute integral term with anti-windup by limiting accumulation
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, pid.lowerLimit, pid.upperLimit);

  // Compute derivative term (rate of change of error)
  float derivative = (error - pid.previousError) / dt;
  pid.previousError = error; // Store current error for next cycle

  // Compute PID output using standard formula
  float output = (pid.kp * error) + (pid.ki * pid.integral) + (pid.kd * derivative);

  // Reverse output if required
  if (pid.reverse == true) {
    output = -output;
  }

  // Constrain output within specified limits
  return constrain(output, pid.lowerLimit, pid.upperLimit);
}

/**
 * PID Controller function with adaptive time step (interval-based).
 * 
 * This function executes a PID controller only if the specified time interval
 * has passed since the last update. It calculates `dt` dynamically based on
 * the elapsed time.
 *
 * @param setpoint Desired target value.
 * @param actual Current measured value.
 * @param pid PID_INTERVAL struct containing PID parameters and state variables.
 * @param interval Minimum update interval (milliseconds).
 * @return The computed control output (constrained), or NaN if the interval has not elapsed.
 */
float pidController(float setpoint, float actual, PID_INTERVAL& pid, uint32_t interval) {
  uint32_t currentMillis = millis(); // Get current time

  // Ensure minimum update interval has passed
  if (currentMillis - pid.lastUpdate < interval) {
    return NAN; // Return NaN to indicate no update was performed
  }

  // Compute time difference (dt) in seconds
  float dt = (currentMillis - pid.lastUpdate) / 1000.0f;
  pid.lastUpdate = currentMillis; // Update last execution timestamp

  float error = setpoint - actual; // Compute error between setpoint and actual value

  // Compute integral term with anti-windup
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, pid.lowerLimit, pid.upperLimit);

  // Compute derivative term (rate of change of error)
  float derivative = (error - pid.previousError) / dt;
  pid.previousError = error; // Store current error for next cycle

  // Compute PID output
  float output = (pid.kp * error) + (pid.ki * pid.integral) + (pid.kd * derivative);

  // Reverse output if required
  if (pid.reverse) {
    output = -output;
  }

  // Constrain output within specified limits
  return constrain(output, pid.lowerLimit, pid.upperLimit);
}


#endif