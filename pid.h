#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * PID structure with a fixed time step (dt-based PID).
 *
 * This structure stores parameters and state variables for a PID controller
 * that operates with a fixed time step (`dt`). The integral and derivative
 * components are computed based on a constant time interval provided externally.
 */
struct PID_DT {
  float kp, ki, kd;       // Proportional, Integral, and Derivative gains
  float lowerLimit, upperLimit; // Output limits to prevent excessive control values
  bool reverse;           // If true, inverts the output sign
  float integral;         // Integral accumulator for I-term
  float previousError;    // Stores the last error value for derivative calculation
};

/**
 * PID structure with an adaptive time step (interval-based PID).
 *
 * This structure is used for a PID controller that updates only when
 * a specified interval has elapsed. The time step (`dt`) is dynamically 
 * calculated based on `millis()`, ensuring it adapts to varying execution timings.
 */
struct PID_INTERVAL {
  float kp, ki, kd;       // Proportional, Integral, and Derivative gains
  float lowerLimit, upperLimit; // Output limits to prevent excessive control values
  bool reverse;           // If true, inverts the output sign
  uint32_t lastUpdate;    // Stores the last update timestamp (milliseconds)
  float integral;         // Integral accumulator for I-term
  float previousError;    // Stores the last error value for derivative calculation
};

float pidController(float setpoint, float actual, PID_DT& pid, float dt);
float pidController(float setpoint, float actual, PID_INTERVAL& pid, uint32_t interval);

#endif