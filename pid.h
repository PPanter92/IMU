#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

struct PID_DT {
  float kp, ki, kd;
  float lowerLimit, upperLimit;
  bool reverse;
  float integral;
  float previousError;
};

struct PID_INTERVAL {
  float kp, ki, kd;
  float lowerLimit, upperLimit;
  bool reverse;
  uint32_t lastUpdate;
  float integral;
  float previousError;
};

float pidController(float setpoint, float actual, PID_DT& pid, float dt);
float pidController(float setpoint, float actual, PID_INTERVAL& pid, uint32_t interval);

#endif