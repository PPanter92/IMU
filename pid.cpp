#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include <Arduino.h>
#include "pid.h"

float pidController(float setpoint, float actual, PID_DT& pid, float dt) {
  float error = setpoint - actual;
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, pid.lowerLimit, pid.upperLimit);
  float derivative = (error - pid.previousError) / dt;
  pid.previousError = error;
  float output = (pid.kp * error) + (pid.ki * pid.integral) + (pid.kd * derivative);
  if (pid.reverse == true) {
    output = -output;
  }
  return constrain(output, pid.lowerLimit, pid.upperLimit);
}


float pidController(float setpoint, float actual, PID_INTERVAL& pid, uint32_t interval) {
  uint32_t currentMillis = millis();
  
  if (currentMillis - pid.lastUpdate < interval) {
    return NAN;
  }
  
  float dt = (currentMillis - pid.lastUpdate) / 1000.0f; // `dt` in Sekunden berechnen
  pid.lastUpdate = currentMillis;

  float error = setpoint - actual;
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, pid.lowerLimit, pid.upperLimit);
  float derivative = (error - pid.previousError) / dt;
  pid.previousError = error;
  float output = (pid.kp * error) + (pid.ki * pid.integral) + (pid.kd * derivative);

  if (pid.reverse) {
    output = -output;
  }

  return constrain(output, pid.lowerLimit, pid.upperLimit);
}


#endif