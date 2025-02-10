#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

float getBatteryVoltage(const uint32_t interval);
void initMotorDriver();
void steerWheelchairMode(const float steer[2], int16_t motorPwmOut[2]);
void setMotorPwm(const int16_t motorPwmIn[2]);
// float pidController(float setpoint, float actual, PID &pid, float dt);

#endif