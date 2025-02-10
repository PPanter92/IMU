#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

void initMotorDriver();
float readBatteryVoltage();
float getBatteryVoltage(const uint32_t interval);
float getMeanBatteryVoltage(const uint32_t interval);
void steerHorizontalMode(const float steer[2], int16_t motorPwmOut[2]);
void setMotorPwm(const int16_t motorPwmIn[2]);

#endif