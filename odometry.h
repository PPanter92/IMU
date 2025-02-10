// Dateiname: odometry.h
#ifndef ODOMETRY_H
#define ODOMETRY_H

void initOdometry();
void getSimpleWay(float wayArray[2]);
bool getSimpleSpeed(float speedArray[2], uint32_t interval);

bool getImuData(float accArray[3], float gyrArray[3], const uint32_t interval);
void calibrateGyr(uint16_t samples = 1000, uint32_t interval = 10);
bool getAlphaAngle(float& alphaAngle, const float factor=0.995, const uint32_t interval=100);
bool getSimpleOdom(float odom[3], uint32_t interval);

#endif