// Dateiname: odometry.cpp
#ifndef ODOMETRY
#define ODOMETRY

#include <Arduino.h>
#include <pio_encoder.h>
#include "odometry.h"
#include <Wire.h>
#include <MPU6050_tockn.h>

#define ENC_L 12
#define ENC_R 14
PioEncoder EncoderL(ENC_L);
PioEncoder EncoderR(ENC_R);

#define IMU_SDA 4
#define IMU_SCL 5
MPU6050 MyIMU(Wire);

static float calibGyr[3] = {8.4f, 4.02f, -0.55f};

void initOdometry() {
  EncoderL.begin();
  EncoderR.begin();
  EncoderR.flip();
  Wire.setSDA(IMU_SDA);
  Wire.setSCL(IMU_SCL);
  Wire.begin();
  Wire.setClock(400 * 1000);
  MyIMU.begin();
  calibrateGyr(3000,1);
}

void getSimpleWay(float wayArray[2]) {
  const float diaWheel = 0.09f;
  const float countsPerRevolution = 1440.0f;
  const float wheelFactor = diaWheel * PI / countsPerRevolution;
  wayArray[0] = EncoderL.getCount() * wheelFactor;
  wayArray[1] = EncoderR.getCount() * wheelFactor;
}

bool getSimpleSpeed(float speedArray[2], uint32_t interval) {
    static float lastWay[2] = { 0.0f }, currentWay[2] = { 0.0f };

    static uint32_t lastMillis = millis();
    uint32_t currentMillis = millis();
    if (currentMillis - lastMillis > interval) {
      float dt = (currentMillis - lastMillis) / 1000.0f;
      lastMillis = currentMillis;
      getSimpleWay(currentWay);

      if (dt > 0) {
          speedArray[0] = (currentWay[0] - lastWay[0]) / dt;
          speedArray[1] = (currentWay[1] - lastWay[1]) / dt;
      } else {
          speedArray[0] = 0;
          speedArray[1] = 0;
      }
      
      lastWay[0] = currentWay[0];
      lastWay[1] = currentWay[1];
      return true;
    }
    else return false;
}

bool getImuData(float accArray[3], float gyrArray[3], const uint32_t interval) {
  MyIMU.update();

  static uint32_t lastMillis = millis();
  uint32_t currentMillis = millis();
  if (currentMillis - lastMillis > interval) {
    lastMillis = currentMillis;

    accArray[0] = MyIMU.getRawAccX() / 16384.0f;
    accArray[1] = MyIMU.getRawAccY() / 16384.0f;
    accArray[2] = MyIMU.getRawAccZ() / 16384.0f;

    gyrArray[0] = MyIMU.getRawGyroX() / 65.535f;
    gyrArray[1] = MyIMU.getRawGyroY() / 65.535f;
    gyrArray[2] = MyIMU.getRawGyroZ() / 65.535f;

    gyrArray[0] -= calibGyr[0];
    gyrArray[1] -= calibGyr[1];
    gyrArray[2] -= calibGyr[2];
    return true;
  }
  else return false;
}

void calibrateGyr(uint16_t samples, uint32_t interval) {
    float sumGyr[3] = {0.0f}, gyrArray[3] = {0.0f}, accArray[3] = {0.0f};
    uint16_t validSamples = 0;

    while (validSamples < samples) {
        if (getImuData(accArray, gyrArray, interval) == true) {
            sumGyr[0] += gyrArray[0];
            sumGyr[1] += gyrArray[1];
            sumGyr[2] += gyrArray[2];
            validSamples++;
        }
    }
    // Berechnen der Durchschnittswerte
    calibGyr[0] = sumGyr[0] / (float)validSamples;
    calibGyr[1] = sumGyr[1] / (float)validSamples;
    calibGyr[2] = sumGyr[2] / (float)validSamples;
}

bool getSimpleOdom(float odom[3], uint32_t interval) {
  static float wheelDist = 0.11f, anglePhi = 0.0f,
               xPos = 0.0f, yPos = 0.0f,
               currentWay[2] = { 0.0f }, lastWay[2] = {0.0f};
  
  static uint32_t lastMillis = millis();
  uint32_t currentMillis = millis();
  if (currentMillis - lastMillis > interval) {
    lastMillis = currentMillis;

    getSimpleWay(currentWay);
    float displacementLeft = currentWay[0] - lastWay[0];
    float displacementRight = currentWay[1] - lastWay[1];
    lastWay[0] = currentWay[0];
    lastWay[1] = currentWay[1];
    float dispAvg = (displacementLeft + displacementRight) / 2.0f;
    float deltaPhi = (displacementLeft - displacementRight) / wheelDist;

    anglePhi = anglePhi + deltaPhi;
    if (anglePhi > PI) anglePhi = anglePhi - (2 * PI);
    else if (anglePhi < -PI) anglePhi = anglePhi + (2 * PI);

    float deltaX = dispAvg * cos(anglePhi);
    float deltaY = dispAvg * sin(anglePhi);

    xPos = xPos + deltaX;
    yPos = yPos + deltaY;

    odom[0] = xPos;
    odom[1] = yPos;
    odom[2] = anglePhi * 180.0f / PI;
    return true;
  }
  else return false;
}

bool getOdom(float odom[3], uint32_t interval) {

  const float distWheel = 0.11f;
  static float phi_t_last = 0.0f, x_t_last = 0.0f, y_t_last = 0.0f;
  static float lastWay[2] = { 0.0f }, currentWay[2] = { 0.0f };

  static uint32_t lastMillis = millis();
  uint32_t currentMillis = millis();
  if (currentMillis - lastMillis > interval) {
    lastMillis = currentMillis;
    getSimpleWay(currentWay);
    float d_R = currentWay[0] - lastWay[0];
    float d_L = currentWay[1] - lastWay[1];

    float delta_phi_t = (d_R - d_L) / distWheel;
    float phi_t = phi_t_last + delta_phi_t;
    float R = d_L * (distWheel / (d_R - d_L)) + (distWheel / 2);
    float d = (d_L + d_R) / 2;
    float x_t = x_t_last + d * cos(phi_t_last + (delta_phi_t / 2));
    float y_t = y_t_last + d * sin(phi_t_last + (delta_phi_t / 2));

    phi_t_last = phi_t;
    x_t_last = x_t;
    y_t_last = y_t;

    lastWay[0] = currentWay[0];
    lastWay[1] = currentWay[1];

    odom[0] = x_t;
    odom[1] = y_t;
    odom[2] = phi_t * 180.0f / PI;
    return true;
  }
  else return false;
}

bool getAlphaAngle(float& alphaAngle, const float factor, const uint32_t interval) {
   // Variablen für die Berechnung und Speicherung der Angle
  static float accData[3] = {0.0f}, gyrData[3] = {0.0f};
  static float dt = 0.0f;
  static uint32_t gyrCurrentMicros;
  static uint32_t gyrLastMicros = micros();

  if (getImuData(accData, gyrData, interval) == true) {
    gyrCurrentMicros = micros();
    // Berechnung des NeigungsAngles auf Basis der linearen Beschleunigung
    float accAngle = atan2(accData[0], accData[2]) * -180 / PI;
    // float accAngle = atan2(accData[0], accData[2] + abs(accData[1])) * -180 / PI;
    // Berechnung des Neigungswinkels auf Basis der Anglegeschwindigkeit (Gyroskop)
    dt = (gyrCurrentMicros - gyrLastMicros) * 1e-6f;
    gyrLastMicros = gyrCurrentMicros;
    // Berechnung des NeigungsAngles im Komplementärfilter
    static float compAngle = accAngle;
    compAngle = factor * (compAngle + gyrData[1] * dt) + (1-factor) * accAngle;
    alphaAngle = compAngle;
    return true;
  }
  else return false;
}

#endif