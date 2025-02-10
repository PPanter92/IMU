// Dateiname: odometry.cpp
#ifndef ODOMETRY
#define ODOMETRY

#include <Arduino.h>
#include <pio_encoder.h>
#include "odometry.h"
#include <Wire.h>
#include <MPU6050_tockn.h>

#define ENC_L 12  // Pin number for the left encoder
#define ENC_R 14  // Pin number for the right encoder

// Initialize encoder objects for left and right wheels
PioEncoder EncoderL(ENC_L);
PioEncoder EncoderR(ENC_R);

#define IMU_SDA 4  // I2C data pin for the IMU
#define IMU_SCL 5  // I2C clock pin for the IMU

MPU6050 MyIMU(Wire); // Create an MPU6050 IMU instance using the Wire library

// Gyroscope calibration offsets (pre-determined values)
static float calibGyr[3] = {8.4f, 4.02f, -0.55f};

/**
 * Initializes the odometry system.
 * 
 * This function initializes the left and right encoders and configures 
 * the I2C connection for the IMU. It also performs gyroscope calibration.
 */
void initOdometry() {
  EncoderL.begin();  // Initialize the left encoder
  EncoderR.begin();  // Initialize the right encoder
  EncoderR.flip();   // Reverse the counting direction for the right encoder (if necessary)

  // Configure I2C communication for the IMU
  Wire.setSDA(IMU_SDA);
  Wire.setSCL(IMU_SCL);
  Wire.begin();
  Wire.setClock(400 * 1000); // Set I2C clock speed to 400 kHz (fast mode)

  MyIMU.begin(); // Initialize the IMU sensor

  // Perform gyroscope calibration (3000 samples, mode 1)
  calibrateGyr(3000, 1);
}

/**
 * Calculates the traveled distance for each wheel.
 *
 * This function converts encoder counts into physical distance 
 * using predefined wheel diameter and encoder resolution.
 *
 * @param wayArray Output array containing the traveled distance for left and right wheels.
 */
void getSimpleWay(float wayArray[2]) {
  const float diaWheel = 0.09f;  // Wheel diameter in meters
  const float countsPerRevolution = 1440.0f; // Encoder counts per full wheel revolution
  const float wheelFactor = diaWheel * PI / countsPerRevolution; // Conversion factor

  // Compute traveled distance for each wheel
  wayArray[0] = EncoderL.getCount() * wheelFactor;
  wayArray[1] = EncoderR.getCount() * wheelFactor;
}

/**
 * Calculates the wheel speeds based on encoder data.
 * 
 * This function computes the speed of each wheel by measuring the 
 * distance traveled over a given time interval. It uses previous 
 * position data to determine the speed.
 * 
 * @param speedArray Output array containing the calculated speeds (m/s) for left and right wheels.
 * @param interval Time interval (ms) between speed updates.
 * @return Returns true if a speed update was performed, false otherwise.
 */
bool getSimpleSpeed(float speedArray[2], uint32_t interval) {
    static float lastWay[2] = { 0.0f }, currentWay[2] = { 0.0f }; // Stores previous and current wheel distances.
    static uint32_t lastMillis = millis(); // Timestamp of the last speed update.
    
    uint32_t currentMillis = millis(); // Get current time.
    if (currentMillis - lastMillis > interval) { // Check if the update interval has passed.
        float dt = (currentMillis - lastMillis) / 1000.0f; // Convert time difference to seconds.
        lastMillis = currentMillis; // Update the last timestamp.

        getSimpleWay(currentWay); // Get the latest traveled distances for each wheel.

        if (dt > 0) { // Ensure division by zero is avoided.
            speedArray[0] = (currentWay[0] - lastWay[0]) / dt; // Compute left wheel speed (m/s).
            speedArray[1] = (currentWay[1] - lastWay[1]) / dt; // Compute right wheel speed (m/s).
        } else {
            speedArray[0] = 0; // If time interval is invalid, set speed to zero.
            speedArray[1] = 0;
        }

        lastWay[0] = currentWay[0]; // Store the current values for the next computation.
        lastWay[1] = currentWay[1];

        return true; // Speed update was performed.
    }
    return false; // No update, interval not yet reached.
}

/**
 * Reads IMU acceleration and gyroscope data at a specified interval.
 * 
 * This function retrieves raw accelerometer and gyroscope values from the IMU,
 * scales them to physical units (g for acceleration, deg/s for gyroscope), 
 * and applies gyroscope calibration offsets.
 *
 * @param accArray Output array for acceleration values (g).
 * @param gyrArray Output array for gyroscope values (deg/s).
 * @param interval Minimum time interval (ms) between measurements.
 * @return Returns true if data was updated, false otherwise.
 */
bool getImuData(float accArray[3], float gyrArray[3], const uint32_t interval) {
  MyIMU.update(); // Update IMU sensor data.

  static uint32_t lastMillis = millis(); // Store the last update timestamp.
  uint32_t currentMillis = millis(); // Get current time.

  if (currentMillis - lastMillis > interval) { // Ensure minimum interval between readings.
    lastMillis = currentMillis; // Update timestamp.

    // Convert raw accelerometer data to g-units (MPU6050 scale factor: 16384 LSB/g)
    accArray[0] = MyIMU.getRawAccX() / 16384.0f;
    accArray[1] = MyIMU.getRawAccY() / 16384.0f;
    accArray[2] = MyIMU.getRawAccZ() / 16384.0f;

    // Convert raw gyroscope data to degrees per second (MPU6050 scale factor: 65.535 LSB/(°/s))
    gyrArray[0] = MyIMU.getRawGyroX() / 65.535f;
    gyrArray[1] = MyIMU.getRawGyroY() / 65.535f;
    gyrArray[2] = MyIMU.getRawGyroZ() / 65.535f;

    // Apply gyroscope calibration offsets
    gyrArray[0] -= calibGyr[0];
    gyrArray[1] -= calibGyr[1];
    gyrArray[2] -= calibGyr[2];

    return true; // Data was updated successfully.
  }
  return false; // No new data yet.
}

/**
 * Calibrates the gyroscope by averaging multiple readings.
 *
 * This function collects multiple gyroscope measurements over a given interval
 * and calculates the average bias, which is then used to correct future readings.
 *
 * @param samples Number of samples to collect for calibration.
 * @param interval Time interval (ms) between each sample.
 */
void calibrateGyr(uint16_t samples, uint32_t interval) {
    float sumGyr[3] = {0.0f}, gyrArray[3] = {0.0f}, accArray[3] = {0.0f}; // Arrays for summing values.
    uint16_t validSamples = 0; // Counter for collected valid samples.

    while (validSamples < samples) {
        if (getImuData(accArray, gyrArray, interval) == true) {
            sumGyr[0] += gyrArray[0];
            sumGyr[1] += gyrArray[1];
            sumGyr[2] += gyrArray[2];
            validSamples++;
        }
    }
    // Compute average gyroscope bias values
    calibGyr[0] = sumGyr[0] / (float)validSamples;
    calibGyr[1] = sumGyr[1] / (float)validSamples;
    calibGyr[2] = sumGyr[2] / (float)validSamples;
}

/**
 * Calculates the tilt angle using a complementary filter.
 * 
 * This function fuses accelerometer and gyroscope data to compute a stable 
 * tilt angle. The accelerometer provides a long-term reference, while the 
 * gyroscope accounts for fast rotational changes.
 *
 * @param alphaAngle Reference to the output tilt angle (degrees).
 * @param factor Complementary filter factor (0 to 1), higher values rely more on gyroscope.
 * @param interval Minimum time interval (ms) between measurements.
 * @return Returns true if the angle was updated, false otherwise.
 */
bool getAlphaAngle(float& alphaAngle, const float factor, const uint32_t interval) {
   // Variables for storing sensor data and time measurements
  static float accData[3] = {0.0f}, gyrData[3] = {0.0f}; // IMU data storage
  static float dt = 0.0f; // Time difference (s)
  static uint32_t gyrCurrentMicros; // Current time in microseconds
  static uint32_t gyrLastMicros = micros(); // Previous time for integration

  if (getImuData(accData, gyrData, interval) == true) { // Check if new IMU data is available
    gyrCurrentMicros = micros(); // Get current time

    // Compute tilt angle based on linear acceleration
    float accAngle = atan2(accData[0], accData[2]) * -180 / PI; // Calculate inclination angle (degrees)

    // Alternative formula that includes absolute Y acceleration (commented out)
    // float accAngle = atan2(accData[0], accData[2] + abs(accData[1])) * -180 / PI;

    // Compute tilt angle based on angular velocity (gyroscope)
    dt = (gyrCurrentMicros - gyrLastMicros) * 1e-6f; // Convert time difference to seconds
    gyrLastMicros = gyrCurrentMicros; // Update last timestamp

    // Complementary filter calculation: blends accelerometer and gyroscope data
    static float compAngle = accAngle; // Initialize with accelerometer reading
    compAngle = factor * (compAngle + gyrData[1] * dt) + (1 - factor) * accAngle; // Apply complementary filter
    
    alphaAngle = compAngle; // Update output angle
    return true; // Successfully updated angle
  }
  return false; // No new data available
}

/**
 * Computes the robot's simple odometry based on wheel encoder data.
 *
 * This function estimates the robot's position (x, y) and orientation (heading angle) 
 * using a differential drive odometry model. The position is updated based on the 
 * displacement of the left and right wheels.
 *
 * @param odom Output array containing:
 *             - odom[0]: X position (meters)
 *             - odom[1]: Y position (meters)
 *             - odom[2]: Orientation angle (degrees)
 * @param interval Minimum time interval (ms) between updates.
 * @return Returns true if the odometry was updated, false otherwise.
 */
bool getSimpleOdom(float odom[3], uint32_t interval) {
  static float wheelDist = 0.11f; // Distance between the two wheels (meters)
  static float anglePhi = 0.0f;   // Orientation angle (radians)
  static float xPos = 0.0f, yPos = 0.0f; // Position coordinates (meters)
  static float currentWay[2] = { 0.0f }, lastWay[2] = { 0.0f }; // Distance traveled by each wheel

  static uint32_t lastMillis = millis(); // Store the last update timestamp
  uint32_t currentMillis = millis(); // Get the current time

  if (currentMillis - lastMillis > interval) { // Ensure minimum interval between updates
    lastMillis = currentMillis; // Update timestamp

    getSimpleWay(currentWay); // Retrieve the latest traveled distances for both wheels

    // Compute displacement for left and right wheels
    float displacementLeft = currentWay[0] - lastWay[0];
    float displacementRight = currentWay[1] - lastWay[1];

    // Update last traveled distance for next iteration
    lastWay[0] = currentWay[0];
    lastWay[1] = currentWay[1];

    // Compute the average displacement and rotation change
    float dispAvg = (displacementLeft + displacementRight) / 2.0f; // Linear movement
    float deltaPhi = (displacementLeft - displacementRight) / wheelDist; // Angular change

    // Update orientation angle
    anglePhi += deltaPhi;

    // Keep the angle within the range of -π to π (normalize orientation)
    if (anglePhi > PI) anglePhi -= (2 * PI);
    else if (anglePhi < -PI) anglePhi += (2 * PI);

    // Compute the change in position using the updated orientation
    float deltaX = dispAvg * cos(anglePhi);
    float deltaY = dispAvg * sin(anglePhi);

    // Update global position
    xPos += deltaX;
    yPos += deltaY;

    // Store updated position and orientation in the output array
    odom[0] = xPos; // X position in meters
    odom[1] = yPos; // Y position in meters
    odom[2] = anglePhi * 180.0f / PI; // Convert orientation angle to degrees

    return true; // Odometry update successful
  }
  return false; // No update performed
}

#endif