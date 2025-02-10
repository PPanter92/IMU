#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "motor_controller.h"

// Definition der Pin-Nummern für die Motoransteuerung und Batteriespannungsmessung.
#define AIN1 6
#define AIN2 7
#define BIN1 8
#define BIN2 9
#define MODE 10
#define BATTERYPIN 28

/**
 * Initializes the motor driver control pins and the operation mode pin.
 */
void initMotorDriver() {
  pinMode(AIN1, OUTPUT); // Set motor driver input AIN1 as an output.
  pinMode(AIN2, OUTPUT); 
  pinMode(BIN1, OUTPUT); 
  pinMode(BIN2, OUTPUT); 
  pinMode(MODE, OUTPUT); 
  pinMode(BATTERYPIN, INPUT); // Set the battery voltage pin as an input.

  digitalWrite(MODE, LOW);  // Set the DRV8835 to InIn-mode.

  analogWriteFreq(20000);   // Set the PWM frequency to 20kHz for motor control.
}

/**
 * Reads the system's battery voltage.
 * This function is internally used to get the latest ADC value.
 * 
 * @return The latest battery voltage measurement.
 */
float readBatteryVoltage() {
  const float voltageDividerFactor = 9.7f; // Factor to scale ADC reading to actual voltage
  return analogRead(BATTERYPIN) * voltageDividerFactor;
}

/**
 * Reads the system's battery voltage periodically.
 * 
 * @param interval Time interval in milliseconds between measurements.
 * @return The most recent battery voltage measurement.
 */
float getBatteryVoltage(const uint32_t interval) {
  static uint32_t lastMillis = millis(); // Stores the last measurement timestamp
  static float batteryVoltage = 8200.0f; // Initial estimated battery voltage

  uint32_t currentMillis = millis();
  if (currentMillis - lastMillis > interval) { // Check if measurement interval elapsed
    lastMillis = currentMillis; // Update last timestamp
    batteryVoltage = readBatteryVoltage(); // Use shared function
  }

  return batteryVoltage; // Return the most recent voltage measurement
}

/**
 * Reads the system's battery voltage and applies a moving average filter.
 * 
 * @param interval Time interval in milliseconds between measurements.
 * @return The filtered battery voltage.
 */
float getMeanBatteryVoltage(const uint32_t interval) {
  static uint32_t lastMillis = millis(); // Timestamp for the last measurement
  static float batteryVoltageSum = 0.0f; // Sum of readings for averaging
  const uint16_t numReadings = 25;       // Number of samples for moving average
  static uint16_t readIndex = 0;         // Circular buffer index
  static float readings[numReadings] = {0.0f}; // Circular buffer for past readings

  uint32_t currentMillis = millis();
  if (currentMillis - lastMillis > interval) { // Check if interval has elapsed
    lastMillis = currentMillis; // Update timestamp

    // Get a new voltage reading (shared function)
    float newReading = readBatteryVoltage();

    // Remove the oldest reading from the sum
    batteryVoltageSum -= readings[readIndex];

    // Store the new reading in the circular buffer
    readings[readIndex] = newReading;

    // Add the new reading to the sum
    batteryVoltageSum += readings[readIndex];

    // Move to the next index in the circular buffer
    readIndex = (readIndex + 1) % numReadings;
  }

  // Compute the moving average
  return batteryVoltageSum / numReadings;
}

/**
 * Computes the PWM values for motor control based on input values.
 *
 * This function calculates the required PWM values for a differential drive
 * system, such as a wheelchair, using two input values for steering.
 *
 * @param steer Array containing values to control direction (throttle and steering).
 * @param motorPwmOut Output array for the motor PWM values.
 */
void steerHorizontalMode(const float steer[2], int16_t motorPwmOut[2]) {
  static float motorPwm[2] = { 0.0f };

  // Compute PWM values for left and right motors based on input steering values.
  motorPwm[0] = steer[0] - steer[1]; // Left motor PWM calculation.
  motorPwm[1] = steer[0] + steer[1]; // Right motor PWM calculation.

  // Compute the vector magnitude to determine if scaling is necessary.
  float mag = sqrt(motorPwm[0] * motorPwm[0] + motorPwm[1] * motorPwm[1]);  

  const float maxMag = 359.0; // Maximum allowed magnitude, derived from 255 * sqrt(2).

  if (mag > maxMag) {
    float scale = maxMag / mag; // Scaling factor to prevent exceeding max magnitude.
    motorPwm[0] *= scale;
    motorPwm[1] *= scale;
  }

  // Constrain the PWM values within the range of -255 to 255.
  motorPwmOut[0] = (int16_t)constrain(motorPwm[0], -255, 255);
  motorPwmOut[1] = (int16_t)constrain(motorPwm[1], -255, 255);
}


/**
 * Sets the PWM values for the motors.
 * Uses the "Mode InIn" configuration of the Pololu DRV8835 motor driver.
 * 
 * This function maps the input motor PWM values to appropriate output values,
 * ensuring that the minimum necessary PWM is applied to overcome static friction.
 * Additionally, it scales the PWM values based on the current battery voltage.
 * 
 * @param motorPwmIn Array containing the desired PWM values for the motors.
 */
void setMotorPwm(const int16_t motorPwmIn[2]) {
  static uint8_t motorPwm[4] = { 0 };  // PWM values for the motor driver.
  static const uint16_t motorPwmOffset[4] = { 90, 90, 90, 90 };  // Minimum PWM values to overcome stall torque.
  static const float motorVoltageLimit = 6000.0f;  // Maximum motor voltage.
  
  float batVal = getMeanBatteryVoltage(100);  // Read the current battery voltage.
  float batFactor = motorVoltageLimit / batVal;  // Compute scaling factor based on battery voltage.
  uint8_t maxMotorPwm = 255 * batFactor;  // Adjust maximum PWM value according to battery voltage.

  // Set PWM values for the motors based on input values and battery voltage.
  if (motorPwmIn[0] > 0) {
    motorPwm[0] = 0;
    // Remap values from 0 to 255 to the range from offset to maximum PWM.
    motorPwm[1] = map(motorPwmIn[0], 0, 255, motorPwmOffset[0], maxMotorPwm);
  } else if (motorPwmIn[0] < 0) {
    motorPwm[0] = map(abs(motorPwmIn[0]), 0, 255, motorPwmOffset[1], maxMotorPwm);
    motorPwm[1] = 0;
  } else {
    motorPwm[0] = 0;
    motorPwm[1] = 0;
  }

  if (motorPwmIn[1] > 0) {
    motorPwm[2] = map(motorPwmIn[1], 0, 255, motorPwmOffset[2], maxMotorPwm);
    motorPwm[3] = 0;
  } else if (motorPwmIn[1] < 0) {
    motorPwm[2] = 0;
    motorPwm[3] = map(abs(motorPwmIn[1]), 0, 255, motorPwmOffset[3], maxMotorPwm);
  } else {
    motorPwm[2] = 0;
    motorPwm[3] = 0;
  }

  // Apply the computed PWM values to the motor control pins.
  analogWrite(AIN1, motorPwm[0]);
  analogWrite(AIN2, motorPwm[1]);
  analogWrite(BIN1, motorPwm[2]);
  analogWrite(BIN2, motorPwm[3]);
}


#endif