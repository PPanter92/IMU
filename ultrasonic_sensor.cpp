#ifndef ULTRASONIC_SENSOR
#define ULTRASONIC_SENSOR

#include <Arduino.h>
#include "ultrasonic_sensor.h"

// Global variables for managing the ultrasonic sensor.
static int pinTrigger, pinEcho = 0; // Pins for triggering and receiving the echo.
volatile static uint32_t echoDuration, interval = 0; // Stores the echo duration and trigger interval.
volatile static bool echoRecieved = false; // Flag to indicate if an echo was received.

/**
 * Initializes the ultrasonic sensor with the given pins and measurement interval.
 * 
 * @param pinTriggerUs Pin number for the ultrasonic sensor's trigger.
 * @param pinEchoUs Pin number for receiving the echo.
 * @param intervalUs Time interval in microseconds at which the sensor is triggered.
 */
void initUsSensor(int pinTriggerUs, int pinEchoUs, uint32_t intervalUs) {
  pinTrigger = pinTriggerUs;
  pinEcho = pinEchoUs;
  interval = intervalUs;
  pinMode(pinTrigger, OUTPUT); // Set the trigger pin as an output.
  pinMode(pinEcho, INPUT); // Set the echo pin as an input.
  attachInterrupt(pinEcho, pulsdurationHandle, CHANGE); // Attach an interrupt handler to detect echo signals.
}

/**
 * Sends a trigger signal to start a distance measurement.
 * 
 * @param intervalTrigger Minimum interval between trigger events.
 */
void sendTriggerUs(uint32_t intervalTrigger) {
  static uint32_t previousMicros = micros(); // Timestamp of the last trigger.
  unsigned long currentMicros = micros();
  static bool pinState = LOW; // Current state of the trigger pin.

  // Send trigger signal if the predefined interval has elapsed.
  if (currentMicros - previousMicros >= intervalTrigger) {
    if (pinState == LOW) {
      digitalWrite(pinTrigger, HIGH); // Set the trigger pin HIGH.
      pinState = HIGH;
      previousMicros = currentMicros; // Update the last trigger timestamp.
    }
  }
  // Set the trigger pin back to LOW after 12 microseconds.
  if (pinState == HIGH && (currentMicros - previousMicros >= 12)) {
    digitalWrite(pinTrigger, LOW);
    pinState = LOW;
  }
}

/**
 * Checks if a new distance measurement is available.
 * 
 * @return True if a new measurement is available, otherwise False.
 */
bool newDistanceUs() {
  sendTriggerUs(interval); // Trigger the ultrasonic sensor.
  return echoRecieved; // Return whether an echo was received.
}

/**
 * Returns the measured distance and resets the reception status.
 * 
 * The measured echo duration is converted to distance using the approximation:
 * - Sound travels at ~343 m/s or ~0.0343 cm/µs
 * - Since the pulse travels to the object and back, the distance is calculated as `duration / 58`
 * 
 * @return The calculated distance in centimeters.
 */
uint16_t getDistanceUs() {
  echoRecieved = false; // Reset the reception status to be ready for the next measurement.
  return echoDuration / 58; // Calculate distance based on echo duration.
}

/**
 * Interrupt Service Routine (ISR) for capturing the echo duration.
 * This function is called whenever the state of the echo pin changes.
 */
void pulsdurationHandle() {
  static uint32_t echoStart = 0; // Timestamp for the start of the echo.
  uint8_t currentState = digitalRead(pinEcho); // Read the current state of the echo pin.

  if (currentState == HIGH) {
    echoStart = micros(); // Store start time when the echo begins.
  } else if (currentState == LOW) {
    echoDuration = micros() - echoStart; // Compute the duration of the echo pulse.
    echoRecieved = true; // Set flag indicating that an echo was received.
  }
}

#endif