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

// /**
//  * Liest die Batteriespannung des Systems.
//  *
//  * @param interval Zeitintervall in Millisekunden zwischen den Messungen.
//  * @return Die gemessene Batteriespannung, skaliert durch einen Spannungsteiler.
//  */
// float getBatteryVoltage(const uint32_t interval) {
//   static uint32_t currentMillis, lastMillis = millis(); // Speichern der aktuellen und letzten Zeitpunkte der Messung.
//   static float batteryVoltage = 8200.0f; // Startwert für die Batteriespannung.
//   const float voltageDividerFactor = 9.7f; // Faktor des Spannungsteilers.
//   currentMillis = millis();
//   if (currentMillis - lastMillis > interval) {
//      lastMillis = currentMillis;
//       batteryVoltage = analogRead(BATTERYPIN) * voltageDividerFactor; // Aktualisiere die Batteriespannung, wenn das Intervall abgelaufen ist.
//   }
//   return batteryVoltage;
// }

/**
 * Liest die Batteriespannung des Systems.
 * 
 * @param interval Zeitintervall in Millisekunden zwischen den Messungen.
 * @return Die gemessene Batteriespannung, skaliert durch einen Spannungsteiler.
 */
float getBatteryVoltage(const uint32_t interval) {
  static uint32_t currentMillis, lastMillis = millis();           // Speichern der aktuellen und letzten Zeitpunkte der Messung.
  static uint32_t batteryVoltage = 0, averageBatteryVoltage = 0;  // Startwert für die Batteriespannung.
  const float voltageDividerFactor = 9.7f;                        // Faktor des Spannungsteilers.
  const uint16_t numReadings = 25;
  static uint16_t readIndex = 0;
  static uint32_t readings[numReadings] = { 0 };

  currentMillis = millis();
  if (currentMillis - lastMillis > interval) {
    lastMillis = currentMillis;
    batteryVoltage = batteryVoltage - readings[readIndex];
    readings[readIndex] = analogRead(BATTERYPIN);  // Aktualisiere die Batteriespannung, wenn das Intervall abgelaufen ist.
    batteryVoltage = batteryVoltage + readings[readIndex];
    // Weiter zum nächsten Wert im Array
    readIndex = readIndex + 1;

    // Wenn wir das Ende des Arrays erreicht haben, gehe zurück zum Anfang
    if (readIndex >= numReadings) {
      readIndex = 0;
    }

    // Berechne den Mittelwert
    averageBatteryVoltage = batteryVoltage / numReadings;
  }
  return averageBatteryVoltage * voltageDividerFactor;
}

/**
 * Initialisiert die Motorsteuerungspinne und die Betriebsmodus-Pinne.
 */
void initMotorDriver() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(MODE, OUTPUT);
  pinMode(BATTERYPIN, INPUT);
  digitalWrite(MODE, LOW);  // Setze den Modus auf LOW.
  analogWriteFreq(20000);   // Setze die PWM-Frequenz auf 20kHz.
}

/**
 * Berechnet die PWM-Werte für die Motorsteuerung basierend auf den Eingabewerten.
 *
 * @param steer Array mit Werten zur Steuerung der Richtung.
 * @param motorPwmOut Ausgangsarray für die PWM-Werte der Motoren.
 */
void steerWheelchairMode(const float steer[2], int16_t motorPwmOut[2]) {
  static float motorPwm[2] = { 0.0f };
  motorPwm[0] = steer[0] - steer[1];                                        // Berechnung der PWM für den linken Motor.
  motorPwm[1] = steer[0] + steer[1];                                        // Berechnung der PWM für den rechten Motor.
  float mag = sqrt(motorPwm[0] * motorPwm[0] + motorPwm[1] * motorPwm[1]);  // Berechnung der Vektormagnitude.
  const float maxMag = 359.0;                                               // Maximal zulässige Magnitude, abgeleitet von 255 * sqrt(2).
  if (mag > maxMag) {
    float scale = maxMag / mag;  // Skalierungsfaktor, wenn die Maximalmagnitude überschritten wird.
    motorPwm[0] *= scale;
    motorPwm[1] *= scale;
  }
  motorPwmOut[0] = (int16_t)constrain(motorPwm[0], -255, 255);  // Begrenzung der PWM-Werte.
  motorPwmOut[1] = (int16_t)constrain(motorPwm[1], -255, 255);
}

/**
 * Setzt die PWM-Werte für die Motoren.
 * Verwendung des Mode InIn des Pololu DRV8835 
 * 
 * @param motorPwmIn Array mit den gewünschten PWM-Werten für die Motoren.
 */
void setMotorPwm(const int16_t motorPwmIn[2]) {
  static uint8_t motorPwm[4] = { 0 };
  static const uint16_t motorPwmOffset[4]{ 90, 90, 90, 90 };  // Minimale PWM-Werte zur Überwindung des Stillstandmoments.
  static const float motorVoltageLimit = 6000.0f;             // Spannungsgrenze für den Motor.
  float batVal = getBatteryVoltage(100);                     // Lese die aktuelle Batteriespannung.
  float batFactor = motorVoltageLimit / batVal;               // Berechnung des Skalierungsfaktors basierend auf der Batteriespannung.
  uint8_t maxMotorPwm = 255 * batFactor;                      // Maximalwert des PWM-Signals auf Basis der Batteriespannung.

  // Setzen der PWM-Werte für die Motoren basierend auf den Eingangswerten und Batteriespannung.
  if (motorPwmIn[0] > 0) {
    motorPwm[0] = 0;
    // Remapping der Werte 0 bis 255 auf den Bereich Offset bis Maximum.
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
  // Anwenden der berechneten PWM-Werte auf die Motorpins.
  analogWrite(AIN1, motorPwm[0]);
  analogWrite(AIN2, motorPwm[1]);
  analogWrite(BIN1, motorPwm[2]);
  analogWrite(BIN2, motorPwm[3]);
}

#endif