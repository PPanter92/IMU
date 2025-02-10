#ifndef ULTRASONIC_SENSOR
#define ULTRASONIC_SENSOR

#include <Arduino.h>
#include "ultrasonic_sensor.h"

// Globale Variablen zur Verwaltung des Ultraschallsensors.
static int pinTrigger, pinEcho = 0;
volatile static uint32_t echoDuration, interval = 0;
volatile static bool echoRecieved = false;

/**
 * Initialisiert den Ultraschallsensor mit den gegebenen Pins und dem Messintervall.
 * 
 * @param pinTriggerUs Pin-Nummer für den Trigger des Ultraschallsensors.
 * @param pinEchoUs Pin-Nummer für das Echo des Ultraschallsensors.
 * @param intervalUs Zeitintervall in Mikrosekunden, in dem der Sensor getriggert wird.
 */
void initUsSensor(int pinTriggerUs, int pinEchoUs, uint32_t intervalUs) {
  pinTrigger = pinTriggerUs;
  pinEcho = pinEchoUs;
  interval = intervalUs;
  pinMode(pinTrigger, OUTPUT); // Setze den Trigger-Pin als Ausgang.
  pinMode(pinEcho, INPUT); // Setze den Echo-Pin als Eingang.
  attachInterrupt(pinEcho, pulsdurationHandle, CHANGE); // Attach an interrupt handler to detect echo signals.
}

/**
 * Sendet ein Trigger-Signal, um die Entfernungsmessung zu starten.
 * 
 * @param intervalTrigger Das minimale Intervall zwischen den Trigger-Ereignissen.
 */
void sendTriggerUs(uint32_t intervalTrigger) {
  static uint32_t previousMicros = micros(); // Zeitpunkt des letzten Triggers.
  unsigned long currentMicros = micros();
  static bool pinState = LOW; // Aktueller Zustand des Trigger-Pins.

  // Trigger-Signal senden, wenn das vorgegebene Intervall abgelaufen ist.
  if (currentMicros - previousMicros >= intervalTrigger) {
    if (pinState == LOW) {
      digitalWrite(pinTrigger, HIGH); // Setze den Trigger-Pin auf HIGH.
      pinState = HIGH;
      previousMicros = currentMicros; // Aktualisiere die Zeit für den letzten Trigger.
    }
  }
  // Setze den Trigger-Pin nach 12 Mikrosekunden zurück auf LOW.
  if (pinState == HIGH && (currentMicros - previousMicros >= 12)) {
    digitalWrite(pinTrigger, LOW);
    pinState = LOW;
  }
}

/**
 * Überprüft, ob eine neue Entfernungsmessung verfügbar ist.
 * 
 * @return True, wenn eine neue Messung verfügbar ist, sonst False.
 */
bool newDistanceUs() {
  sendTriggerUs(interval); // Trigger den Ultraschallsensor.
  return echoRecieved; // Gibt zurück, ob ein Echo empfangen wurde.
}

/**
 * Gibt die gemessene Entfernung zurück und setzt den Empfangsstatus zurück.
 * 
 * @return Die berechnete Entfernung in Zentimetern.
 */
uint16_t getDistanceUs() {
  echoRecieved = false; // Reset des Empfangsstatus, um für die nächste Messung bereit zu sein.
  return echoDuration / 58; // Berechne die Entfernung basierend auf der Dauer des Echos.
}

/**
 * Interrupt Service Routine zur Erfassung der Dauer des Echos.
 * Wird aufgerufen, wenn der Zustand des Echo-Pins sich ändert.
 */
void pulsdurationHandle() {
  static uint32_t echoStart = 0; // Startzeitpunkt des Echos.
  uint8_t currentState = digitalRead(pinEcho); // Lese den aktuellen Zustand des Echo-Pins.

  if (currentState == HIGH) {
    echoStart = micros(); // Startzeit speichern, wenn das Echo beginnt.
  } else if (currentState == LOW) {
    echoDuration = micros() - echoStart; // Berechne die Dauer des Echos.
    echoRecieved = true; // Setze den Status, dass ein Echo empfangen wurde.
  }
}


#endif