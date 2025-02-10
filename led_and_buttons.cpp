// Dateiname: led_and_buttons.cpp
#ifndef LED_AND_BUTTON
#define LED_AND_BUTTON

#include <Arduino.h>
#include "led_and_buttons.h"

extern volatile bool buttonRPressed;
extern volatile bool buttonLPressed;

/**
 * Initialisiert die Pins für eine LED und zwei Tasten mit Interrupts.
 * 
 * @param pinLed Pin-Nummer für die LED.
 * @param pinBL Pin-Nummer für den linken Button.
 * @param pinBR Pin-Nummer für den rechten Button.
 */
void initLedAndButtons(int pinLed, int pinBL, int pinBR) {
    pinMode(pinLed, OUTPUT); // Setzt den LED-Pin als Ausgang.
    pinMode(pinBL, INPUT); // Setzt den Pin für den linken Button als Eingang.
    pinMode(pinBR, INPUT); // Setzt den Pin für den rechten Button als Eingang.
    attachInterrupt(pinBL, buttonLHandle, FALLING); // Konfiguriert einen Interrupt für den linken Button auf fallende Flanke.
    attachInterrupt(pinBR, buttonRHandle, FALLING); // Konfiguriert einen Interrupt für den rechten Button auf fallende Flanke.
}
 
/**
 * Lässt die LED in blockierender Weise blinken.
 * 
 * @param pinLed Pin-Nummer der LED.
 * @param interval Zeitintervall in Millisekunden zwischen den Blinkvorgängen.
 */
void blinkLedBlocking(int pinLed, uint32_t interval) {
    static bool toggle = 0;
    delay(interval); // Verzögert das Programm für das gegebene Intervall.
    digitalWrite(pinLed, toggle); // Wechselt den Zustand der LED.
    Serial.println(toggle); // Gibt den aktuellen Zustand der LED aus.
    toggle = !toggle; // Wechselt den Zustand für das nächste Blinken.
}

/**
 * Lässt die LED in nicht-blockierender Weise blinken.
 * 
 * @param pinLed Pin-Nummer der LED.
 * @param interval Zeitintervall in Millisekunden zwischen den Blinkvorgängen.
 */
void blinkLedNonBlocking(int pinLed, uint32_t interval) {
    static uint32_t previousMillis = 0;
    static bool toggle = 0;
    uint32_t currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis; // Aktualisiert die Zeit für das nächste Blinken.
        digitalWrite(pinLed, toggle); // Wechselt den Zustand der LED.
        toggle = !toggle; // Umschalten für das nächste Mal.
    }
}

/**
 * Verändert die Helligkeit der LED in einer nicht-blockierenden Weise.
 * 
 * @param pinLed Pin-Nummer der LED.
 * @param interval Zeitintervall in Millisekunden für die Helligkeitsänderung.
 */
void fadeLedNonBlocking(int pinLed, uint32_t interval) {
    static uint32_t previousMillis = 0;
    static uint8_t brightness = 0;
    static uint8_t fadeAmount = 1;
    uint32_t currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis; // Setzt die Zeit zurück für die nächste Helligkeitsänderung.
        analogWrite(pinLed, brightness); // Setzt die Helligkeit der LED.
        brightness += fadeAmount; // Verändert die Helligkeit für den nächsten Durchgang.
        if (brightness <= 0 || brightness >= 255) {
            fadeAmount = -fadeAmount; // Ändert die Richtung der Helligkeitsänderung.
        }
    }
}

/**
 * Interrupt Service Routine (ISR) für den rechten Button.
 * Verhindert das Prellen des Buttons durch Implementierung einer einfachen Entprellung.
 */
void buttonRHandle() {
    static uint32_t lastPressed = millis(); // Letzte Zeit, zu der der Button gedrückt wurde.
    uint32_t currentMillis = millis(); // Aktuelle Zeit.
    if (currentMillis - lastPressed > 250) { // Prüft, ob mehr als 50 Millisekunden seit dem letzten Drücken vergangen sind.
        lastPressed = currentMillis; // Aktualisiert die letzte Druckzeit.
        buttonRPressed = !buttonRPressed; // Toggelt den Zustand des rechten Buttons.
    }
}

/**
 * Interrupt Service Routine (ISR) für den linken Button.
 * Verhindert das Prellen des Buttons durch Implementierung einer einfachen Entprellung.
 */
void buttonLHandle() {
    static uint32_t lastPressed = millis(); // Letzte Zeit, zu der der Button gedrückt wurde.
    uint32_t currentMillis = millis(); // Aktuelle Zeit.
    if (currentMillis - lastPressed > 250) { // Prüft, ob mehr als 50 Millisekunden seit dem letzten Drücken vergangen sind.
        lastPressed = currentMillis; // Aktualisiert die letzte Druckzeit.
        buttonLPressed = !buttonLPressed; // Toggelt den Zustand des linken Buttons.
    }
}

#endif
