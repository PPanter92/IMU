// Filename: led_and_buttons.cpp FINAL
#ifndef LED_AND_BUTTON
#define LED_AND_BUTTON

#include <Arduino.h>
#include "led_and_buttons.h"


// Declare external volatile variables for button states.
// These variables should be defined elsewhere in the program.

extern volatile bool buttonRPressed; // State of the right button.
extern volatile bool buttonLPressed; // State of the left button.


/**
 * Initializes the pins for an LED and two buttons with interrupts.
 * 
 * @param pinLed Pin number for the LED.
 * @param pinBL Pin number for the left button.
 * @param pinBR Pin number for the right button.
 */
void initLedAndButtons(int pinLed, int pinBL, int pinBR) {
    pinMode(pinLed, OUTPUT); // Sets the LED pin as an output.
    pinMode(pinBL, INPUT); // Sets the pin for the left button as an input.
    pinMode(pinBR, INPUT); // Sets the pin for the right button as an input.
    attachInterrupt(pinBL, buttonLHandle, FALLING); // Configures an interrupt for the left button on a falling edge.
    attachInterrupt(pinBR, buttonRHandle, FALLING); // Configures an interrupt for the right button on a falling edge.
}
 
/**
 * Makes the LED blink in a blocking manner.
 * 
 * @param pinLed Pin number of the LED.
 * @param interval Time interval in milliseconds between blink cycles.
 */
void blinkLedBlocking(int pinLed, uint32_t interval) {
    static bool toggle = 0;
    delay(interval); // Delays the program for the given interval.
    digitalWrite(pinLed, toggle); // Toggles the state of the LED.
    Serial.println(toggle); // Prints the current state of the LED.
    toggle = !toggle; // Switches the state for the next blink.
}

/**
 * Makes the LED blink in a non-blocking manner.
 * 
 * @param pinLed Pin number of the LED.
 * @param interval Time interval in milliseconds between blink cycles.
 */
void blinkLedNonBlocking(int pinLed, uint32_t interval) {
    static uint32_t previousMillis = 0;
    static bool toggle = 0;
    uint32_t currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis; // Updates the time for the next blink.
        digitalWrite(pinLed, toggle); // Toggles the state of the LED.
        toggle = !toggle; // Switch state for the next time.
    }
}

/**
 * Adjusts the brightness of the LED in a non-blocking manner.
 * 
 * @param pinLed Pin number of the LED.
 * @param interval Time interval in milliseconds for brightness change.
 */
void fadeLedNonBlocking(int pinLed, uint32_t interval) {
    static uint32_t previousMillis = 0;
    static uint8_t brightness = 0;
    static int8_t fadeAmount = 1; // Use int8_t to allow negative values for fade direction
    uint32_t currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis; // Resets the time for the next brightness change.
        analogWrite(pinLed, brightness); // Sets the LED brightness.
        brightness += fadeAmount; // Adjusts brightness for the next cycle.
        if (brightness <= 0 || brightness >= 255) {
            fadeAmount = -fadeAmount; // Reverses the direction of brightness change.
        }
    }
}


/**
 * Interrupt Service Routine (ISR) for the right button.
 * Prevents button bouncing by implementing a simple debounce mechanism.
 */
void buttonRHandle() {
    static uint32_t lastPressed = millis(); // Last time the button was pressed.
    uint32_t currentMillis = millis(); // Current time.
    if (currentMillis - lastPressed > 250) { // Checks if more than 250 milliseconds have passed since the last press.
        lastPressed = currentMillis; // Updates the last press time.
        buttonRPressed = !buttonRPressed; // Toggles the state of the right button.
    }
}

/**
 * Interrupt Service Routine (ISR) for the left button.
 * Prevents button bouncing by implementing a simple debounce mechanism.
 */
void buttonLHandle() {
    static uint32_t lastPressed = millis(); // Last time the button was pressed.
    uint32_t currentMillis = millis(); // Current time.
    if (currentMillis - lastPressed > 250) { // Checks if more than 250 milliseconds have passed since the last press.
        lastPressed = currentMillis; // Updates the last press time.
        buttonLPressed = !buttonLPressed; // Toggles the state of the left button.
    }
}


#endif
