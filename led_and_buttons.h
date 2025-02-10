#ifndef LED_AND_BUTTON_H
#define LED_AND_BUTTON_H

void initLedAndButtons(int pinLed, int pinBL, int pinBR); 
void blinkLedBlocking(int pinLed, uint32_t interval);
void blinkLedNonBlocking(int pinLed, uint32_t interval);
void fadeLedNonBlocking(int pinLed, uint32_t interval);
void buttonRHandle();
void buttonLHandle();

#endif