#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

void initUsSensor(int pinTriggerUs, int pinEchoUs, uint32_t intervalUs);
void sendTriggerUs(uint32_t interval);
bool newDistanceUs();
uint16_t getDistanceUs();
void pulsdurationHandle();

#endif