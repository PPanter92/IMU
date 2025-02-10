#ifndef WEBSERVER_MS_H
#define WEBSERVER_MS_H

#include <WebSocketsServer.h>

void initWebsocketServer(const char* hotspotName, const char* hotspotPassword, bool createHotspot);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
void websocketSendData(float websocketArray[7], uint32_t sendIntervall);
void websocketRefresh();
bool websocketGetSteerData(float steerVals[2]);
bool websocketGetPidData(float pidVals[9]);

#endif