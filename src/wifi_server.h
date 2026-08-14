#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

#include <Arduino.h>

extern int carInputClientId;
extern volatile int targetDirection;
extern volatile unsigned long lastCommandTime;

void initWiFi();
void cleanupWSClients();

#endif // WIFI_SERVER_H