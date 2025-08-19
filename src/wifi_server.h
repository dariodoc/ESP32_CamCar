#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

void initWiFi();
void arduinoOTA_task(void *parameters); 
void cleanupWSClients_task(void *parameters); 
void sendTelemetryTask(void *parameters);

#endif // WIFI_SERVER_H