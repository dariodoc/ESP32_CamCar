#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

void initWiFi();
void cleanupWSClients();
void sendTelemetryTask(void *parameters);

#endif // WIFI_SERVER_H