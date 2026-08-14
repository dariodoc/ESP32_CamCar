#ifndef CAMERA_SETUP_H
#define CAMERA_SETUP_H

#include <Arduino.h>
#include "ESPAsyncWebServer.h"

void setupCamera();
// 🚀 Ahora recibe el puntero al servidor asíncrono
void initCameraWebSocket(AsyncWebServer *webServer);
void streamCameraFrame();

#endif // CAMERA_SETUP_H