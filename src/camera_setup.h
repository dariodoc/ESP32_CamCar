#ifndef CAMERA_SETUP_H
#define CAMERA_SETUP_H

#include <Arduino.h>

void setupCamera();
void initCameraWebSocket();
void streamCameraFrame();

#endif // CAMERA_SETUP_H