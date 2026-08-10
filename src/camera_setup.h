#ifndef CAMERA_SETUP_H
#define CAMERA_SETUP_H

void setupCamera();
void startCameraServerTask(void *pvParameters);

extern TaskHandle_t CameraServerTaskHandle;


#endif // CAMERA_SETUP_H