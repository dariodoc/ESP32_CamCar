#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Variables Compartidas
extern volatile bool enableLaser;
extern volatile bool melodyOn;
extern volatile bool enableObstacleAvoidance;
extern volatile bool obstacleFound;

void setPanAngle(int angle);
void setTiltAngle(int angle);

// Handlers de Tareas Activas
extern TaskHandle_t playMelodyTaskHandle;
extern TaskHandle_t obstacleAvoidanceModeTaskHandle;

// Prototipos de Periféricos y Control
void setupPeripherals();
void centerServos();
void turnLaserOn(bool state);
void ledIndicator(int blinkTimes, int delayTimeMS);
void ledIndicator(int state);
void leftRearLed(int state);
void rightRearLed(int state);

// Tareas FreeRTOS Activas
void playMelody(void *parameters);
void obstacleAvoidanceMode(void *parameters);

#endif // PERIPHERALS_H