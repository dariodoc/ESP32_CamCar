#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Variables Compartidas
extern volatile bool enableLaser;
extern volatile bool melodyOn;
extern volatile bool enableObstacleAvoidance;
extern volatile bool enableIROnlyMode; // 🚀 Modo sin ultrasónico
extern volatile bool obstacleFound;
extern volatile bool enableSonicTelemetry;
extern volatile float latestSonicDistanceCM;

void setPanAngle(int angle);
void setTiltAngle(int angle);

// Handlers de Tareas Activas
extern TaskHandle_t playMelodyTaskHandle;
extern TaskHandle_t obstacleAvoidanceModeTaskHandle;

// Prototipos de Periféricos y Control
void setupPeripherals();
void startPeripheralTasks();
void centerServos();
void turnLaserOn(bool state);
void ledIndicator(int blinkTimes, int delayTimeMS);
void ledIndicator(int state);
void leftRearLed(int state);
void rightRearLed(int state);
void restorePeripheralsAfterI2CResetLocked();

// Tareas FreeRTOS Activas
void playMelody(void *parameters);
void obstacleAvoidanceMode(void *parameters);

#endif // PERIPHERALS_H