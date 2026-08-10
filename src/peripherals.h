#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Variables Compartidas
extern volatile int targetPan;
extern volatile int targetTilt;
extern volatile bool enableLight;
extern volatile bool melodyOn;
extern volatile bool enableObstacleAvoidance;
extern volatile bool obstacleFound;

// Handlers de Tareas
extern TaskHandle_t playMelodyTaskHandle;
extern TaskHandle_t obstacleAvoidanceModeTaskHandle;
extern TaskHandle_t servoControlTaskHandle;

// Prototipos de Periféricos y LEDs
void setupPeripherals();
void ledIndicator(int blinkTimes, int delayTimeMS);
void ledIndicator(int state);
void leftBackLed(int state);
void rightBackLed(int state);

// Tareas FreeRTOS
void playMelody(void *parameters);
void obstacleAvoidanceMode(void *parameters);
void servoControlTask(void *parameters);

#endif // PERIPHERALS_H