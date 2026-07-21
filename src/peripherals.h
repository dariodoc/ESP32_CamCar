#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <Arduino.h>

// En peripherals.h
extern volatile int targetPan;
extern volatile int targetTilt;

// Handlers de Tareas de Periféricos
extern TaskHandle_t playMelodyTask;
extern TaskHandle_t obstacleAvoidanceModeTask;
extern TaskHandle_t servoControlTaskHandle;

// Prototypes
void setupPeripherals();
void ledIndicator(int blinkTimes, int delayTimeMS);
void ledIndicator(int state);
void leftBackLed(int state);
void rightBackLed(int state);

// Tareas FreeRTOS de Periféricos
void playMelody(void *parameters);
void obstacleAvoidanceMode(void *parameters);
void servoControlTask(void *parameters);

#endif // PERIPHERALS_H