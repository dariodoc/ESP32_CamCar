#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Variables Compartidas
extern volatile int panDirection;  // 0 = Stop, 1 = Izq, 2 = Der
extern volatile int tiltDirection; // 0 = Stop, 1 = Arriba, 2 = Abajo
extern volatile bool enableLaser;
extern volatile bool melodyOn;
extern volatile bool enableObstacleAvoidance;
extern volatile bool obstacleFound;

// Handlers de Tareas
extern TaskHandle_t playMelodyTaskHandle;
extern TaskHandle_t obstacleAvoidanceModeTaskHandle;
extern TaskHandle_t servoControlTaskHandle;

// Prototipos de Periféricos y Control
void setupPeripherals();
void configurePCFPins(); // 🚀 Función auxiliar para configurar modos de pines I2C
void centerServos(); // 🚀 Función para sincronizar y centrar servos
void turnLaserOn(bool state);
void ledIndicator(int blinkTimes, int delayTimeMS);
void ledIndicator(int state);
void leftRearLed(int state);
void rightRearLed(int state);

// Tareas FreeRTOS
void playMelody(void *parameters);
void obstacleAvoidanceMode(void *parameters);
void servoControlTask(void *parameters);

#endif // PERIPHERALS_H