#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include "AsyncTCP.h"

#define WS_MAX_QUEUED_MESSAGES 1
#include "ESPAsyncWebServer.h"

#include <sstream>
#include <ArduinoOTA.h>

#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Manejador del Mutex para proteger el bus I2C
extern SemaphoreHandle_t i2cMutex;

// Helper para intentar tomar el Mutex con un Timeout seguro (p. ej. 20ms)
bool lockI2C(TickType_t timeoutMs = 20);
void unlockI2C();

// #define DEBUG // Descomenta para habilitar el monitor Serial

// --- PINES DE LA CÁMARA ESP32-CAM ---
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// --- PINES Y CONSTANTES DE PERIFÉRICOS ---
const int builtinLedPin = 33;
const int lightPin = 4;
const int buzzerPin = 13;
const int buzzerChannel = 3;
const int panPin = 12;
const int tiltPin = 2;
const int panCenter = 75;
const int tiltCenter = 90;

// --- CONSTANTES DEL MOTOR ---
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define STOP 0
#define FORWARDLEFT 5
#define FORWARDRIGHT 6
#define BACKLEFT 7
#define BACKRIGHT 8

// --- CONFIGURACIÓN DE TAREAS (FreeRTOS) ---
#define STACK_SIZE 1024 * 4

// --- Declaraciones Externas (Variables Globales Compartidas) ---

// Servidor y WebSockets
extern AsyncWebServer server;
extern AsyncWebSocket wsCamera;
extern AsyncWebSocket wsCarInput;
extern int cameraClientId;

extern volatile bool melodyOn;
extern volatile int motorSpeed;
extern volatile int currentDirection;

// Estado del coche
extern volatile bool enableLight;
extern volatile bool enableObstacleAvoidance;
extern volatile bool obstacleFound;

// Variables para desacoplar los servos y motores
extern volatile int targetPan;
extern volatile int targetTilt;
extern volatile int targetDirection;

// Handlers de Tareas
extern TaskHandle_t playMelodyTask;
extern TaskHandle_t obstacleAvoidanceModeTask;

#endif // CONFIG_H