#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include <sstream>
#include <ArduinoOTA.h>
#include "ESP32Servo.h"
#include "SparkFun_TB6612.h"
#include "Melodies.h"
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>

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

// Servos
extern Servo panServo;
extern Servo tiltServo;

// Motores y PCF8574
// extern PCF8574 pcf8574;
extern Motor leftMotor;
extern Motor rightMotor;

// Estado del coche
extern bool enableLight;
extern bool melodyOn;
extern bool enableObstacleAvoidance;
extern bool obstacleFound;
extern int motorSpeed;
extern int currentDirection;

// Handlers de Tareas
extern TaskHandle_t playMelodyTask;
extern TaskHandle_t obstacleAvoidanceModeTask;

#endif // CONFIG_H