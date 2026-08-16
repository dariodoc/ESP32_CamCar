#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ======================================================
// CONFIGURACIÓN DE PINES - FREENOVE ESP32-WROVER
// ======================================================
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4

#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// --- PINES DE PERIFÉRICOS ---
const int builtinLedPin = 2;
const int laserPin = 5;
const int leftRearLedPin = 6;
const int rightRearLedPin = 6;
const int obstacleDetectorPin = 5;

const int buzzerPin = 33;
const int buzzerChannel = 3;
const int panPin = 12;
const int tiltPin = 13;
const int panCenter = 75;
const int tiltCenter = 90;

// --- PINES DE LOS MOTORES ---

const int motorFLIn1pin = 0;  // Pin de PCF8574 para el motor delantero izquierdo
const int motorFLIn2pin = 1;  // Pin de PCF8574 para el motor delantero izquierdo
const int motorFLPWMPin = 14; // Pin del esp32 para el motor delantero izquierdo
const int motorFLoffset = 1;  // 1 para normal, -1 para invertir dirección

const int motorBLIn1pin = 3;  // Pin de PCF8574 para el motor trasero izquierdo
const int motorBLIn2pin = 4;  // Pin de PCF8574 para el motor trasero izquierdo
const int motorBLPWMPin = 14; // Pin del esp32 para el motor trasero izquierdo
const int motorBLoffset = 1;  // 1 para normal, -1 para invertir dirección

const int motorFRIn1pin = 0;  // Pin de PCF8574 para el motor delantero derecho
const int motorFRIn2pin = 1;  // Pin de PCF8574 para el motor delantero derecho
const int motorFRPWMPin = 15; // Pin del esp32 para el motor delantero derecho
const int motorFRoffset = 1;  // 1 para normal, -1 para invertir dirección

const int motorBRIn1pin = 3;  // Pin de PCF8574 para el motor trasero derecho
const int motorBRIn2pin = 4;  // Pin de PCF8574 para el motor trasero derecho
const int motorBRPWMPin = 15; // Pin del esp32 para el motor trasero derecho
const int motorBRoffset = 1;  // 1 para normal, -1 para invertir dirección

const int leftSTBYpin = 2;  // Pin de PCF8574 para Standby del controlador de motor izquierdo
const int rightSTBYpin = 2; // Pin de PCF8574 para Standby del controlador de motor derecho

// --- CONFIGURACIÓN DE FREERTOS ---
#define STACK_SIZE (1024 * 4)

#endif // CONFIG_H