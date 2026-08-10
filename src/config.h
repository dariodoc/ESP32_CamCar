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
const int lightPin = 33;
const int buzzerPin = 32;
const int buzzerChannel = 3;
const int panPin = 12;
const int tiltPin = 13;
const int panCenter = 75;
const int tiltCenter = 90;

// --- PINES DE LOS MOTORES ---
const int PWMPinleftMotor = 1;   // Pin del esp32 para el motor izquierdo
const int PWMPinrightMotor = 3;  // Pin del esp32 para el motor derecho
const int In1pinleftMotor1 = 3;  // Pin de PCF8574 para el motor izquierdo
const int In2pinleftMotor1 = 4;  // Pin de PCF8574 para el motor izquierdo
const int In1pinrightMotor2 = 1; // Pin de PCF8574 para el motor derecho
const int In2pinrightMotor2 = 0; // Pin de PCF8574 para el motor derecho
const int STBYpin = 2;           // Pin de PCF8574 para Standby del controlador de motor
const int offset = 1;            // 1 para normal, -1 para invertir dirección

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

// --- CONFIGURACIÓN DE FREERTOS ---
#define STACK_SIZE (1024 * 4)

#endif // CONFIG_H