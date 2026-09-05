#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// 🚀 Define DEBUG para habilitar las impresiones por el puerto serial
// #define DEBUG

#ifdef DEBUG
#include <TelnetStream.h>
#endif
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

const int buzzerChannel = 3; // Canal del buzzer
const int panCenter = 90;    // Posición central del pan
const int tiltCenter = 90;   // Posición central del tilt

const int motorFLoffset = 1; // 1 para normal, -1 para invertir dirección
const int motorBLoffset = 1; // 1 para normal, -1 para invertir dirección
const int motorFRoffset = 1; // 1 para normal, -1 para invertir dirección
const int motorBRoffset = 1; // 1 para normal, -1 para invertir dirección

// --- PINES DE PERIFÉRICOS ---

const int builtinLedPin = 2;    // Pin del LED incorporado en la placa ESP32
const int laserPin = 13;        // Pin del PCA9685 para el láser
const int leftRearLedPin = 15;  // Pin del PCA9685 para el led trasero izquierdo
const int rightRearLedPin = 14; // Pin del PCA9685 para el led trasero derecho
// --- PINES DEL SENSOR ULTRASÓNICO ---
const int trigPin = 33; // Disparo (Trigger) - Salida ESP32 (3.3V)
const int echoPin = 32; // Eco (Echo) - Entrada ESP32 (Requiere divisor de voltaje a 3.3V)

const int obstacleDetectorPin1 = 3; // Pin del pcf8574
const int obstacleDetectorPin2 = 2; // Pin del pcf8574
const int obstacleDetectorPin3 = 1; // Pin del pcf8574
const int obstacleDetectorPin4 = 0; // Pin del pcf8574

const int buzzerPin = 12; // Pin del ESP32 para el buzzer (PWM)
const int panPin = 4;     // Pin del PCA9685 para el pan
const int tiltPin = 5;    // Pin del PCA9685 para el tilt

// --- PINES DEL PCF8574 Y PCA9685 PARA LOS MOTORES ---

const int motorFLIn1pin = 4; // Pin de PCF8574 para el motor delantero izquierdo
const int motorFLIn2pin = 5; // Pin de PCF8574 para el motor delantero izquierdo
const int motorFLPWMPin = 0; // Pin del PCA9685 para el motor delantero izquierdo

const int motorFRIn1pin = 7; // Pin de PCF8574 para el motor delantero derecho
const int motorFRIn2pin = 6; // Pin de PCF8574 para el motor delantero derecho
const int motorFRPWMPin = 1; // Pin del PCA9685 para el motor delantero derecho

const int motorBLIn1pin = 2; // Pin de PCF8574 para el motor trasero izquierdo
const int motorBLIn2pin = 3; // Pin de PCF8574 para el motor trasero izquierdo
const int motorBLPWMPin = 3; // Pin del PCA9685 para el motor trasero izquierdo

const int motorBRIn1pin = 1; // Pin de PCF8574 para el motor trasero derecho
const int motorBRIn2pin = 0; // Pin de PCF8574 para el motor trasero derecho
const int motorBRPWMPin = 2; // Pin del PCA9685 para el motor trasero derecho

const int STBYpin = 4; // Pin de PCF8574 para Standby

// --- CONFIGURACIÓN DE FREERTOS ---
#define STACK_SIZE (1024 * 4)

#endif // CONFIG_H