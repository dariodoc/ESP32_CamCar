#include "config.h"
#include "peripherals.h"
#include "i2c_manager.h"
#include "motor_control.h"
#include "PCF8574.h"
#include "ESP32Servo.h"
#include "Melodies.h"
#include <Wire.h>

PCF8574 LMCpcf8574(&Wire, 0x20);
PCF8574 RMCpcf8574(&Wire, 0x24);

Servo panServo;
Servo tiltServo;

volatile bool enableLaser = false;
volatile bool melodyOn = false;
volatile bool enableObstacleAvoidance = false;
volatile bool obstacleFound = false;

volatile int panDirection = 0;
volatile int tiltDirection = 0;

// 🚀 Variables de posición global para mantener sincronía interna
static int currentPan = panCenter;   // 75
static int currentTilt = tiltCenter; // 90

TaskHandle_t playMelodyTaskHandle = NULL;
TaskHandle_t obstacleAvoidanceModeTaskHandle = NULL;
TaskHandle_t servoControlTaskHandle = NULL;

volatile bool isCentering = false; // 👈 Bandera de centrado suave

// --- Control de LEDs Traseros ---
void leftRearLed(int state)
{
    if (lockI2C(20))
    {
        LMCpcf8574.digitalWrite(leftRearLedPin, !state);
        unlockI2C();
    }
}

void rightRearLed(int state)
{
    if (lockI2C(20))
    {
        RMCpcf8574.digitalWrite(rightRearLedPin, !state);
        unlockI2C();
    }
}

void configurePCFPins()
{
    LMCpcf8574.pinMode(motorFLIn1pin, OUTPUT);
    LMCpcf8574.pinMode(motorFLIn2pin, OUTPUT);
    LMCpcf8574.pinMode(motorBLIn1pin, OUTPUT);
    LMCpcf8574.pinMode(motorBLIn2pin, OUTPUT);
    LMCpcf8574.pinMode(leftSTBYpin, OUTPUT);

    LMCpcf8574.pinMode(leftRearLedPin, OUTPUT);
    LMCpcf8574.pinMode(laserPin, OUTPUT);

    RMCpcf8574.pinMode(motorFRIn1pin, OUTPUT);
    RMCpcf8574.pinMode(motorFRIn2pin, OUTPUT);
    RMCpcf8574.pinMode(motorBRIn1pin, OUTPUT);
    RMCpcf8574.pinMode(motorBRIn2pin, OUTPUT);
    RMCpcf8574.pinMode(rightSTBYpin, OUTPUT);

    RMCpcf8574.pinMode(rightRearLedPin, OUTPUT);
    RMCpcf8574.pinMode(obstacleDetectorPin, INPUT);
}

void setupPeripherals()
{
    pinMode(builtinLedPin, OUTPUT);
    digitalWrite(builtinLedPin, HIGH); // LED OFF
    ledcDetachPin(buzzerPin);

    Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
    vTaskDelay(pdMS_TO_TICKS(100)); // 👈 100 ms para arranque en frío
    Wire.setClock(400000);
    Wire.setTimeOut(50);

    LMCpcf8574.begin();
    RMCpcf8574.begin();

    configurePCFPins(); // 👈 Lógica centralizada de pines

    turnLaserOn(false);
    panServo.attach(panPin);
    tiltServo.attach(tiltPin);
    centerServos();

    ledIndicator(3, 100);
}

void ledIndicator(int blinkTimes, int delayTimeMS)
{
    for (int i = 0; i < blinkTimes; i++)
    {
        digitalWrite(builtinLedPin, LOW);
        toneToPlay(buzzerPin, buzzerChannel, NOTE_G5, delayTimeMS);
        digitalWrite(builtinLedPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(delayTimeMS));
    }
}

void ledIndicator(int state)
{
    digitalWrite(builtinLedPin, !state);
}

void turnLaserOn(bool state)
{
    if (lockI2C(20))
    {
        LMCpcf8574.digitalWrite(laserPin, !state);
        unlockI2C();
    }
}

void playMelody(void *parameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        gameOfThrones(buzzerPin, buzzerChannel);
        melodyOn = false;
        ledcWriteTone(buzzerChannel, 0);
    }
}

void centerServos()
{
    panDirection = 0;
    tiltDirection = 0;
    isCentering = true; // 🚀 Notifica a servoControlTask que inicie el centrado suave
}

void servoControlTask(void *parameters)
{
    const int stepSize = 1;

    for (;;)
    {
        bool moved = false;

        // 🚀 MODO CENTRADO SUAVE (EASING)
        if (isCentering)
        {
            bool panDone = (currentPan == panCenter);
            bool tiltDone = (currentTilt == tiltCenter);

            // Suavizado PAN
            if (!panDone)
            {
                if (currentPan < panCenter)
                    currentPan += stepSize;
                else if (currentPan > panCenter)
                    currentPan -= stepSize;
                panServo.write(currentPan);
                moved = true;
            }

            // Suavizado TILT
            if (!tiltDone)
            {
                if (currentTilt < tiltCenter)
                    currentTilt += stepSize;
                else if (currentTilt > tiltCenter)
                    currentTilt -= stepSize;
                tiltServo.write(currentTilt);
                moved = true;
            }

            // Si ambos alcanzaron el centro objetivo, se desactiva el modo centrado
            if (panDone && tiltDone)
            {
                isCentering = false;
            }
        }
        // 🚀 MODO CONTROL MANUAL (Joystick / Botones)
        else
        {
            // --- Manejo Servo PAN ---
            if (panDirection == 1 && currentPan < 180)
            { // Mover Izquierda
                currentPan += stepSize;
                if (currentPan > 180)
                    currentPan = 180;
                panServo.write(currentPan);
                moved = true;
            }
            else if (panDirection == 2 && currentPan > 0)
            { // Mover Derecha
                currentPan -= stepSize;
                if (currentPan < 0)
                    currentPan = 0;
                panServo.write(currentPan);
                moved = true;
            }

            // --- Manejo Servo TILT ---
            if (tiltDirection == 1 && currentTilt > 0)
            { // Mover Arriba
                currentTilt -= stepSize;
                if (currentTilt < 0)
                    currentTilt = 0;
                tiltServo.write(currentTilt);
                moved = true;
            }
            else if (tiltDirection == 2 && currentTilt < 180)
            { // Mover Abajo
                currentTilt += stepSize;
                if (currentTilt > 180)
                    currentTilt = 180;
                tiltServo.write(currentTilt);
                moved = true;
            }
        }

        // Mantiene una cadencia fluida de 20 ms por grado cuando hay movimiento
        vTaskDelay(pdMS_TO_TICKS(moved ? 20 : 50));
    }
}

void obstacleAvoidanceMode(void *parameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    int lastDetect = -1;

    for (;;)
    {
        if (!enableObstacleAvoidance)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            lastDetect = -1;
        }

        int detect = HIGH;

        if (lockI2C(20))
        {
            detect = RMCpcf8574.digitalRead(obstacleDetectorPin);
            unlockI2C();
        }

        if (detect != lastDetect)
        {
            obstacleFound = (detect == LOW);
            lastDetect = detect;
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50));
    }
}