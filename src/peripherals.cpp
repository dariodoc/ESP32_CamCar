#include "config.h"
#include "peripherals.h"
#include "i2c_manager.h"
#include "motor_control.h"
#include "PCF8574.h"
#include "ESP32Servo.h"
#include "Melodies.h"
#include <Wire.h>

PCF8574 leftmotorscontrolpcf8574(&Wire, 0x20);
PCF8574 rightmotorscontrolpcf8574(&Wire, 0x24);

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

// 🚀 Función para centrar físicamente y actualizar contadores
void centerServos()
{
    panDirection = 0;
    tiltDirection = 0;
    currentPan = panCenter;
    currentTilt = tiltCenter;
    panServo.write(panCenter);
    tiltServo.write(tiltCenter);
}

// --- Control de LEDs Traseros ---
void leftRearLed(int state)
{
    if (lockI2C(20))
    {
        rightmotorscontrolpcf8574.digitalWrite(leftRearLedPin, !state);
        unlockI2C();
    }
}

void rightRearLed(int state)
{
    if (lockI2C(20))
    {
        rightmotorscontrolpcf8574.digitalWrite(rightRearLedPin, !state);
        unlockI2C();
    }
}

void configurePCFPins()
{
    leftmotorscontrolpcf8574.pinMode(In1pinleftMotor1, OUTPUT);
    leftmotorscontrolpcf8574.pinMode(In2pinleftMotor1, OUTPUT);
    leftmotorscontrolpcf8574.pinMode(STBYpin, OUTPUT);
    leftmotorscontrolpcf8574.pinMode(laserPin, OUTPUT);

    rightmotorscontrolpcf8574.pinMode(In1pinrightMotor2, OUTPUT);
    rightmotorscontrolpcf8574.pinMode(In2pinrightMotor2, OUTPUT);
    rightmotorscontrolpcf8574.pinMode(obstacleDetectorPin, INPUT);
    rightmotorscontrolpcf8574.pinMode(leftRearLedPin, OUTPUT);
    rightmotorscontrolpcf8574.pinMode(rightRearLedPin, OUTPUT);
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

    leftmotorscontrolpcf8574.begin();
    rightmotorscontrolpcf8574.begin();

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
        leftmotorscontrolpcf8574.digitalWrite(laserPin, !state);
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

void servoControlTask(void *parameters)
{
    const int stepSize = 1;

    for (;;)
    {
        bool moved = false;

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
            detect = rightmotorscontrolpcf8574.digitalRead(obstacleDetectorPin);
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