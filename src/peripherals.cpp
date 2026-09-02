#include "config.h"
#include "peripherals.h"
#include "i2c_manager.h"
#include "motor_control.h"
#include "PCF8574.h"
#include "Adafruit_PWMServoDriver.h"
#include "Melodies.h"
#include <Wire.h>

PCF8574 FMCpcf8574(&Wire, 0x20);
PCF8574 BMCpcf8574(&Wire, 0x24);
Adafruit_PWMServoDriver pca9685(0x40, Wire);

volatile bool enableLaser = false;
volatile bool melodyOn = false;
volatile bool enableObstacleAvoidance = false;
volatile bool obstacleFound = false;

volatile int panDirection = 0;
volatile int tiltDirection = 0;

// Variables de posición global para mantener sincronía interna
static int currentPan = panCenter;   // 75
static int currentTilt = tiltCenter; // 90

TaskHandle_t playMelodyTaskHandle = NULL;
TaskHandle_t obstacleAvoidanceModeTaskHandle = NULL;
TaskHandle_t servoControlTaskHandle = NULL;

volatile bool isCentering = false; // Bandera de centrado suave

// --- Control de LEDs Traseros ---
void leftRearLed(int state)
{
    if (lockI2C(20))
    {
        int pwmValue = state ? 0 : 4095;
        pca9685.setPWM(leftRearLedPin, 0, pwmValue);
        unlockI2C();
    }
}

void rightRearLed(int state)
{
    if (lockI2C(20))
    {
        int pwmValue = state ? 0 : 4095;
        pca9685.setPWM(rightRearLedPin, 0, pwmValue);
        unlockI2C();
    }
}

void writeServoPCA(uint8_t channel, int angle)
{
    int constrainedAngle = constrain(angle, 0, 180);
    int uS = 0;

    // Calibración independiente por servo / canal
    if (channel == panPin) // Canal 4 (Pan)
    {
        // 🚀
        uS = map(constrainedAngle, 0, 180, 600, 2400);
    }
    else if (channel == tiltPin) // Canal 5 (Tilt)
    {
        // Rango protegido para el Tilt para evitar que se brinque el engrane
        uS = map(constrainedAngle, 0, 180, 600, 2400);
    }
    else
    {
        uS = map(constrainedAngle, 0, 180, 600, 2400);
    }

    if (lockI2C(20))
    {
        pca9685.writeMicroseconds(channel, uS);
        unlockI2C();
    }
}

// --- Control de Ángulos Pan y Tilt usando PCA9685 ---
void setPanAngle(int angle)
{
    currentPan = constrain(angle, 0, 180);
    writeServoPCA(panPin, currentPan); // panPin = 4 en PCA9685
}

void setTiltAngle(int angle)
{
    // 🚀 Restringimos el rango a 10° - 170° para evitar que el servo toque el tope mecánico y se desenganche
    int safeAngle = constrain(angle, 10, 170);

    currentTilt = 180 - safeAngle; // Mantiene la inversión vertical
    writeServoPCA(tiltPin, currentTilt);
}

void configurePCFPins()
{
    FMCpcf8574.pinMode(motorFLIn1pin, OUTPUT);
    FMCpcf8574.pinMode(motorFLIn2pin, OUTPUT);
    FMCpcf8574.pinMode(motorFRIn1pin, OUTPUT);
    FMCpcf8574.pinMode(motorFRIn2pin, OUTPUT);

    BMCpcf8574.pinMode(motorBRIn1pin, OUTPUT);
    BMCpcf8574.pinMode(motorBRIn2pin, OUTPUT);
    BMCpcf8574.pinMode(motorBLIn1pin, OUTPUT);
    BMCpcf8574.pinMode(motorBLIn2pin, OUTPUT);

    BMCpcf8574.pinMode(STBYpin, OUTPUT);

    FMCpcf8574.pinMode(obstacleDetectorPin1, INPUT_PULLUP);
    FMCpcf8574.pinMode(obstacleDetectorPin2, INPUT_PULLUP);
    FMCpcf8574.pinMode(obstacleDetectorPin3, INPUT_PULLUP);
    FMCpcf8574.pinMode(obstacleDetectorPin4, INPUT_PULLUP);
}

void setupPeripherals()
{
    pinMode(builtinLedPin, OUTPUT);
    digitalWrite(builtinLedPin, HIGH); // LED OFF
    ledcDetachPin(buzzerPin);

    Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
    vTaskDelay(pdMS_TO_TICKS(100)); // 100 ms para arranque en frío

    // 🚀 Reducir a 100 kHz para mayor tolerancia al ruido eléctrico
    Wire.setClock(100000);
    Wire.setTimeOut(100);

    FMCpcf8574.begin();
    BMCpcf8574.begin();
    pca9685.begin();
    pca9685.setPWMFreq(50);

    configurePCFPins(); // Lógica centralizada de pines

    turnLaserOn(false);

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
        int pwmValue = state ? 0 : 4095;
        pca9685.setPWM(laserPin, 0, pwmValue);
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
    setPanAngle(panCenter);   // panCenter = 75 en config.h
    setTiltAngle(tiltCenter); // tiltCenter = 90 en config.h
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
            detect = FMCpcf8574.digitalRead(obstacleDetectorPin1);
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