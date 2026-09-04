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

static int currentPan = panCenter;
static int currentTilt = tiltCenter;

TaskHandle_t playMelodyTaskHandle = NULL;
TaskHandle_t obstacleAvoidanceModeTaskHandle = NULL;

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
    int uS = map(constrainedAngle, 0, 180, 600, 2400);

    if (lockI2C(20))
    {
        pca9685.writeMicroseconds(channel, uS);
        unlockI2C();
    }
}

void setPanAngle(int angle)
{
    currentPan = constrain(angle, 0, 180);
    writeServoPCA(panPin, currentPan);
}

void setTiltAngle(int angle)
{
    int safeAngle = constrain(angle, 10, 170);
    currentTilt = 180 - safeAngle;
    writeServoPCA(tiltPin, currentTilt);
}

void setupPeripherals()
{
    pinMode(builtinLedPin, OUTPUT);
    digitalWrite(builtinLedPin, HIGH);
    ledcDetachPin(buzzerPin);

    Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
    vTaskDelay(pdMS_TO_TICKS(100));

    Wire.setClock(100000);
    Wire.setTimeOut(100);

    FMCpcf8574.begin();
    BMCpcf8574.begin();
    pca9685.begin();
    pca9685.setPWMFreq(50);

    // 🚀 LIBERACIÓN OBLIGATORIA DEL PUERTO AL ARRANQUE:
    // Fuerza a nivel físico que los 8 bits (incluyendo los sensores 0-3)
    // inicien como ENTRADAS (1s lógicos) antes de que el motor toque el bus
    if (lockI2C(50))
    {
        Wire.beginTransmission(0x20);
        Wire.write(0xFF);
        Wire.endTransmission();

        Wire.beginTransmission(0x24);
        Wire.write(0xFF);
        Wire.endTransmission();

        unlockI2C();
    }

    turnLaserOn(false);
    centerServos();
    ledIndicator(3, 100);
}

void ledIndicator(int state)
{
    digitalWrite(builtinLedPin, state ? HIGH : LOW);
}

void ledIndicator(int blinkTimes, int delayTimeMS)
{
    for (int i = 0; i < blinkTimes; i++)
    {
        digitalWrite(builtinLedPin, HIGH);
        toneToPlay(buzzerPin, buzzerChannel, NOTE_G5, delayTimeMS);
        digitalWrite(builtinLedPin, LOW);

        if (i < blinkTimes - 1)
        {
            vTaskDelay(pdMS_TO_TICKS(delayTimeMS));
        }
    }
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
    setPanAngle(panCenter);
    setTiltAngle(tiltCenter);
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