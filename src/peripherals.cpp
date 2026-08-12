#include "config.h"
#include "peripherals.h"
#include "i2c_manager.h"
#include "motor_control.h"
#include "PCF8574.h"
#include "ESP32Servo.h"
#include "Melodies.h"
#include <Wire.h>

PCF8574 motorcontrolpcf8574(&Wire, 0x20);
PCF8574 peripheralspcf8574(&Wire, 0x24);

Servo panServo;
Servo tiltServo;

volatile bool enableLight = false;
volatile bool melodyOn = false;
volatile bool enableObstacleAvoidance = false;
volatile bool obstacleFound = false;

volatile int targetPan = 75;
volatile int targetTilt = 90;

TaskHandle_t playMelodyTaskHandle = NULL;
TaskHandle_t obstacleAvoidanceModeTaskHandle = NULL;
TaskHandle_t servoControlTaskHandle = NULL;

// --- Control de LEDs Traseros ---
void leftBackLed(int state)
{
    if (lockI2C(20))
    {
        peripheralspcf8574.digitalWrite(P7, !state);
        unlockI2C();
    }
}

void rightBackLed(int state)
{
    if (lockI2C(20))
    {
        peripheralspcf8574.digitalWrite(P6, !state);
        unlockI2C();
    }
}

void setupPeripherals()
{
    pinMode(builtinLedPin, OUTPUT);
    digitalWrite(builtinLedPin, HIGH); // LED OFF
    pinMode(lightPin, OUTPUT);
    digitalWrite(lightPin, LOW);
    ledcDetachPin(buzzerPin);

    Wire.begin(26, 27);
    Wire.setClock(400000);
    vTaskDelay(pdMS_TO_TICKS(100));

    motorcontrolpcf8574.begin();
    peripheralspcf8574.begin();

    motorcontrolpcf8574.pinMode(In1pinleftMotor1, OUTPUT);
    motorcontrolpcf8574.pinMode(In2pinleftMotor1, OUTPUT);
    motorcontrolpcf8574.pinMode(STBYpin, OUTPUT);
    motorcontrolpcf8574.pinMode(In1pinrightMotor2, OUTPUT);
    motorcontrolpcf8574.pinMode(In2pinrightMotor2, OUTPUT);

    peripheralspcf8574.pinMode(P5, INPUT);
    peripheralspcf8574.pinMode(P7, OUTPUT);
    peripheralspcf8574.pinMode(P6, OUTPUT);

    Wire.setTimeOut(50);

    panServo.attach(panPin);
    tiltServo.attach(tiltPin);
    panServo.write(panCenter);
    tiltServo.write(tiltCenter);

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
    static int currentPan = panCenter;
    static int currentTilt = tiltCenter;
    const int maxStep = 2;

    for (;;)
    {
        if (currentPan == targetPan && currentTilt == targetTilt)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (currentPan != targetPan)
        {
            int diff = targetPan - currentPan;
            currentPan += (abs(diff) <= maxStep) ? diff : ((diff > 0) ? maxStep : -maxStep);
            panServo.write(currentPan);
        }

        if (currentTilt != targetTilt)
        {
            int diff = targetTilt - currentTilt;
            currentTilt += (abs(diff) <= maxStep) ? diff : ((diff > 0) ? maxStep : -maxStep);
            tiltServo.write(currentTilt);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
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
            detect = peripheralspcf8574.digitalRead(P5);
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