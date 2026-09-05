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

    // 🚀 LANZAMIENTO DE TAREAS FREERTOS PARA MÚSICA Y OBSTÁCULOS
    xTaskCreatePinnedToCore(playMelody, "PlayMelodyTask", STACK_SIZE, NULL, 1, &playMelodyTaskHandle, 1);
    xTaskCreatePinnedToCore(obstacleAvoidanceMode, "ObstacleTask", STACK_SIZE, NULL, 1, &obstacleAvoidanceModeTaskHandle, 1);
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

void setupUltrasonic()
{
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
}

float getDistanceCM()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 25000); // Timeout de ~25ms (~4 metros)

    if (duration == 0)
        return -1.0;

    return (duration * 0.0343) / 2.0;
}

// 🚀 Tarea unificada: Infrarrojos (PCF8574) + Ultrasónico (Trig 33 / Echo 32)
void obstacleAvoidanceMode(void *parameters)
{
    setupUltrasonic();
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        if (!enableObstacleAvoidance)
        {
            obstacleFound = false;
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            lastWakeTime = xTaskGetTickCount();
        }

        bool irObstacle = false;

        // 1. Lectura segura del PCF8574 (0x20) sin afectar los motores delanteros
        if (lockI2C(20))
        {
            // Solicitamos el estado actual del puerto
            Wire.requestFrom(0x20, 1);
            if (Wire.available())
            {
                uint8_t currentData = Wire.read();

                // 🚀 MÁSCARA INTELIGENTE: Solo aseguramos que P0-P3 (sensores) tengan pull-up (1)
                // Manteniendo intactos los bits P4-P7 de los motores delanteros
                uint8_t safeReadMask = currentData | 0x0F;

                Wire.beginTransmission(0x20);
                Wire.write(safeReadMask);
                Wire.endTransmission();

                // Evaluamos el estado real de los 4 sensores IR (P0 a P3)
                bool ir1 = !(currentData & (1 << obstacleDetectorPin1));
                bool ir2 = !(currentData & (1 << obstacleDetectorPin2));
                bool ir3 = !(currentData & (1 << obstacleDetectorPin3));
                bool ir4 = !(currentData & (1 << obstacleDetectorPin4));

                irObstacle = (ir1 || ir2 || ir3 || ir4);
            }
            unlockI2C();
        }

        // 2. Lectura del Ultrasónico (Trig 33 / Echo 32)
        float distance = getDistanceCM();
        bool usObstacle = (distance >= 2.0 && distance <= 10.0);

#ifdef DEBUG
        if (irObstacle)
        {
            TelnetStream.println("🛑 Obstáculo por INFRARROJOS\r");
        }
        else if (usObstacle)
        {
            TelnetStream.printf("🛑 Obstáculo por ULTRASÓNICO: %.2f cm\r\n", distance);
        }
#endif

        // 3. Respuesta a obstáculo
        obstacleFound = (irObstacle || usObstacle);

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50));
    }
}