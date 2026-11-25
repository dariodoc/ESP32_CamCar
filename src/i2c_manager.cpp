#include "config.h"
#include "i2c_manager.h"
#include "custom_motor_driver.h"
#include "motor_control.h"
#include "peripherals.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

extern Adafruit_PWMServoDriver pca9685;

static SemaphoreHandle_t i2cMutex = NULL;
static volatile int i2cFailCounter = 0;
static portMUX_TYPE i2cFailMux = portMUX_INITIALIZER_UNLOCKED;

void initI2CManager()
{
    if (i2cMutex == NULL)
    {
        i2cMutex = xSemaphoreCreateMutex();
    }
}

// Ejecutar solo mientras el llamador posee i2cMutex
static void resetI2CHardwareLocked()
{
    Wire.end();
    vTaskDelay(pdMS_TO_TICKS(10));

    // Recuperación física del bus (I2C Bus Clear): 9 pulsos SCL para liberar SDA
    pinMode(SIOD_GPIO_NUM, INPUT_PULLUP);
    pinMode(SIOC_GPIO_NUM, OUTPUT);
    for (int i = 0; i < 9; i++)
    {
        digitalWrite(SIOC_GPIO_NUM, LOW);
        delayMicroseconds(5);
        digitalWrite(SIOC_GPIO_NUM, HIGH);
        delayMicroseconds(5);
        if (digitalRead(SIOD_GPIO_NUM) == HIGH)
        {
            break;
        }
    }
    pinMode(SIOC_GPIO_NUM, INPUT);

    Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
    vTaskDelay(pdMS_TO_TICKS(50));

    Wire.setClock(100000);
    Wire.setTimeOut(50);

    pca9685.begin();
    pca9685.setPWMFreq(50);

    // Restaurar salidas PCA9685 (láser y LEDs traseros), invalidar cachés de motores
    // y resincronizar las sombras reales de los PCF8574 (todo ya bajo i2cMutex)
    restorePeripheralsAfterI2CResetLocked();
    notifyMotorsI2CResetLocked();
    forceSyncMotorsI2C();
}

bool recoverI2CBus()
{
    if (i2cMutex == NULL)
        return false;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
        resetI2CHardwareLocked();
        taskENTER_CRITICAL(&i2cFailMux);
        i2cFailCounter = 0;
        taskEXIT_CRITICAL(&i2cFailMux);
        xSemaphoreGive(i2cMutex);
        return true;
    }
    return false;
}

bool lockI2C(TickType_t timeoutMs)
{
    if (i2cMutex == NULL)
        return false;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE)
    {
        taskENTER_CRITICAL(&i2cFailMux);
        i2cFailCounter = 0;
        taskEXIT_CRITICAL(&i2cFailMux);
        return true;
    }

    taskENTER_CRITICAL(&i2cFailMux);
    int fails = ++i2cFailCounter;
    taskEXIT_CRITICAL(&i2cFailMux);

    if (fails >= 5)
    {
        // Solo recuperar el bus si esta tarea logra adquirir legalmente el mutex
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(250)) == pdTRUE)
        {
            resetI2CHardwareLocked();
            taskENTER_CRITICAL(&i2cFailMux);
            i2cFailCounter = 0;
            taskEXIT_CRITICAL(&i2cFailMux);
            return true;
        }
    }

    return false;
}

void unlockI2C()
{
    if (i2cMutex != NULL)
    {
        xSemaphoreGive(i2cMutex);
    }
}