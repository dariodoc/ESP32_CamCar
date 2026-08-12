#include "i2c_manager.h"
#include <Wire.h>
#include "PCF8574.h"

extern PCF8574 motorcontrolpcf8574;
extern PCF8574 peripheralspcf8574;

static SemaphoreHandle_t i2cMutex = NULL;
static int i2cFailCounter = 0;

void initI2CManager()
{
    if (i2cMutex == NULL)
    {
        i2cMutex = xSemaphoreCreateMutex();
    }
}

bool lockI2C(TickType_t timeoutMs)
{
    if (i2cMutex == NULL)
        return false;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE)
    {
        i2cFailCounter = 0;
        return true;
    }

    i2cFailCounter++;
#ifdef DEBUG
    Serial.printf("⚠️ Fallo de acceso a I2C (%d/3)\n", i2cFailCounter);
#endif

    if (i2cFailCounter >= 3)
    {
#ifdef DEBUG
        Serial.println("🔄 Reseteando hardware I2C por congelamiento de bus...");
#endif
        Wire.end();
        vTaskDelay(pdMS_TO_TICKS(10));
        Wire.begin(26, 27);
        Wire.setClock(400000);
        Wire.setTimeOut(50);

        motorcontrolpcf8574.begin();
        peripheralspcf8574.begin();

        i2cFailCounter = 0;
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