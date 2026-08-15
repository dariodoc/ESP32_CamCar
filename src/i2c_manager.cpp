#include "config.h"
#include "i2c_manager.h"
#include <Wire.h>
#include "PCF8574.h"
#include "peripherals.h"

extern PCF8574 leftmotorscontrolpcf8574;
extern PCF8574 rightmotorscontrolpcf8574;

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

    // Intentamos tomar el mutex normalmente
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
        // Liberación de seguridad por si una tarea colgada se quedó con el mutex
        xSemaphoreGive(i2cMutex);

        Wire.end();
        vTaskDelay(pdMS_TO_TICKS(10));

        Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
        vTaskDelay(pdMS_TO_TICKS(50));
        Wire.setClock(400000);
        Wire.setTimeOut(50);

        leftmotorscontrolpcf8574.begin();
        rightmotorscontrolpcf8574.begin();

        configurePCFPins();

        i2cFailCounter = 0;

        // Intentamos tomar el mutex inmediatamente tras la recuperación
        return (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
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