#include "config.h"
#include "i2c_manager.h"
#include <Wire.h>
#include "PCF8574.h"
#include "Adafruit_PWMServoDriver.h"
#include "peripherals.h"

extern PCF8574 FMCpcf8574;
extern PCF8574 BMCpcf8574;
extern Adafruit_PWMServoDriver pca9685;

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
   // Serial.printf("⚠️ Fallo de acceso a I2C (%d/3)\n", i2cFailCounter);
#endif

    if (i2cFailCounter >= 3)
    {
#ifdef DEBUG
      //  Serial.println("🔄 Reseteando hardware I2C por congelamiento de bus...");
#endif
        // Liberación de seguridad por si una tarea colgada se quedó con el mutex
        xSemaphoreGive(i2cMutex);

        Wire.end();
        vTaskDelay(pdMS_TO_TICKS(10));

        Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
        vTaskDelay(pdMS_TO_TICKS(50));

        // 🚀 Mantener reloj en 100 kHz para evitar ruina de señal por interferencia
        Wire.setClock(100000);
        Wire.setTimeOut(100);

        FMCpcf8574.begin();
        BMCpcf8574.begin();
        pca9685.begin();
        pca9685.setPWMFreq(50);

        // 🚀 Sincronizar en frío el bus en HIGH (0xFF) tras el reinicio del bus
        Wire.beginTransmission(0x20);
        Wire.write(0xFF);
        Wire.endTransmission();

        Wire.beginTransmission(0x24);
        Wire.write(0xFF);
        Wire.endTransmission();

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