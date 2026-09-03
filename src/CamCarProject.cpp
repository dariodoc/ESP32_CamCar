#include "config.h"
#include "i2c_manager.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include "wifi_server.h"
#include <ArduinoOTA.h>

void setup()
{
    setCpuFrequencyMhz(240);

#ifdef DEBUG
    // 🚀 Inicializa el puerto serial a 115200 baudios
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(100)); // Pequeña pausa para estabilizar el puerto
    Serial.println("\n--- INICIANDO ROBOT MECANUM ---");
#endif

    initI2CManager();
    setupCamera();
    setupPeripherals();
    initWiFi();
}

void loop()
{
    ArduinoOTA.handle();

    // Atiende peticiones de la App Freenove en el puerto 4000
    loopCmdServer();

    vTaskDelay(pdMS_TO_TICKS(10));
}