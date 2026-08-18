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
    Serial.begin(115200);
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