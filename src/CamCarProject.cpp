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

    initI2CManager();
    setupCamera();
    setupPeripherals();
    initWiFi();
}

void loop()
{
    ArduinoOTA.handle();

    // Como el servidor corre en su propia tarea FreeRTOS, el loop solo asiste a OTA
    vTaskDelay(pdMS_TO_TICKS(50));
}