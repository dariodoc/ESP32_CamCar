#include "config.h"
#include "i2c_manager.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include "wifi_server.h"
#include "display_control.h"
#include <ArduinoOTA.h>

void setup()
{
    setCpuFrequencyMhz(240);

#ifdef DEBUG
    Serial.begin(115200);
#endif

    initI2CManager();   // 1. Hardware I2C
    setupPeripherals(); // 2. Expansores PCF y PCA

    initDisplayTask(); // 3. Iniciar tarea del display (ejecuta el reset por P5)

    // 🚀 Pausa de guarda obligatoria: Espera a que la pantalla complete sus 500ms de secuencia antes de encender la radio Wi-Fi
    vTaskDelay(pdMS_TO_TICKS(600));

    setupCamera(); // 4. Cámara
    initWiFi();    // 5. Wi-Fi (Picos de RF aislados)
}

void loop()
{
    ArduinoOTA.handle();

    // Como el servidor corre en su propia tarea FreeRTOS, el loop solo asiste a OTA
    vTaskDelay(pdMS_TO_TICKS(50));
}