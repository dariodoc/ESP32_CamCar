#include "config.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include "wifi_server.h"

TaskHandle_t arduinoOTATask;

void initTasks()
{
    // Tarea para actualizaciones inalámbricas (OTA)
    xTaskCreatePinnedToCore(
        arduinoOTA_task,
        "arduinoOTA",
        STACK_SIZE,
        NULL,
        0, // Prioridad baja
        &arduinoOTATask,
        0);

    // Tarea para enviar telemetría (RSSI, etc.) al cliente
    xTaskCreatePinnedToCore(
        sendTelemetryTask,
        "Telemetry",
        2048, // Esta tarea no necesita tanto stack
        NULL,
        1, // Prioridad normal
        NULL,
        0);
}

void setup()
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Deshabilitar brownout detector

#ifdef DEBUG
    Serial.begin(115200);
#endif

    if (!SPIFFS.begin(true))
    {
#ifdef DEBUG
        Serial.println("Error montando SPIFFS");
#endif
        ESP.restart();
    }

    setupPeripherals();
    setupCamera();
    initWiFi();
    initTasks();
}

void loop()
{
    // El loop queda vacío porque todo se maneja con tareas (FreeRTOS)
    vTaskDelay(portMAX_DELAY);
}