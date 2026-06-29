#include "config.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include "wifi_server.h"
#include "camera_setup.h"
#include <esp_camera.h>

TaskHandle_t arduinoOTATask;
TaskHandle_t sendCameraPictureTask;
TaskHandle_t cleanupWSClientsTask;
extern TaskHandle_t servoControlTaskHandle;
extern void servoControlTask(void *parameters);

void initTasks()
{
    // Tarea OTA
    xTaskCreatePinnedToCore(arduinoOTA_task, "arduinoOTA", 1024 * 8, NULL, 2, &arduinoOTATask, 0);

    // Tarea de Telemetría
    xTaskCreatePinnedToCore(sendTelemetryTask, "Telemetry", 2048, NULL, 0, NULL, 0);

    // MUEVE ESTAS DOS TAREAS AQUÍ (Asegúrate de tener declarados los TaskHandle_t correspondientes)
    xTaskCreatePinnedToCore(sendCameraPicture, "sendCameraPicture", STACK_SIZE, NULL, 2, &sendCameraPictureTask, 0);
    xTaskCreatePinnedToCore(cleanupWSClients_task, "cleanupWSClients", 2048, NULL, 1, &cleanupWSClientsTask, 0);

    // --- NUEVO: Lanzar el controlador asíncrono de los servos en el Core 1 ---
    xTaskCreatePinnedToCore(
        servoControlTask,
        "ServoControl",
        2048,
        NULL,
        1, // Prioridad normal
        &servoControlTaskHandle,
        1); // Forzado al núcleo 1
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