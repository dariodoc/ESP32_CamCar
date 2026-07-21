#include "config.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include "wifi_server.h"
#include <esp_camera.h>

TaskHandle_t sendCameraPictureTask;

void initTasks()
{
    // Tarea de Streaming (Core 1 para dejar Core 0 exclusivo al stack WiFi)
    xTaskCreatePinnedToCore(sendCameraPicture, "sendCameraPicture", 1024 * 8, NULL, 2, &sendCameraPictureTask, 1);

    // Tareas de Periféricos y Actuadores (Core 1)
    xTaskCreatePinnedToCore(servoControlTask, "ServoControl", 1024 * 2, NULL, 1, &servoControlTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(playMelody, "playMelody", STACK_SIZE, NULL, 1, &playMelodyTask, CONFIG_ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(obstacleAvoidanceMode, "obstacleAvoidanceMode", STACK_SIZE, NULL, 2, &obstacleAvoidanceModeTask, CONFIG_ARDUINO_RUNNING_CORE);
}

void setup()
{
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
    ArduinoOTA.handle();

    static int lastSpeed = -1;
    static int lastDirection = -1;
    static unsigned long lastCleanupTime = 0;

    unsigned long currentMillis = millis();

    // Mantenimiento periódico de WebSockets
    if (currentMillis - lastCleanupTime >= 2000)
    {
        lastCleanupTime = currentMillis;
        cleanupWSClients();
    }

    // Filtro inteligente de obstáculos
    if (obstacleFound && (targetDirection == FORWARD || targetDirection == FORWARDLEFT || targetDirection == FORWARDRIGHT))
    {
        toneToPlay(buzzerPin, buzzerChannel, NOTE_G5, 200);
        targetDirection = STOP;
    }

    // Actualización de motores
    if (targetDirection != lastDirection || motorSpeed != lastSpeed)
    {
        moveCar(targetDirection);
        lastDirection = targetDirection;
        lastSpeed = motorSpeed;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}