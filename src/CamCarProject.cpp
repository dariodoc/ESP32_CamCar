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
// --- Variables globales para controlar los tiempos en el Loop ---
unsigned long lastServoTime = 0;
unsigned long lastCleanupTime = 0;

void initTasks()
{
    // Tarea de Telemetría
    // xTaskCreatePinnedToCore(sendTelemetryTask, "Telemetry", 2048, NULL, 0, NULL, 0);

    xTaskCreatePinnedToCore(sendCameraPicture, "sendCameraPicture", 1024 * 8, NULL, 2, &sendCameraPictureTask, 0);
  // --- NUEVO: Tarea de Servos ---
    xTaskCreatePinnedToCore(servoControlTask, "ServoControl", 1024 * 8, NULL, 1, &servoControlTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);

    // --- NUEVO: Instanciar tareas de periféricos de forma permanente ---
    // Se inician pero se dormirán inmediatamente gracias al código que pondremos en peripherals.cpp
    xTaskCreatePinnedToCore(playMelody, "playMelody", STACK_SIZE, NULL, 1, &playMelodyTask, CONFIG_ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(obstacleAvoidanceMode, "obstacleAvoidanceMode", STACK_SIZE, NULL, 2, &obstacleAvoidanceModeTask, CONFIG_ARDUINO_RUNNING_CORE);
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
    ArduinoOTA.handle();
    static int lastDirection = -1;

    // 👇 AÑADE ESTE BLOQUE PARA EVITAR QUE LA RAM EXPLOTE
    static unsigned long lastCleanupTime = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastCleanupTime >= 2000) {
        lastCleanupTime = currentMillis;
        wsCamera.cleanupClients();
        wsCarInput.cleanupClients();
    }
    // 👆 FIN DEL BLOQUE

    // --- FILTRO INTELIGENTE DE OBSTÁCULOS ---
    // Si hay un obstáculo Y el usuario intenta ir hacia adelante, forzamos un STOP
    if (obstacleFound && (targetDirection == FORWARD || targetDirection == FORWARDLEFT || targetDirection == FORWARDRIGHT)) {
        toneToPlay(buzzerPin, buzzerChannel, NOTE_G5, 200);
        targetDirection = STOP;
    }

    // Ejecutamos el movimiento (permitirá la reversa porque no será filtrada arriba)
    if (targetDirection != lastDirection) {
        moveCar(targetDirection);
        lastDirection = targetDirection;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}