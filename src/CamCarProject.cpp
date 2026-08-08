#include "config.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include "wifi_server.h"
#include <esp_camera.h>
#include "Melodies.h"
#include <Wire.h> // 👈 1. Necesario para declarar 'Wire' en este archivo

// 👈 2. Declaramos que estos objetos existen en peripherals.cpp
#include "PCF8574.h"
extern PCF8574 motorcontrolpcf8574;
extern PCF8574 peripheralspcf8574;

//TaskHandle_t sendCameraPictureTask;

SemaphoreHandle_t i2cMutex = NULL;

// En CamCarProject.cpp
static int i2cFailCounter = 0;

bool lockI2C(TickType_t timeoutMs)
{
    if (i2cMutex == NULL)
        return false;

    // Intentamos tomar el Mutex
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE)
    {
        i2cFailCounter = 0; // Operación exitosa, reseteamos contador
        return true;
    }

    // Si no pudimos tomar el Mutex tras el timeout (bus colgado por ruido)
    i2cFailCounter++;
#ifdef DEBUG
    Serial.printf("⚠️ Fallo de acceso a I2C (%d/3)\n", i2cFailCounter);
#endif

    // Si falla 3 veces consecutivas, reseteamos el hardware I2C automáticamente
    if (i2cFailCounter >= 3)
    {
#ifdef DEBUG
        Serial.println("🔄 Reseteando hardware I2C por congelamiento de bus...");
#endif
        Wire.end();
        vTaskDelay(pdMS_TO_TICKS(10));
        Wire.begin(14, 15);
        Wire.setClock(400000);
        Wire.setTimeOut(50);

        motorcontrolpcf8574.begin();
        peripheralspcf8574.begin();

        i2cFailCounter = 0;
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

void initTasks()
{
    // Tarea de Streaming (Core 1 para dejar Core 0 exclusivo al stack WiFi)
   // xTaskCreatePinnedToCore(sendCameraPicture, "sendCameraPicture", 1024 * 8, NULL, 2, &sendCameraPictureTask, 0);

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

    if (!SPIFFS.begin(true)) // El parámetro 'true' formatea SPIFFS si falla al montar
    {
#ifdef DEBUG
        Serial.println("Error montando SPIFFS, pero continuando...");
#endif
    }

    i2cMutex = xSemaphoreCreateMutex();

    if (i2cMutex == NULL)
    {
        Serial.println("❌ Error crítico: No se pudo crear el Mutex de I2C");
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

    // 🔒 Copia atómica para evitar Race Conditions con la tarea de WebSocket
    int currentTargetDir = targetDirection;
    int currentMotorSpeed = motorSpeed;

    // Filtro inteligente de obstáculos
    if (obstacleFound && (currentTargetDir == FORWARD || currentTargetDir == FORWARDLEFT || currentTargetDir == FORWARDRIGHT))
    {
        toneToPlay(buzzerPin, buzzerChannel, NOTE_G5, 200);
        currentTargetDir = STOP;
        targetDirection = STOP;
    }

    // Actualización de motores con copia local segura
    if (currentTargetDir != lastDirection || currentMotorSpeed != lastSpeed)
    {
        moveCar(currentTargetDir);
        lastDirection = currentTargetDir; // Garantiza coincidencia exacta con lo que se envió a los motores
        lastSpeed = currentMotorSpeed;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}