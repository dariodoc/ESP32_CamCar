#include "config.h"
#include "i2c_manager.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include "wifi_server.h"
#include <ArduinoOTA.h>
#include "Melodies.h"

void initTasks()
{
    xTaskCreatePinnedToCore(servoControlTask, "ServoControl", 1024 * 2, NULL, 1, &servoControlTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(playMelody, "playMelody", STACK_SIZE, NULL, 0, &playMelodyTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(obstacleAvoidanceMode, "obstacleAvoidanceMode", 1024 * 2, NULL, 2, &obstacleAvoidanceModeTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);
}

void setup()
{
    // Forzar explícitamente los 240 MHz desde el arranque
    setCpuFrequencyMhz(240);

#ifdef DEBUG
    Serial.begin(115200);
#endif

    initI2CManager();
    setupCamera();
    setupPeripherals();
    initWiFi();
    initTasks();
}

void loop()
{
    ArduinoOTA.handle();

    static unsigned long lastCleanupTime = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastCleanupTime >= 2000)
    {
        lastCleanupTime = currentMillis;
        cleanupWSClients();
    }

    // 🛑 CONTROL DE SEGURIDAD KEEP ALIVE (Timeout: 1200 ms)
    if (currentMillis - lastCommandTime > 1200 && (joystickX != 0.0f || joystickY != 0.0f))
    {
        joystickX = 0.0f;
        joystickY = 0.0f;
#ifdef DEBUG
        Serial.println("❌ Timeout de comunicación: Frenando por seguridad.");
#endif
    }

    // Detección de Obstáculos
    if (obstacleFound && joystickY > 0.0f)
    {
        toneToPlay(buzzerPin, buzzerChannel, NOTE_G5, 200);
        joystickY = 0.0f; // Bloquea avance hacia adelante
    }

    // Actualiza movimiento proporcional
    processDifferentialDrive(joystickX, joystickY);

    vTaskDelay(pdMS_TO_TICKS(10));
}