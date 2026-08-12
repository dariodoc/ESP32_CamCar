#include "config.h"
#include "i2c_manager.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include "wifi_server.h"
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include "Melodies.h"

void initTasks()
{
    xTaskCreatePinnedToCore(servoControlTask, "ServoControl", 1024 * 2, NULL, 1, &servoControlTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(playMelody, "playMelody", STACK_SIZE, NULL, 1, &playMelodyTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(obstacleAvoidanceMode, "obstacleAvoidanceMode", 1024 * 2, NULL, 2, &obstacleAvoidanceModeTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);
}

void initArduinoOTA()
{
    // ... tu configuración de WiFi actual ...

    // Configuración obligatoria de ArduinoOTA
    ArduinoOTA.onStart([]()
                       { String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem"; });
    ArduinoOTA.begin(); // 👈 Sin esto el ESP32 no escuchará peticiones en el puerto 3232
}

void setup()
{
    // Forzar explícitamente los 240 MHz desde el arranque
    setCpuFrequencyMhz(240);

#ifdef DEBUG
    Serial.begin(115200);
#endif

    initI2CManager();
    setupPeripherals();
    setupCamera();
    initWiFi();
    initTasks();
    initArduinoOTA();
}

void loop()
{
    ArduinoOTA.handle();

    static int lastSpeed = -1;
    static int lastDirection = -1;
    static unsigned long lastCleanupTime = 0;

    unsigned long currentMillis = millis();

    if (currentMillis - lastCleanupTime >= 2000)
    {
        lastCleanupTime = currentMillis;
        cleanupWSClients();
    }

    // 🛑 CONTROL DE SEGURIDAD KEEP ALIVE (Timeout: 1000 ms)
    // Si no se recibe latido ni datos en 1 segundo y el coche no está en STOP:
    if (currentMillis - lastCommandTime > 1200 && targetDirection != STOP)
    {
        targetDirection = STOP;
#ifdef DEBUG
        Serial.println("❌ Timeout de comunicación: Frenando por seguridad.");
#endif
    }

    int currentTargetDir = targetDirection;
    int currentMotorSpeed = motorSpeed;

    if (obstacleFound && (currentTargetDir == FORWARD || currentTargetDir == FORWARDLEFT || currentTargetDir == FORWARDRIGHT))
    {
        toneToPlay(buzzerPin, buzzerChannel, NOTE_G5, 200);
        currentTargetDir = STOP;
        targetDirection = STOP;
    }

    if (currentTargetDir != lastDirection || currentMotorSpeed != lastSpeed)
    {
        moveCar(currentTargetDir);
        lastDirection = currentTargetDir;
        lastSpeed = currentMotorSpeed;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
}