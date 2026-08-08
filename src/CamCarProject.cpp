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
    xTaskCreatePinnedToCore(playMelody, "playMelody", STACK_SIZE, NULL, 1, &playMelodyTask, CONFIG_ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(obstacleAvoidanceMode, "obstacleAvoidanceMode", 1024 * 2, NULL, 2, &obstacleAvoidanceModeTask, CONFIG_ARDUINO_RUNNING_CORE);
}

void setup()
{
#ifdef DEBUG
    Serial.begin(115200);
#endif

    SPIFFS.begin(true);

    initI2CManager();
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

    if (currentMillis - lastCleanupTime >= 2000)
    {
        lastCleanupTime = currentMillis;
        cleanupWSClients();
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

    vTaskDelay(pdMS_TO_TICKS(10));
}