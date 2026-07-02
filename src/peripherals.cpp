#include "config.h"
#include "peripherals.h"
#include "motor_control.h" // Para llamar a moveCar(STOP)
#include "PCF8574.h"

// Definición de objetos y variables de periféricos
PCF8574 pcf8574(0x20, 14, 15);
Servo panServo;
Servo tiltServo;

volatile bool enableLight = false;
volatile bool melodyOn = false;
volatile bool enableObstacleAvoidance = false;
volatile bool obstacleFound = false;

TaskHandle_t playMelodyTask;
TaskHandle_t obstacleAvoidanceModeTask;

void setupPeripherals()
{
    pinMode(builtinLedPin, OUTPUT);
    digitalWrite(builtinLedPin, HIGH); // LED OFF

    ledcDetachPin(buzzerPin);

    panServo.attach(panPin);
    tiltServo.attach(tiltPin);
    panServo.write(panCenter);
    tiltServo.write(tiltCenter);

    pcf8574.pinMode(P5, INPUT);  // Sensor de distancia
    pcf8574.pinMode(P6, OUTPUT); // LED trasero izquierdo
    pcf8574.pinMode(P7, OUTPUT); // LED trasero derecho
    pinMode(lightPin, OUTPUT);   // Luces frontales

    if (!pcf8574.begin())
    {
#ifdef DEBUG
        Serial.println("PCF8574 FAILED");
#endif
    }
    // --- NUEVO: Blindaje del Bus I2C ---
    // Como el PCF ya inició el bus con los pines correctos, ahora sí le ponemos el límite
    Wire.setTimeOut(50);
    ledIndicator(3, 250);
}

void ledIndicator(int blinkTimes, int delayTimeMS)
{
    for (int i = 0; i < blinkTimes; i++)
    {
        digitalWrite(builtinLedPin, LOW); // LED ON
        toneToPlay(buzzerPin, buzzerChannel, NOTE_G5, delayTimeMS);
        digitalWrite(builtinLedPin, HIGH); // LED OFF
        vTaskDelay(pdMS_TO_TICKS(delayTimeMS));
    }
}

void ledIndicator(int state)
{
    digitalWrite(builtinLedPin, !state); // LOW enciende el LED
}

void leftBackLed(int state)
{
    pcf8574.digitalWrite(P6, !state); // LOW enciende el LED
}

void rightBackLed(int state)
{
    pcf8574.digitalWrite(P7, !state); // LOW enciende el LED
}

void playMelody(void *parameters)
{
#ifdef DEBUG
    Serial.printf("playMelody() initialized on core: %d\n", xPortGetCoreID());
#endif

    for (;;)
    {
        // La tarea se duerme indefinidamente y NO consume CPU hasta recibir la señal
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        gameOfThrones(buzzerPin, buzzerChannel);
        melodyOn = false;

        // Silenciamos el canal en lugar de destruir la configuración del hardware
        ledcWriteTone(buzzerChannel, 0);
    }
}

void obstacleAvoidanceMode(void *parameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        if (!enableObstacleAvoidance)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        int detect = pcf8574.digitalRead(P5);

        // 1. Simplemente actualizamos la bandera indicando si hay pared
        obstacleFound = (detect == LOW);

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(30));
    }
}