#include "config.h"
#include "peripherals.h"
#include "motor_control.h"
#include "PCF8574.h"

// Definición de objetos y variables de periféricos
PCF8574 motorcontrolpcf8574(&Wire, 0x20);
PCF8574 peripheralspcf8574(&Wire, 0x24);

Servo panServo;
Servo tiltServo;

volatile bool enableLight = false;
volatile bool melodyOn = false;
volatile bool enableObstacleAvoidance = false;
volatile bool obstacleFound = false;

// Variables de posición objetivo (actualizadas por eventos WS)
volatile int targetPan = 75;
volatile int targetTilt = 90;

TaskHandle_t playMelodyTask = NULL;
TaskHandle_t obstacleAvoidanceModeTask = NULL;
TaskHandle_t servoControlTaskHandle = NULL;

void scanI2C()
{
    byte error, address;
    int nDevices = 0;

    Serial.println("Escaneando bus I2C...");

    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("Dispositivo I2C encontrado en direccion 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(" !");
            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Error desconocido en direccion 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No se encontraron dispositivos I2C\n");
    else
        Serial.println("Escaneo completado.\n");
}

void setupPeripherals()
{
    pinMode(builtinLedPin, OUTPUT);
    digitalWrite(builtinLedPin, HIGH); // LED OFF
    pinMode(lightPin, OUTPUT);
    digitalWrite(lightPin, LOW); // Nos aseguramos de que inicie apagado
    ledcDetachPin(buzzerPin);

    motorcontrolpcf8574.pinMode(P3, OUTPUT);
    motorcontrolpcf8574.pinMode(P4, OUTPUT);
    // motorcontrolpcf8574.pinMode(P6, OUTPUT);
    motorcontrolpcf8574.pinMode(P2, OUTPUT);
    motorcontrolpcf8574.pinMode(P1, OUTPUT);
    motorcontrolpcf8574.pinMode(P0, OUTPUT);
    // motorcontrolpcf8574.pinMode(P7, OUTPUT);

    peripheralspcf8574.pinMode(P5, INPUT);
    peripheralspcf8574.pinMode(P7, OUTPUT);
    peripheralspcf8574.pinMode(P6, OUTPUT);

    Wire.begin(14, 15);
    vTaskDelay(pdMS_TO_TICKS(500)); // Esperamos un poco para que el bus I2C se estabilice

    scanI2C();

    // Diagnóstico para el primer PCF (0x20)
    if (motorcontrolpcf8574.begin())
    {
        //  Serial.println("PCF8574 (0x20) inicializado correctamente.");
    }
    else
    {
        //   Serial.println("ERROR: No se pudo inicializar PCF8574 (0x20).");
    }

    // Diagnóstico para el segundo PCF (0x24)
    if (peripheralspcf8574.begin())
    {

        //  Serial.println("PCF8574 (0x24) inicializado correctamente.");
    }
    else
    {
        // Serial.println("ERROR: No se pudo inicializar PCF8574 (0x24).");
    }

    // --- NUEVO: Blindaje del Bus I2C ---
    // Como el PCF ya inició el bus con los pines correctos, ahora sí le ponemos el límite
    Wire.setTimeOut(50);

    panServo.attach(panPin);
    tiltServo.attach(tiltPin);
    panServo.write(panCenter);
    tiltServo.write(tiltCenter);

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
    peripheralspcf8574.digitalWrite(P7, !state); // LOW enciende el LED
}

void rightBackLed(int state)
{
    peripheralspcf8574.digitalWrite(P6, !state); // LOW enciende el LED
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

// --- NUEVO: Tarea dedicada para mover los servos de forma síncrona ---
void servoControlTask(void *parameters)
{
    // Inicializamos las posiciones actuales en el centro al arrancar
    static int currentPan = 75;
    static int currentTilt = 90;

    // Configura cuántos grados máximo se puede mover el servo por ciclo (menor número = más suave y menos consumo)
    const int maxStep = 2;

    for (;;)
    {
        // --- Suavizado de PASO para PAN ---
        if (currentPan != targetPan)
        {
            int diff = targetPan - currentPan;
            if (abs(diff) <= maxStep)
            {
                currentPan = targetPan; // Si está muy cerca, llega al objetivo
            }
            else
            {
                currentPan += (diff > 0) ? maxStep : -maxStep; // Se mueve a pasos sutiles
            }
            panServo.write(currentPan);
        }

        // --- Suavizado de PASO para TILT ---
        if (currentTilt != targetTilt)
        {
            int diff = targetTilt - currentTilt;
            if (abs(diff) <= maxStep)
            {
                currentTilt = targetTilt;
            }
            else
            {
                currentTilt += (diff > 0) ? maxStep : -maxStep;
            }
            tiltServo.write(currentTilt);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Bajamos a 20ms para compensar la suavidad de los pasos
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

        int detect = peripheralspcf8574.digitalRead(P5);

        // 1. Simplemente actualizamos la bandera indicando si hay pared
        obstacleFound = (detect == LOW);

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(30));
    }
}