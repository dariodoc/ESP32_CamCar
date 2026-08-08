#include "config.h"
#include "peripherals.h"
#include "motor_control.h"
#include "PCF8574.h"
#include "ESP32Servo.h"
#include "Melodies.h"

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

void leftBackLed(int state)
{
    if (lockI2C(20))
    {
        peripheralspcf8574.digitalWrite(P7, !state);
        unlockI2C();
    }
}

void rightBackLed(int state)
{
    if (lockI2C(20))
    {
        peripheralspcf8574.digitalWrite(P6, !state);
        unlockI2C();
    }
}

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
    digitalWrite(lightPin, LOW); 
    ledcDetachPin(buzzerPin);

    // 1. INICIALIZAR EL BUS I2C PRIMERO QUE NADA
    Wire.begin(14, 15);
    Wire.setClock(400000); 
    vTaskDelay(pdMS_TO_TICKS(100)); // Breve pausa para estabilizar las líneas SDA/SCL

    // 2. AHORA SÍ INICIALIZAR LOS EXPANDIDORES PCF8574
    motorcontrolpcf8574.begin();
    peripheralspcf8574.begin();

    // 3. CONFIGURAR LOS PINES DEL PCF8574
    motorcontrolpcf8574.pinMode(P3, OUTPUT);
    motorcontrolpcf8574.pinMode(P4, OUTPUT);
    motorcontrolpcf8574.pinMode(P2, OUTPUT);
    motorcontrolpcf8574.pinMode(P1, OUTPUT);
    motorcontrolpcf8574.pinMode(P0, OUTPUT);

    peripheralspcf8574.pinMode(P5, INPUT);
    peripheralspcf8574.pinMode(P7, OUTPUT);
    peripheralspcf8574.pinMode(P6, OUTPUT);

    Wire.setTimeOut(50);

    // 4. ATCHAR SERVOS Y POSICIÓN INICIAL
    panServo.attach(panPin);
    tiltServo.attach(tiltPin);
    panServo.write(panCenter);
    tiltServo.write(tiltCenter);

    ledIndicator(3, 100);
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
// peripherals.cpp
void servoControlTask(void *parameters)
{
    static int currentPan = panCenter;
    static int currentTilt = tiltCenter;
    const int maxStep = 2;

    for (;;)
    {
        if (currentPan == targetPan && currentTilt == targetTilt)
        {
            // Si los servos ya llegaron a su destino, dormir 100ms para liberar CPU
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (currentPan != targetPan)
        {
            int diff = targetPan - currentPan;
            currentPan += (abs(diff) <= maxStep) ? diff : ((diff > 0) ? maxStep : -maxStep);
            panServo.write(currentPan);
        }

        if (currentTilt != targetTilt)
        {
            int diff = targetTilt - currentTilt;
            currentTilt += (abs(diff) <= maxStep) ? diff : ((diff > 0) ? maxStep : -maxStep);
            tiltServo.write(currentTilt);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void obstacleAvoidanceMode(void *parameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    int lastDetect = -1;

    for (;;)
    {
        if (!enableObstacleAvoidance)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            lastDetect = -1;
        }

        int detect = HIGH;

        // === ZONA PROTEGIDA I2C ===
        if (lockI2C(20))
        { // Espera hasta 20ms por el bus
            detect = peripheralspcf8574.digitalRead(P5);
            unlockI2C(); // Siempre liberar la llave inmediatamente
        }
        else
        {
            Serial.println("⚠️ Bus I2C ocupado, se omitió lectura del sensor");
        }
        // ==========================

        if (detect != lastDetect)
        {
            obstacleFound = (detect == LOW);
            lastDetect = detect;
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50));
    }
}