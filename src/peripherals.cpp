#include "config.h"
#include "peripherals.h"
#include "i2c_manager.h"
#include "motor_control.h"
#include "custom_motor_driver.h"
#include <PCF8574.h>
#include <Adafruit_PWMServoDriver.h>
#include "Melodies.h"
#include "display_control.h"
#include <Wire.h>

PCF8574 FMCpcf8574(&Wire, 0x20);
PCF8574 BMCpcf8574(&Wire, 0x24);
Adafruit_PWMServoDriver pca9685(0x40, Wire);

volatile bool enableLaser = false;
volatile bool melodyOn = false;
volatile bool enableObstacleAvoidance = false;
volatile bool enableIROnlyMode = false;
volatile bool obstacleFound = false;

// Removed duplicate servo declarations

void leftRearLed(int state)
{
    if (lockI2C(20))
    {
        if (state) pca9685.setPWM(leftRearLedPin, 0, 4096); // GND continuo -> Enciende
        else pca9685.setPWM(leftRearLedPin, 4096, 0);       // 3.3V continuo -> Apaga
        unlockI2C();
    }
}

void rightRearLed(int state)
{
    if (lockI2C(20))
    {
        if (state) pca9685.setPWM(rightRearLedPin, 0, 4096); // GND continuo -> Enciende
        else pca9685.setPWM(rightRearLedPin, 4096, 0);       // 3.3V continuo -> Apaga
        unlockI2C();
    }
}

static int currentPan = panCenter;
static int currentTilt = tiltCenter;
static int targetPan = panCenter;
static int targetTilt = tiltCenter;
TaskHandle_t servoTaskHandle = NULL;

void writeServoPCA(uint8_t channel, int angle, uint16_t startTick = 0)
{
    int constrainedAngle = constrain(angle, 0, 180);
    int uS = map(constrainedAngle, 0, 180, 550, 2650);

    // En 50Hz, 1 ciclo = 20,000 microsegundos = 4096 ticks
    // ticks = uS * 4096 / 20000 = uS * 0.2048
    uint16_t pulseTicks = (uS * 4096) / 20000;
    uint16_t endTick = (startTick + pulseTicks) % 4096;

    if (lockI2C(20))
    {
        pca9685.setPWM(channel, startTick, endTick);
        unlockI2C();
    }
}

void servoSlewTask(void *pvParameters)
{
    while (true)
    {
        bool panMoved = false;
        bool tiltMoved = false;

        if (currentPan < targetPan) { 
            currentPan += 1; 
            panMoved = true; 
        }
        else if (currentPan > targetPan) { 
            currentPan -= 1; 
            panMoved = true; 
        }

        if (currentTilt < targetTilt) { 
            currentTilt += 1; 
            tiltMoved = true; 
        }
        else if (currentTilt > targetTilt) { 
            currentTilt -= 1; 
            tiltMoved = true; 
        }

        static int panIdleTime = 0;
        static int tiltIdleTime = 0;

        if (panMoved) {
            writeServoPCA(panPin, currentPan, 0); // Empieza en tick 0 (0ms)
            panIdleTime = 0;
        } else {
            panIdleTime += 20;
            if (panIdleTime == 500) {
                if (lockI2C(20)) { pca9685.setPWM(panPin, 0, 4096); unlockI2C(); }
            }
            if (panIdleTime > 1000) panIdleTime = 1000;
        }

        if (tiltMoved) {
            writeServoPCA(tiltPin, currentTilt, 2048); // Empieza en tick 2048 (10ms después!)
            tiltIdleTime = 0;
        } else {
            tiltIdleTime += 20;
            if (tiltIdleTime == 500) {
                if (lockI2C(20)) { pca9685.setPWM(tiltPin, 0, 4096); unlockI2C(); }
            }
            if (tiltIdleTime > 1000) tiltIdleTime = 1000;
        }
        
        // 20ms delay (50Hz) alinea con el refresh real del servo
        // Movimiento de 2 grados / 20ms = 100 grados por segundo (Cinemático)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void setPanAngle(int angle)
{
    // Limite simetrico
    int newAngle = constrain(angle, 30, 150);
    // Filtro de "banda muerta" (Deadband). 
    // Ignora pequeños temblores del dedo en la pantalla táctil de la app.
    // Evita el "jittering" constante y su altísimo consumo eléctrico.
    if (abs(newAngle - targetPan) > 2) {
        targetPan = newAngle;
    }
}

void setTiltAngle(int angle)
{
    // Límite físico estricto: evita que la cámara choque con el chasis (Stall)
    // Un motor estancado jala amperaje máximo infinito y tira el voltaje.
    int safeAngle = constrain(angle, 50, 130); 
    int newAngle = 180 - safeAngle;
    if (abs(newAngle - targetTilt) > 2) {
        targetTilt = newAngle;
    }
}



void setupPeripherals()
{
    if (servoTaskHandle == NULL)
    {
        xTaskCreatePinnedToCore(servoSlewTask, "ServoTask", 2048, NULL, 1, &servoTaskHandle, 1);
    }

    pinMode(builtinLedPin, OUTPUT);
    digitalWrite(builtinLedPin, HIGH);
    ledcDetachPin(buzzerPin);

    Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
    vTaskDelay(pdMS_TO_TICKS(100));

    Wire.setClock(100000);
    Wire.setTimeOut(100);

    FMCpcf8574.begin();
    BMCpcf8574.begin();
    pca9685.begin();
    pca9685.setPWMFreq(50);

    // 🚀 LIBERACIÓN OBLIGATORIA DEL PUERTO AL ARRANQUE:
    // Fuerza a nivel físico que los 8 bits (incluyendo los sensores 0-3)
    // inicien como ENTRADAS (1s lógicos) antes de que el motor toque el bus
    // Reemplazar las escrituras directas Wire.write(0xFF) por la sombra sincronizada:
    if (lockI2C(50))
    {
        Wire.beginTransmission(0x20);
        Wire.write(0xFF);
        Wire.endTransmission();

        Wire.beginTransmission(0x24);
        Wire.write(getBmcPcfShadow()); // Respeta el estado del Pin 5
        Wire.endTransmission();

        unlockI2C();
    }

    turnLaserOn(false);
    centerServos();

    ledIndicator(3, 100);
}

void ledIndicator(int state)
{
    digitalWrite(builtinLedPin, state ? HIGH : LOW);
}

void ledIndicator(int blinkTimes, int delayTimeMS)
{
    for (int i = 0; i < blinkTimes; i++)
    {
        digitalWrite(builtinLedPin, HIGH);
        toneToPlay(buzzerPin, buzzerChannel, NOTE_G5, delayTimeMS);
        digitalWrite(builtinLedPin, LOW);

        if (i < blinkTimes - 1)
        {
            vTaskDelay(pdMS_TO_TICKS(delayTimeMS));
        }
    }
}

void turnLaserOn(bool state)
{
    if (lockI2C(20))
    {
        if (state) {
            pca9685.setPWM(laserPin, 0, 4096); // GND continuo -> Enciende
            toneToPlay(buzzerPin, buzzerChannel, 2000, 100); // Beep agudo: Recibió ON
        } else {
            pca9685.setPWM(laserPin, 4096, 0); // 3.3V continuo -> Apaga
            toneToPlay(buzzerPin, buzzerChannel, 500, 200);  // Beep grave: Recibió OFF
        }
        unlockI2C();
    }
}

void playMelody(void *parameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        gameOfThrones(buzzerPin, buzzerChannel);
        melodyOn = false;
        ledcWriteTone(buzzerChannel, 0);
    }
}

void centerServos()
{
    setPanAngle(panCenter);
    setTiltAngle(tiltCenter);
}

void setupUltrasonic()
{
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
}

float getDistanceCM()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Timeout de 30ms (~500 cm max)
    long duration = pulseIn(echoPin, HIGH, 30000); 

    if (duration == 0)
        return -1.0;

    return (duration * 0.0343) / 2.0;
}

// 🚀 Tarea unificada: Infrarrojos (PCF8574) + Ultrasónico (Trig 33 / Echo 32)
void obstacleAvoidanceMode(void *parameters)
{
    setupUltrasonic();
    TickType_t lastWakeTime = xTaskGetTickCount();

    // Estado previo de evasión para saber cuándo mandar el freno de seguridad
    bool wasAvoiding = false;

    for (;;)
    {
        if (!enableObstacleAvoidance && !enableIROnlyMode)
        {
            if (wasAvoiding)
            {
                wasAvoiding = false;
                brakeAllMotors(); // Frena al recuperar el control manual
                updateDisplayState(DISPLAY_CLEAR_ALERT, "");
            }
            if (obstacleFound)
            {
                obstacleFound = false;
                updateDisplayState(DISPLAY_CLEAR_ALERT, "");
            }
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            lastWakeTime = xTaskGetTickCount();
        }
        else
        {
            wasAvoiding = true;
        }

        bool ir1 = false, ir2 = false, ir3 = false, ir4 = false;

        // 1. Lectura del PCF8574 (0x20) - Infrarrojos
        if (lockI2C(20))
        {
            Wire.requestFrom(0x20, 1);
            if (Wire.available())
            {
                uint8_t currentData = Wire.read();
                ir1 = !(currentData & (1 << obstacleDetectorIR1)); // Back-Right
                ir2 = !(currentData & (1 << obstacleDetectorIR2)); // Front-Right
                ir3 = !(currentData & (1 << obstacleDetectorIR3)); // Front-Left
                ir4 = !(currentData & (1 << obstacleDetectorIR4)); // Back-Left
            }
            unlockI2C();
        }

        // 2. Lectura del Ultrasónico (Filtro Mediana en el tiempo)
        static float usHistory[3] = {100.0, 100.0, 100.0};
        
        float rawDist = getDistanceCM();
        if (rawDist < 0) rawDist = 100.0; // timeout
        
        usHistory[0] = usHistory[1];
        usHistory[1] = usHistory[2];
        usHistory[2] = rawDist;
        
        // Ordenar copia para mediana
        float sorted[3] = {usHistory[0], usHistory[1], usHistory[2]};
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2 - i; j++) {
                if (sorted[j] > sorted[j + 1]) {
                    float temp = sorted[j];
                    sorted[j] = sorted[j + 1];
                    sorted[j + 1] = temp;
                }
            }
        }
        float distance = sorted[1];
        if (distance >= 100.0) distance = -1.0;

        
        // Con el filtro de mediana, la lectura es súper estable. Reacción instantánea.
        // Incluimos desde 0.1cm hasta 22.0cm (ignoramos -1.0 que es timeout)
        bool usObstacle = (distance > 0.0 && distance <= 22.0); 

        if (enableIROnlyMode) {
            usObstacle = false; // 🚀 Ignorar ultrasónico por completo en modo 3
        }

        // 3. Histeresis de pantalla
        static int obstacleCounter = 0;
        bool rawObstacle = (ir1 || ir2 || ir3 || ir4 || usObstacle);
        bool previousObstacleState = obstacleFound;
        
        if (rawObstacle) {
            obstacleCounter = 5; 
            obstacleFound = true;
        } else {
            if (obstacleCounter > 0) obstacleCounter--;
            else obstacleFound = false;
        }

        if (obstacleFound != previousObstacleState)
        {
            if (obstacleFound) {
                char debugMsg[32] = "";
                if (usObstacle) snprintf(debugMsg, sizeof(debugMsg), "OBSTACULO: US (%.0fcm)", distance);
                else if (ir2) snprintf(debugMsg, sizeof(debugMsg), "OBSTACULO: IR2 (Der)");
                else if (ir3) snprintf(debugMsg, sizeof(debugMsg), "OBSTACULO: IR3 (Izq)");
                else if (ir1) snprintf(debugMsg, sizeof(debugMsg), "OBSTACULO: IR1 (Atras)");
                else if (ir4) snprintf(debugMsg, sizeof(debugMsg), "OBSTACULO: IR4 (Atras)");
                else snprintf(debugMsg, sizeof(debugMsg), "OBSTACULO DETECTADO");
                
                updateDisplayState(DISPLAY_OBSTACLE_ALERT, debugMsg);
            }
            else {
                updateDisplayState(DISPLAY_CLEAR_ALERT,"");
            }
        }

        // 4. MÁQUINA DE ESTADOS - PILOTO AUTOMÁTICO (Estilo Tanque / Mecanum)
        static int autoState = 0; // 0=Avanzar, 1=Frenando, 2=Reversa, 3=Girando, 4=Strafing
        static int stateTimer = 0;
        static int turnDirection = 90;
        static int strafeDirection = 90;
        static int stuckCounter = 0;
        static int forwardCounter = 0;

        if (enableObstacleAvoidance || enableIROnlyMode)
        {
            // Velocidad dinámica: si ve algo a menos de 40cm, reduce la velocidad para no estrellarse por inercia
            int speed = 600; 
            if (!enableIROnlyMode && distance > 18.0 && distance < 40.0) {
                speed = 400; // Aproximación lenta
            }
            
            if (autoState == 0) // AVANZANDO
            {
                if (usObstacle || ir2 || ir3) 
                {
                    stuckCounter++;
                    forwardCounter = 0;
                    autoState = 1;
                    stateTimer = 2; // Frenar por ~120ms
                    brakeAllMotors();
                    
                    if (stuckCounter >= 4) {
                        turnDirection = (esp_random() % 2 == 0) ? 90 : -90;
                    } else if (stuckCounter >= 2) {
                        if (ir3) strafeDirection = -90; 
                        else if (ir2) strafeDirection = 90; 
                        else strafeDirection = (esp_random() % 2 == 0) ? 90 : -90;
                    } else {
                        if (ir3) turnDirection = -90; 
                        else if (ir2) turnDirection = 90;  
                        else turnDirection = (esp_random() % 2 == 0) ? 90 : -90; 
                    }
                }
                else
                {
                    driveMecanum(0, speed, 0, 0); // 0 = Adelante
                    forwardCounter++;
                    if (forwardCounter > 25) { 
                        stuckCounter = 0;      
                        if (forwardCounter > 100) forwardCounter = 100; 
                    }
                }
            }
            else if (autoState == 1) // FRENANDO
            {
                brakeAllMotors();
                if (stateTimer > 0) stateTimer--;
                else {
                    if (stuckCounter >= 2 && stuckCounter < 4) {
                        autoState = 4; // Strafing
                        stateTimer = 15; 
                    } else {
                        autoState = 2;
                        stateTimer = (stuckCounter >= 4) ? 14 : 7; 
                    }
                }
            }
            else if (autoState == 2) // REVERSA
            {
                driveMecanum(180, speed, 0, 0); // 180 = Atrás
                if (stateTimer > 0) stateTimer--;
                else {
                    autoState = 3;
                    stateTimer = (stuckCounter >= 4) ? 18 : 9; 
                }
            }
            else if (autoState == 3) // GIRANDO
            {
                driveMecanum(0, 0, turnDirection, speed);
                if (stateTimer > 0) stateTimer--;
                else {
                    autoState = 0; // Volver a avanzar
                }
            }
            else if (autoState == 4) // STRAFING LATERAL (MECANUM)
            {
                driveMecanum(strafeDirection, speed, 0, 0);
                if (stateTimer > 0) stateTimer--;
                else {
                    autoState = 0; // Volver a avanzar
                }
            }
        }
        else
        {
            autoState = 0;
            stateTimer = 0;
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(60));
    }
}