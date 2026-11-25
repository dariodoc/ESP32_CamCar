#include "config.h"
#include "peripherals.h"
#include "i2c_manager.h"
#include "motor_control.h"
#include "custom_motor_driver.h"
#include <Adafruit_PWMServoDriver.h>
#include "Melodies.h"
#include "display_control.h"
#include <Wire.h>
#include "driver/rmt.h"

Adafruit_PWMServoDriver pca9685(PCA9685_ADDR, Wire);

volatile bool enableLaser = false;
volatile bool melodyOn = false;
volatile bool enableObstacleAvoidance = false;
volatile bool enableIROnlyMode = false;
volatile bool obstacleFound = false;
volatile bool enableSonicTelemetry = false;
volatile float latestSonicDistanceCM = -1.0f;

static int lastLeftLed = -1;
static int lastRightLed = -1;

// Restaurar salidas PCA9685 manteniendo la posesión previa de i2cMutex
void restorePeripheralsAfterI2CResetLocked()
{
    if (enableLaser) pca9685.setPWM(laserPin, 0, 4096);
    else pca9685.setPWM(laserPin, 4096, 0);

    if (lastLeftLed == 1) pca9685.setPWM(leftRearLedPin, 0, 4096);
    else pca9685.setPWM(leftRearLedPin, 4096, 0);

    if (lastRightLed == 1) pca9685.setPWM(rightRearLedPin, 0, 4096);
    else pca9685.setPWM(rightRearLedPin, 4096, 0);
}

void leftRearLed(int state)
{
    int normState = state ? 1 : 0;
    if (normState == lastLeftLed)
        return;

    if (lockI2C(20))
    {
        lastLeftLed = normState;
        if (normState) pca9685.setPWM(leftRearLedPin, 0, 4096); // GND continuo -> Enciende
        else pca9685.setPWM(leftRearLedPin, 4096, 0);           // 3.3V continuo -> Apaga
        unlockI2C();
    }
}

void rightRearLed(int state)
{
    int normState = state ? 1 : 0;
    if (normState == lastRightLed)
        return;

    if (lockI2C(20))
    {
        lastRightLed = normState;
        if (normState) pca9685.setPWM(rightRearLedPin, 0, 4096); // GND continuo -> Enciende
        else pca9685.setPWM(rightRearLedPin, 4096, 0);           // 3.3V continuo -> Apaga
        unlockI2C();
    }
}

static int currentPan = panCenter;
static int currentTilt = tiltCenter;
static volatile int targetPan = panCenter;
static volatile int targetTilt = tiltCenter;
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
    int panIdleTime = 0;
    int tiltIdleTime = 0;
    bool panAttached = true;
    bool tiltAttached = true;

    while (true)
    {
        int tPan = targetPan;
        int tTilt = targetTilt;
        bool panMoved = false;
        bool tiltMoved = false;

        if (currentPan < tPan) {
            currentPan += 1;
            panMoved = true;
        }
        else if (currentPan > tPan) {
            currentPan -= 1;
            panMoved = true;
        }

        if (currentTilt < tTilt) {
            currentTilt += 1;
            tiltMoved = true;
        }
        else if (currentTilt > tTilt) {
            currentTilt -= 1;
            tiltMoved = true;
        }

        if (panMoved) {
            writeServoPCA(panPin, currentPan, 0); // Empieza en tick 0 (0ms)
            panIdleTime = 0;
            panAttached = true;
        } else if (panAttached) {
            panIdleTime += 20;
            if (panIdleTime >= 500) {
                if (lockI2C(20)) {
                    pca9685.setPWM(panPin, 0, 4096);
                    unlockI2C();
                    panAttached = false;
                }
            }
        }

        if (tiltMoved) {
            writeServoPCA(tiltPin, currentTilt, 2048); // Empieza en tick 2048 (10ms después)
            tiltIdleTime = 0;
            tiltAttached = true;
        } else if (tiltAttached) {
            tiltIdleTime += 20;
            if (tiltIdleTime >= 500) {
                if (lockI2C(20)) {
                    pca9685.setPWM(tiltPin, 0, 4096);
                    unlockI2C();
                    tiltAttached = false;
                }
            }
        }

        // 20ms delay (50Hz) alinea con el refresh real del servo
        // Movimiento de 1 grado / 20ms = 50 grados por segundo
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void setPanAngle(int angle)
{
    int newAngle = constrain(angle, 30, 150);
    if (newAngle != targetPan) {
        targetPan = newAngle;
    }
}

void setTiltAngle(int angle)
{
    // Límite físico estricto: evita que la cámara choque con el chasis (Stall)
    int safeAngle = constrain(angle, 50, 130);
    int newAngle = 180 - safeAngle;
    if (newAngle != targetTilt) {
        targetTilt = newAngle;
    }
}

void setupPeripherals()
{
    pinMode(builtinLedPin, OUTPUT);
    digitalWrite(builtinLedPin, HIGH);
    ledcDetachPin(buzzerPin);

    Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
    Wire.setClock(100000); // 100kHz para robustez frente a EMI de motores
    Wire.setTimeOut(50);   // Evitar deadlock de I2C sin cortar transacciones normales
    vTaskDelay(pdMS_TO_TICKS(100));

    pca9685.begin();
    pca9685.setPWMFreq(50);

    // Liberación inicial del bus I2C antes de lanzar tareas de periféricos
    if (lockI2C(50))
    {
        Wire.beginTransmission(PCF_FMC_ADDR);
        Wire.write(0xFF);
        Wire.endTransmission();

        Wire.beginTransmission(PCF_BMC_ADDR);
        Wire.write(getBmcPcfShadow()); // Respeta el estado del Pin 5
        Wire.endTransmission();

        pca9685.setPWM(laserPin, 4096, 0); // Láser apagado en arranque sin pitido
        unlockI2C();
    }

    centerServos();
    ledIndicator(3, 100);
}

void startPeripheralTasks()
{
    if (servoTaskHandle == NULL)
    {
        xTaskCreatePinnedToCore(servoSlewTask, "ServoTask", 2048, NULL, 1, &servoTaskHandle, 1);
    }
    if (playMelodyTaskHandle == NULL)
    {
        xTaskCreatePinnedToCore(playMelody, "playMelody", 2048, NULL, 1, &playMelodyTaskHandle, 1);
    }
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
    bool updated = false;
    if (lockI2C(20))
    {
        if (state) pca9685.setPWM(laserPin, 0, 4096); // GND continuo -> Enciende
        else pca9685.setPWM(laserPin, 4096, 0);       // 3.3V continuo -> Apaga
        unlockI2C();
        updated = true;
    }

    // Pitido fuera del mutex I2C para no bloquear motores ni servos
    if (updated && !melodyOn)
    {
        toneToPlay(buzzerPin, buzzerChannel, state ? 2000 : 500, state ? 60 : 80);
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
        ulTaskNotifyTake(pdTRUE, 0); // Limpiar notificaciones pendientes tras apagar
    }
}

void centerServos()
{
    setPanAngle(panCenter);
    setTiltAngle(tiltCenter);
}

// RMT configuración para el pin ECHO
#define RMT_RX_CHANNEL RMT_CHANNEL_0

void setupUltrasonic()
{
    pinMode(trigPin, OUTPUT);
    digitalWrite(trigPin, LOW);

    // Configurar RMT (Remote Control) para leer el ECHO en hardware
    rmt_config_t rmt_rx = {};
    rmt_rx.channel = RMT_RX_CHANNEL;
    rmt_rx.gpio_num = (gpio_num_t)echoPin;
    rmt_rx.clk_div = 80; // Divisor 80: reloj de 80MHz -> 1 tick = 1 microsegundo
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100; // Filtrar ruidos menores a 100us
    rmt_rx.rx_config.idle_threshold = 30000;    // Timeout de 30ms (30,000 us)

    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);
}

float getDistanceCM()
{
    RingbufHandle_t rb = NULL;
    rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb);
    if (rb == NULL)
        return -1.0;

    // Drenar todas las lecturas viejas del Ring Buffer
    size_t rx_size = 0;
    rmt_item32_t *dummy = NULL;
    while ((dummy = (rmt_item32_t *)xRingbufferReceive(rb, &rx_size, 0)) != NULL)
    {
        vRingbufferReturnItem(rb, (void *)dummy);
    }

    // Iniciar captura en hardware RMT
    rmt_rx_start(RMT_RX_CHANNEL, true);

    // Mandar el pulso de 10us por TRIG
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Esperar eco en hardware (Max 40ms cediendo CPU)
    rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rb, &rx_size, pdMS_TO_TICKS(40));

    if (item)
    {
        // Copiar campos ANTES de devolver el item al ringbuffer (evita Use-After-Free)
        uint32_t duration = item[0].duration0;
        uint32_t level = item[0].level0;
        vRingbufferReturnItem(rb, (void *)item);
        rmt_rx_stop(RMT_RX_CHANNEL);

        if (level == 1 && duration > 100 && duration < 30000)
        {
            return (duration * 0.0343f) / 2.0f;
        }
    }
    else
    {
        rmt_rx_stop(RMT_RX_CHANNEL);
    }

    return -1.0f;
}

// Tarea unificada: Infrarrojos (PCF8574) + Ultrasónico (Trig 33 / Echo 32)
void obstacleAvoidanceMode(void *parameters)
{
    setupUltrasonic();
    TickType_t lastWakeTime = xTaskGetTickCount();

    bool wasAvoiding = false;
    static float usHistory[3] = {100.0f, 100.0f, 100.0f};
    static int obstacleCounter = 0;
    static int autoState = 0; // 0=Avanzar, 1=Frenando, 2=Reversa, 3=Girando, 4=Strafing
    static int stateTimer = 0;
    static int turnDirection = 90;
    static int strafeDirection = 90;
    static bool nextManeuverStrafe = false;
    static int stuckCounter = 0;
    static int forwardCounter = 0;

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

            // Reiniciar estado autónomo y filtros antes de dormir
            autoState = 0;
            stateTimer = 0;
            nextManeuverStrafe = false;
            stuckCounter = 0;
            forwardCounter = 0;
            obstacleCounter = 0;
            usHistory[0] = usHistory[1] = usHistory[2] = 100.0f;

            if (enableSonicTelemetry)
            {
                latestSonicDistanceCM = getDistanceCM();
                vTaskDelay(pdMS_TO_TICKS(120));
                lastWakeTime = xTaskGetTickCount();
                continue;
            }

            latestSonicDistanceCM = -1.0f;
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            lastWakeTime = xTaskGetTickCount();
            continue;
        }
        else
        {
            wasAvoiding = true;
        }

        bool ir1 = false, ir2 = false, ir3 = false, ir4 = false;

        // 1. Lectura del PCF8574 (0x20) - Infrarrojos
        if (lockI2C(20))
        {
            Wire.requestFrom(PCF_FMC_ADDR, 1);
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

        // 2. Lectura del Ultrasónico (solo si NO estamos en modo solo-IR)
        float distance = -1.0f;
        bool usObstacle = false;

        if (!enableIROnlyMode)
        {
            float rawDist = getDistanceCM();
            if (rawDist < 0) rawDist = 100.0f; // timeout

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
            distance = sorted[1];
            if (distance >= 100.0f) distance = -1.0f;
            latestSonicDistanceCM = distance;
            usObstacle = (distance > 0.0f && distance <= 22.0f);
        }
        else
        {
            latestSonicDistanceCM = -1.0f;
        }

        // Guardia post-eco: si el usuario desactivó el modo autónomo durante getDistanceCM(), salir ya
        if (!enableObstacleAvoidance && !enableIROnlyMode)
        {
            continue;
        }

        // 3. Histéresis de pantalla
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
                updateDisplayState(DISPLAY_CLEAR_ALERT, "");
            }
        }

        // 4. MÁQUINA DE ESTADOS - PILOTO AUTOMÁTICO (Estilo Tanque / Mecanum)
        int speed = 1200;
        if (!enableIROnlyMode && distance > 18.0f && distance < 40.0f) {
            speed = 700; // Aproximación lenta
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
                    nextManeuverStrafe = false;
                    turnDirection = (esp_random() % 2 == 0) ? 90 : -90;
                } else if (stuckCounter >= 2 && !(ir2 && ir3)) {
                    nextManeuverStrafe = true;
                    if (ir3) strafeDirection = -90;
                    else if (ir2) strafeDirection = 90;
                    else strafeDirection = (esp_random() % 2 == 0) ? 90 : -90;
                } else {
                    nextManeuverStrafe = false;
                    if (ir3 && !ir2) turnDirection = -90;
                    else if (ir2 && !ir3) turnDirection = 90;
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
                if (nextManeuverStrafe) {
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
            if (ir1 || ir4) {
                brakeAllMotors();
                if (ir1 && !ir4) turnDirection = 90;
                else if (ir4 && !ir1) turnDirection = -90;
                autoState = 3;
                stateTimer = (stuckCounter >= 4) ? 18 : 9;
            } else {
                driveMecanum(180, speed, 0, 0); // 180 = Atrás
                if (stateTimer > 0) stateTimer--;
                else {
                    autoState = 3;
                    stateTimer = (stuckCounter >= 4) ? 18 : 9;
                }
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
            bool sideBlocked = (strafeDirection > 0) ? (ir3 || ir4) : (ir2 || ir1);
            if (sideBlocked) {
                brakeAllMotors();
                autoState = 2;
                stateTimer = 7;
            } else {
                driveMecanum(strafeDirection, speed, 0, 0);
                if (stateTimer > 0) stateTimer--;
                else {
                    autoState = 0; // Volver a avanzar
                }
            }
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(60));
    }
}