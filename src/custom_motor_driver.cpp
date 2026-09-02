#include "custom_motor_driver.h"
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Wire.h>

extern PCF8574 FMCpcf8574;
extern bool lockI2C(TickType_t timeoutMs = 20);
extern void unlockI2C();

// Estado en sombra para preservar salidas y entradas sin depender del Read-Modify-Write de la librería
static uint8_t pcfOutputState = 0xFF; 

Motor::Motor(int In1pin, int In2pin, int PWMpin, int offset, PCF8574 *pcfDev, Adafruit_PWMServoDriver *pcaController)
{
    In1 = In1pin;
    In2 = In2pin;
    PWM = PWMpin;
    Offset = offset;
    pcf = pcfDev;
    pca = pcaController;
}

void Motor::setMotorState(int stateIn1, int stateIn2, int speed)
{
    if (lockI2C(20))
    {
        if (pcf == &FMCpcf8574)
        {
            // 1. Modificar bits de dirección de este motor
            if (stateIn1 == HIGH) pcfOutputState |= (1 << In1);
            else                  pcfOutputState &= ~(1 << In1);

            if (stateIn2 == HIGH) pcfOutputState |= (1 << In2);
            else                  pcfOutputState &= ~(1 << In2);

            // 2. FORZAR SIEMPRE EN 1 LÓGICO LOS SENSORES EN EL EXPANSOR 0x20
            pcfOutputState |= (1 << obstacleDetectorPin1);
            pcfOutputState |= (1 << obstacleDetectorPin2);
            pcfOutputState |= (1 << obstacleDetectorPin3);
            pcfOutputState |= (1 << obstacleDetectorPin4);

            // 3. Transmisión nativa I2C al puerto 0x20
            Wire.beginTransmission(0x20);
            Wire.write(pcfOutputState);
            Wire.endTransmission();
        }
        else
        {
            // Para el expansor de atrás (BMCpcf8574 / 0x24)
            pcf->digitalWrite(In1, stateIn1);
            pcf->digitalWrite(In2, stateIn2);
        }

        // 4. Ajuste de PWM en PCA9685 dentro del mismo lock
        pca->setPWM(PWM, 0, speed);

        unlockI2C();
    }
}

void Motor::fwd(int speed)  { setMotorState(HIGH, LOW, speed); }
void Motor::rev(int speed)  { setMotorState(LOW, HIGH, speed); }
void Motor::brake()        { setMotorState(HIGH, HIGH, 0); }

void Motor::drive(int speed)
{
    speed = speed * Offset;
    if (speed >= 0) fwd(speed);
    else            rev(-speed);
}

void Motor::drive(int speed, int duration)
{
    drive(speed);
    vTaskDelay(pdMS_TO_TICKS(duration));
}