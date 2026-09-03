#include "custom_motor_driver.h"
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Wire.h>

extern PCF8574 FMCpcf8574;
extern PCF8574 BMCpcf8574;
extern bool lockI2C(TickType_t timeoutMs = 20);
extern void unlockI2C();

// 🚀 REGISTROS EN SOMBRA: Estado persistente en RAM para ambos PCF8574
static uint8_t fmcPcfShadow = 0xFF; // Frente (0x20): Motores FL, FR y Sensores IR
static uint8_t bmcPcfShadow = 0xFF; // Atrás  (0x24): Motores BL, BR y STBY
static bool currentStandbyState = false;

// Control atómico y eficiente del pin STBY en el PCF8574 Trasero (0x24)
void setStandbyPin(bool enable)
{
    // Solo transmitimos por I2C si el estado del STBY realmente cambia
    if (currentStandbyState == enable)
        return;

    if (lockI2C(20))
    {
        currentStandbyState = enable;

        if (enable)
            bmcPcfShadow |= (1 << STBYpin);
        else
            bmcPcfShadow &= ~(1 << STBYpin);

        Wire.beginTransmission(0x24);
        Wire.write(bmcPcfShadow);
        Wire.endTransmission();

        unlockI2C();
    }
}

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
            // --- EXPANSOR FRONTAL (0x20) ---
            if (stateIn1 == HIGH)
                fmcPcfShadow |= (1 << In1);
            else
                fmcPcfShadow &= ~(1 << In1);

            if (stateIn2 == HIGH)
                fmcPcfShadow |= (1 << In2);
            else
                fmcPcfShadow &= ~(1 << In2);

            // Preservar SIEMPRE en 1 (entradas) los sensores de obstáculos
            fmcPcfShadow |= (1 << obstacleDetectorPin1);
            fmcPcfShadow |= (1 << obstacleDetectorPin2);
            fmcPcfShadow |= (1 << obstacleDetectorPin3);
            fmcPcfShadow |= (1 << obstacleDetectorPin4);

            Wire.beginTransmission(0x20);
            Wire.write(fmcPcfShadow);
            Wire.endTransmission();
        }
        else if (pcf == &BMCpcf8574)
        {
            // --- EXPANSOR TRASERO (0x24) ---
            if (stateIn1 == HIGH)
                bmcPcfShadow |= (1 << In1);
            else
                bmcPcfShadow &= ~(1 << In1);

            if (stateIn2 == HIGH)
                bmcPcfShadow |= (1 << In2);
            else
                bmcPcfShadow &= ~(1 << In2);

            Wire.beginTransmission(0x24);
            Wire.write(bmcPcfShadow);
            Wire.endTransmission();
        }

        // Ajuste de velocidad PWM en el PCA9685
        pca->setPWM(PWM, 0, speed);

        unlockI2C();
    }
}

void Motor::fwd(int speed) { setMotorState(HIGH, LOW, speed); }
void Motor::rev(int speed) { setMotorState(LOW, HIGH, speed); }
void Motor::brake() { setMotorState(HIGH, HIGH, 0); }

void Motor::drive(int speed)
{
    speed = speed * Offset;
    if (speed >= 0)
        fwd(speed);
    else
        rev(-speed);
}

void Motor::drive(int speed, int duration)
{
    drive(speed);
    vTaskDelay(pdMS_TO_TICKS(duration));
}