#include "custom_motor_driver.h"
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Wire.h>

extern PCF8574 FMCpcf8574;
extern PCF8574 BMCpcf8574;
extern bool lockI2C(TickType_t timeoutMs = 20);
extern void unlockI2C();

static uint8_t fmcPcfShadow = 0xFF; // Expansor Frontal (0x20): Bits 0-3 en 1 (Entradas IR)
static uint8_t bmcPcfShadow = 0xFF; // Expansor Trasero  (0x24): Motores BL, BR y STBY
static bool currentStandbyState = false;

void setStandbyPin(bool enable)
{
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
            // Actualización de dirección para FL y FR (Pines 4, 5, 6, 7)
            if (stateIn1 == HIGH)
                fmcPcfShadow |= (1 << In1);
            else
                fmcPcfShadow &= ~(1 << In1);

            if (stateIn2 == HIGH)
                fmcPcfShadow |= (1 << In2);
            else
                fmcPcfShadow &= ~(1 << In2);

            // 🚀 MÁSCARA ATÓMICA DE ENTRADAS: Forzar los pines 0, 1, 2 y 3 siempre a 1 (HIGH)
            fmcPcfShadow |= 0x0F;

            Wire.beginTransmission(0x20);
            Wire.write(fmcPcfShadow);
            Wire.endTransmission();
        }
        else if (pcf == &BMCpcf8574)
        {
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