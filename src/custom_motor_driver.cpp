#include "custom_motor_driver.h"
#include "config.h"
#include "i2c_manager.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Wire.h>

static uint8_t fmcPcfShadow = 0xFF; // Expansor Frontal (0x20): Bits 0-3 en 1 (Entradas IR)
static uint8_t bmcPcfShadow = 0xFF; // Expansor Trasero  (0x24): Motores BL, BR, STBY y Reset TFT
static bool currentStandbyState = false;
static bool tftResetHigh = true;
static uint8_t lastWrittenFmc = 0x00; // Forzar primera escritura I2C
static uint8_t lastWrittenBmc = 0x00; // Forzar primera escritura I2C

uint8_t getBmcPcfShadow()
{
    return bmcPcfShadow;
}

void setPcfDisplayResetPin(bool state)
{
    if (lockI2C(50))
    {
        tftResetHigh = state;
        if (state)
            bmcPcfShadow |= (1 << tftResetPcfPin);
        else
            bmcPcfShadow &= ~(1 << tftResetPcfPin);

        Wire.beginTransmission(PCF_BMC_ADDR);
        Wire.write(bmcPcfShadow);
        Wire.endTransmission();
        lastWrittenBmc = bmcPcfShadow;

        unlockI2C();
    }
}

bool setStandbyPin(bool enable)
{
    if (currentStandbyState == enable)
        return true;

    if (lockI2C(20))
    {
        currentStandbyState = enable;

        if (enable)
            bmcPcfShadow |= (1 << STBYpin);
        else
            bmcPcfShadow &= ~(1 << STBYpin);

        unlockI2C();
        return true;
    }
    return false;
}

Motor::Motor(int In1pin, int In2pin, int PWMpin, int offset, uint8_t pcfAddress, Adafruit_PWMServoDriver *pcaController)
{
    In1 = In1pin;
    In2 = In2pin;
    PWM = PWMpin;
    Offset = offset;
    pcfAddr = pcfAddress;
    pca = pcaController;
    lastStateIn1 = -1;
    lastStateIn2 = -1;
    lastSpeed = -1;
}

void Motor::invalidateCache()
{
    lastStateIn1 = -1;
    lastStateIn2 = -1;
    lastSpeed = -1;
}

void forceSyncMotorsI2C()
{
    Wire.beginTransmission(PCF_FMC_ADDR);
    Wire.write(fmcPcfShadow);
    Wire.endTransmission();

    Wire.beginTransmission(PCF_BMC_ADDR);
    Wire.write(bmcPcfShadow);
    Wire.endTransmission();

    lastWrittenFmc = fmcPcfShadow;
    lastWrittenBmc = bmcPcfShadow;
}

bool syncMotorsI2C()
{
    bool fmcDirty = (fmcPcfShadow != lastWrittenFmc);
    bool bmcDirty = (bmcPcfShadow != lastWrittenBmc);
    if (!fmcDirty && !bmcDirty)
        return true;

    if (lockI2C(20))
    {
        bool ok = true;

        // 1. Enviar estado de motores delanteros (FMC) solo si cambió y confirmar ACK
        if (fmcDirty)
        {
            Wire.beginTransmission(PCF_FMC_ADDR);
            Wire.write(fmcPcfShadow);
            if (Wire.endTransmission() == 0)
                lastWrittenFmc = fmcPcfShadow;
            else
                ok = false;
        }

        // 2. Enviar estado de motores traseros y STBY (BMC) solo si cambió y confirmar ACK
        if (bmcDirty)
        {
            Wire.beginTransmission(PCF_BMC_ADDR);
            Wire.write(bmcPcfShadow);
            if (Wire.endTransmission() == 0)
                lastWrittenBmc = bmcPcfShadow;
            else
                ok = false;
        }

        unlockI2C();
        return ok;
    }
    return false;
}

bool Motor::setMotorState(int stateIn1, int stateIn2, int speed)
{
    // Bypass: Si el estado no ha cambiado, no enviamos nada por I2C
    if (stateIn1 == lastStateIn1 && stateIn2 == lastStateIn2 && speed == lastSpeed)
    {
        return true;
    }

    if (lockI2C(20))
    {
        if (pcfAddr == PCF_FMC_ADDR)
        {
            if (stateIn1 == HIGH)
                fmcPcfShadow |= (1 << In1);
            else
                fmcPcfShadow &= ~(1 << In1);

            if (stateIn2 == HIGH)
                fmcPcfShadow |= (1 << In2);
            else
                fmcPcfShadow &= ~(1 << In2);

            // Máscara atómica de entradas: Forzar pines 0..3 en HIGH (entradas IR)
            fmcPcfShadow |= 0x0F;
        }
        else
        {
            if (stateIn1 == HIGH)
                bmcPcfShadow |= (1 << In1);
            else
                bmcPcfShadow &= ~(1 << In1);

            if (stateIn2 == HIGH)
                bmcPcfShadow |= (1 << In2);
            else
                bmcPcfShadow &= ~(1 << In2);

            // Preservar estado real del pin de reset del TFT (Pin 5 del BMC)
            if (tftResetHigh)
                bmcPcfShadow |= (1 << tftResetPcfPin);
            else
                bmcPcfShadow &= ~(1 << tftResetPcfPin);
        }

        uint16_t duty = (speed >= 4095) ? 4095 : speed;
        // Desfase perfecto basado en el pin del PWM (0, 1, 2 o 3)
        // Separa los 4 motores por 1024 ticks exactos para evitar superposicion de picos
        uint16_t startTick = (PWM * 1024) % 4096;

        uint8_t err = 0;
        if (duty >= 4095) {
            err = pca->setPWM(PWM, 4096, 0); // FULLY ON
        } else if (duty == 0) {
            err = pca->setPWM(PWM, 0, 4096); // FULLY OFF
        } else {
            err = pca->setPWM(PWM, startTick, (startTick + duty) % 4096);
        }

        unlockI2C();

        if (err == 0)
        {
            lastStateIn1 = stateIn1;
            lastStateIn2 = stateIn2;
            lastSpeed = speed;
            return true;
        }
        lastSpeed = -1;
        return false;
    }
    return false;
}

bool Motor::fwd(int speed) { return setMotorState(HIGH, LOW, speed); }
bool Motor::rev(int speed) { return setMotorState(LOW, HIGH, speed); }
// En TB6612FNG, PWM = 0 produce Short Brake sin generar pico de corriente al frenar
bool Motor::brake() { return setMotorState(HIGH, HIGH, 0); }

bool Motor::drive(int speed)
{
    speed = speed * Offset;
    if (speed >= 0)
        return fwd(speed);
    else
        return rev(-speed);
}

void Motor::drive(int speed, int duration)
{
    drive(speed);
    vTaskDelay(pdMS_TO_TICKS(duration));
}