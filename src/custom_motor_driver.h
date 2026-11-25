#ifndef CUSTOM_MOTOR_DRIVER_H
#define CUSTOM_MOTOR_DRIVER_H

#include <Arduino.h>
#include "Adafruit_PWMServoDriver.h"

bool setStandbyPin(bool enable);
void setPcfDisplayResetPin(bool state);
uint8_t getBmcPcfShadow();
bool syncMotorsI2C();
void forceSyncMotorsI2C();

class Motor
{
private:
    int In1;
    int In2;
    int PWM;
    int Offset;
    uint8_t pcfAddr;
    Adafruit_PWMServoDriver *pca;

    // Caché de estado para evitar saturar el bus I2C
    int lastStateIn1;
    int lastStateIn2;
    int lastSpeed;

    bool setMotorState(int stateIn1, int stateIn2, int speed);

public:
    Motor(int In1pin, int In2pin, int PWMpin, int offset, uint8_t pcfAddress, Adafruit_PWMServoDriver *pcaController);
    bool fwd(int speed);
    bool rev(int speed);
    bool brake();
    bool drive(int speed);
    void drive(int speed, int duration);
    void invalidateCache();
};

#endif