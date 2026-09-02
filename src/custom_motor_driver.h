#ifndef CUSTOM_MOTOR_DRIVER_H
#define CUSTOM_MOTOR_DRIVER_H

#include <Arduino.h>
#include "PCF8574.h"
#include "Adafruit_PWMServoDriver.h"

class Motor
{
private:
    int In1;
    int In2;
    int PWM;
    int Offset;
    PCF8574 *pcf;
    Adafruit_PWMServoDriver *pca;

    // 🚀 Declaración requerida para corregir el error de compilación
    void setMotorState(int stateIn1, int stateIn2, int speed);

public:
    Motor(int In1pin, int In2pin, int PWMpin, int offset, PCF8574 *pcfDev, Adafruit_PWMServoDriver *pcaController);
    void fwd(int speed);
    void rev(int speed);
    void brake();
    void drive(int speed);
    void drive(int speed, int duration);
};

#endif