#ifndef CUSTOM_MOTOR_DRIVER_H
#define CUSTOM_MOTOR_DRIVER_H

#include <Arduino.h>
#include <PCF8574.h>
#include <Adafruit_PWMServoDriver.h>

class Motor
{
public:
    
    // Constructor con soporte para expandidor PCF8574 y controlador PCA9685 por I2C
    Motor(int In1pin, int In2pin, int PWMpin, int offset, PCF8574 *pcfDev, Adafruit_PWMServoDriver *pcaController);

    void drive(int speed);
    void drive(int speed, int duration);
    void brake();

private:
    int In1, In2, PWM, Offset, Standby;
    PCF8574 *pcf;
    Adafruit_PWMServoDriver *pca;

    void fwd(int speed);
    void rev(int speed);
};

#endif // CUSTOM_MOTOR_DRIVER_H