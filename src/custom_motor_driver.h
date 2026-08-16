#ifndef CUSTOM_MOTOR_DRIVER_H
#define CUSTOM_MOTOR_DRIVER_H

#include <Arduino.h>
#include <PCF8574.h>

#define DEFAULT_MOTOR_SPEED 255

class Motor
{
public:
    // Constructor estándar (pines GPIO directos del microcontrolador)
    Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin);

    // Constructor con soporte para expandidor PCF8574 por I2C
    Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin, PCF8574 *pcfDev);

    void drive(int speed);
    void drive(int speed, int duration);
    void brake();

private:
    int In1, In2, PWM, Offset, Standby;
    PCF8574 *pcf;

    void fwd(int speed);
    void rev(int speed);
};

#endif // CUSTOM_MOTOR_DRIVER_H