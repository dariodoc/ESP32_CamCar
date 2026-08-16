#ifndef SPARKFUN_TB6612_h
#define SPARKFUN_TB6612_h

#include <Arduino.h>
#include <PCF8574.h> // 👈 Requerido para declarar PCF8574*

#define DEFAULTSPEED 255 // max speed for analogWrite

class Motor
{
public:
    // Constructor original
    Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin);

    // 🚀 NUEVO CONSTRUCTOR: Recibe el puntero al PCF8574
    Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin, PCF8574 *pcfDev);

    void drive(int speed);
    void drive(int speed, int duration);
    void brake();

private:
    int In1, In2, PWM, Offset, Standby;
    PCF8574 *pcf; // 👈 Puntero al expandidor específico de este motor

    void fwd(int speed);
    void rev(int speed);
};

#endif