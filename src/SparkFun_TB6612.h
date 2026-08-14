#ifndef SPARKFUN_TB6612_h
#define SPARKFUN_TB6612_h

#include <Arduino.h>
#include <PCF8574.h> // 👈 Requerido para declarar PCF8574*

#define DEFAULTSPEED 255

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
    void standby();

private:
    int In1, In2, PWM, Offset, Standby;
    PCF8574 *pcf; // 👈 Puntero al expandidor específico de este motor

    void fwd(int speed);
    void rev(int speed);
};

void forward(Motor &motor1, Motor &motor2, int speed);
void forward(Motor &motor1, Motor &motor2);

void back(Motor &motor1, Motor &motor2, int speed);
void back(Motor &motor1, Motor &motor2);

void left(Motor &left, Motor &right, int speed);
void right(Motor &left, Motor &right, int speed);

void brake(Motor &motor1, Motor &motor2);

void forwardleft(Motor &left, Motor &right, int speed);
void forwardright(Motor &left, Motor &right, int speed);

void backleft(Motor &left, Motor &right, int speed);
void backright(Motor &left, Motor &right, int speed);

#endif