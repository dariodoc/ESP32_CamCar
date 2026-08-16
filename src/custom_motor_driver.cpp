#include "custom_motor_driver.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Funciones globales para control de concurrencia I2C
extern bool lockI2C(TickType_t timeoutMs = 20);
extern void unlockI2C();

// Constructor para GPIO directos
Motor::Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin)
{
    In1 = In1pin;
    In2 = In2pin;
    PWM = PWMpin;
    Offset = offset;
    Standby = STBYpin;
    pcf = nullptr;

    pinMode(PWM, OUTPUT);
}

// Constructor con PCF8574
Motor::Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin, PCF8574 *pcfDev)
{
    In1 = In1pin;
    In2 = In2pin;
    PWM = PWMpin;
    Offset = offset;
    Standby = STBYpin;
    pcf = pcfDev;

    pinMode(PWM, OUTPUT);
}

void Motor::fwd(int speed)
{
    if (pcf != nullptr)
    {
        if (lockI2C(20))
        {
            pcf->digitalWrite(Standby, HIGH);
            pcf->digitalWrite(In1, HIGH);
            pcf->digitalWrite(In2, LOW);
            unlockI2C();
        }
    }
    else
    {
        digitalWrite(Standby, HIGH);
        digitalWrite(In1, HIGH);
        digitalWrite(In2, LOW);
    }

    analogWrite(PWM, speed);
}

void Motor::rev(int speed)
{
    if (pcf != nullptr)
    {
        if (lockI2C(20))
        {
            pcf->digitalWrite(Standby, HIGH);
            pcf->digitalWrite(In1, LOW);
            pcf->digitalWrite(In2, HIGH);
            unlockI2C();
        }
    }
    else
    {
        digitalWrite(Standby, HIGH);
        digitalWrite(In1, LOW);
        digitalWrite(In2, HIGH);
    }

    analogWrite(PWM, speed);
}

void Motor::brake()
{
    if (pcf != nullptr)
    {
        if (lockI2C(20))
        {
            pcf->digitalWrite(In1, HIGH);
            pcf->digitalWrite(In2, HIGH);
            unlockI2C();
        }
    }
    else
    {
        digitalWrite(In1, HIGH);
        digitalWrite(In2, HIGH);
    }

    analogWrite(PWM, 0);
}

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
    delay(duration);
}