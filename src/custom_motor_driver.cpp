#include "custom_motor_driver.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Funciones globales para control de concurrencia I2C
extern bool lockI2C(TickType_t timeoutMs = 20);
extern void unlockI2C();

// Constructor con PCF8574 y PCA9685
Motor::Motor(int In1pin, int In2pin, int PWMpin, int offset, PCF8574 *pcfDev, Adafruit_PWMServoDriver *pcaController)
{
    In1 = In1pin;
    In2 = In2pin;
    PWM = PWMpin;
    Offset = offset;
    pcf = pcfDev;
    pca = pcaController;
}

void Motor::fwd(int speed)
{
    if (pcf != nullptr)
    {
        if (lockI2C(20))
        {

            pcf->digitalWrite(In1, HIGH);
            pcf->digitalWrite(In2, LOW);
            unlockI2C();
        }
    }
    else
    {

        digitalWrite(In1, HIGH);
        digitalWrite(In2, LOW);
    }

    pca->setPWM(PWM, 0, speed);
}

void Motor::rev(int speed)
{
    if (pcf != nullptr)
    {
        if (lockI2C(20))
        {

            pcf->digitalWrite(In1, LOW);
            pcf->digitalWrite(In2, HIGH);
            unlockI2C();
        }
    }
    else
    {

        digitalWrite(In1, LOW);
        digitalWrite(In2, HIGH);
    }

    pca->setPWM(PWM, 0, speed);
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

    pca->setPWM(PWM, 0, 0); // Detener el PWM
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