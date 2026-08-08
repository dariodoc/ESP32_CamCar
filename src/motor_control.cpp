#include "config.h"
#include "motor_control.h"
#include "peripherals.h"
#include "i2c_manager.h"
#include "PCF8574.h"
#include "SparkFun_TB6612.h"

extern PCF8574 motorcontrolpcf8574;

// Definición de los objetos motor y variables de velocidad
Motor leftMotor(P3, P4, 1, 1, P2);
Motor rightMotor(P1, P0, 3, 1, P2);

volatile int motorSpeed = 255;
volatile int currentDirection = STOP;

void setCarMotorsStandby(bool enable)
{
    if (lockI2C(20))
    {
        motorcontrolpcf8574.digitalWrite(P2, enable ? HIGH : LOW);
        unlockI2C();
    }
}

void moveCar(int inputValue)
{
#ifdef DEBUG
    Serial.printf("Got value as %d\n", inputValue);
#endif

    switch (inputValue)
    {
    case FORWARD:
        currentDirection = FORWARD;
        forward(leftMotor, rightMotor, motorSpeed);
        break;
    case BACKWARD:
        currentDirection = BACKWARD;
        back(leftMotor, rightMotor, motorSpeed);
        break;
    case LEFT:
        currentDirection = LEFT;
        left(leftMotor, rightMotor, motorSpeed);
        break;
    case RIGHT:
        currentDirection = RIGHT;
        right(leftMotor, rightMotor, motorSpeed);
        break;
    case FORWARDLEFT:
        currentDirection = FORWARDLEFT;
        forwardleft(leftMotor, rightMotor, motorSpeed);
        break;
    case FORWARDRIGHT:
        currentDirection = FORWARDRIGHT;
        forwardright(leftMotor, rightMotor, motorSpeed);
        break;
    case BACKLEFT:
        currentDirection = BACKLEFT;
        backleft(leftMotor, rightMotor, motorSpeed);
        break;
    case BACKRIGHT:
        currentDirection = BACKRIGHT;
        backright(leftMotor, rightMotor, motorSpeed);
        break;
    case STOP:
        currentDirection = STOP;
        brake(leftMotor, rightMotor);
        leftBackLed(HIGH);
        rightBackLed(HIGH);
        return;
    }

    leftBackLed(LOW);
    rightBackLed(LOW);
}