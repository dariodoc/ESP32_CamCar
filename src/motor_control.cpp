#include "config.h"
#include "motor_control.h"
#include "peripherals.h"
#include "i2c_manager.h"
#include "PCF8574.h"
#include "SparkFun_TB6612.h"

extern PCF8574 leftmotorscontrolpcf8574;
extern PCF8574 rightmotorscontrolpcf8574;

// 🚀 INSTANCIACIÓN POR PUNTERO:
// Si ambos motores están cableados al PCF izquierdo (0x20):
Motor leftMotor(In1pinleftMotor1, In2pinleftMotor1, PWMPinleftMotor, offset, STBYpin, &leftmotorscontrolpcf8574);
Motor rightMotor(In1pinrightMotor2, In2pinrightMotor2, PWMPinrightMotor, offset, STBYpin, &rightmotorscontrolpcf8574);

// (Nota: Si el motor derecho estuviese físicamente conectado al segundo PCF (0x24),
//  solo le pasarías &rightmotorscontrolpcf8574 al segundo objeto y funcionaría automáticamente).

volatile int motorSpeed = 255;
volatile int currentDirection = STOP;

void setCarMotorsStandby(bool enable)
{
    if (lockI2C(20))
    {
        leftmotorscontrolpcf8574.digitalWrite(STBYpin, enable ? HIGH : LOW);
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
        leftRearLed(HIGH);
        rightRearLed(HIGH);
        return;
    }

    leftRearLed(LOW);
    rightRearLed(LOW);
}