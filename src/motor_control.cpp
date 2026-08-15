#include "config.h"
#include "motor_control.h"
#include "peripherals.h"
#include "i2c_manager.h"
#include "PCF8574.h"
#include "SparkFun_TB6612.h"

extern PCF8574 leftmotorscontrolpcf8574;
extern PCF8574 rightmotorscontrolpcf8574;

Motor leftMotor(In1pinleftMotor1, In2pinleftMotor1, PWMPinleftMotor, offset, STBYpin, &leftmotorscontrolpcf8574);
Motor rightMotor(In1pinrightMotor2, In2pinrightMotor2, PWMPinrightMotor, offset, STBYpin, &rightmotorscontrolpcf8574);

volatile float joystickX = 0.0f;
volatile float joystickY = 0.0f;

void setCarMotorsStandby(bool enable)
{
    if (lockI2C(20))
    {
        leftmotorscontrolpcf8574.digitalWrite(STBYpin, enable ? HIGH : LOW);
        unlockI2C();
    }
}

// int scaleMotorSpeed(float val)
// {
//     if (val == 0.0f) return 0;

//     int minPWM = 200; // Supera la zona muerta mecánica
//     int maxPWM = 255;

//     int absPWM = minPWM + (int)(fabs(val) * (maxPWM - minPWM));
//     return (val > 0) ? absPWM : -absPWM;
// }

int scaleMotorSpeed(float val)
{
    if (val == 0.0f)
        return 0;

    // 🚀 VELOCIDAD MÁXIMA CONSTANTE:
    // Retorna 255 si el joystick va adelante, y -255 si va en reversa
    return (val > 0) ? 255 : -255;
}

void processDifferentialDrive(float x, float y)
{
    if (x == 0.0f && y == 0.0f)
    {
        brake(leftMotor, rightMotor);
        leftRearLed(HIGH);
        rightRearLed(HIGH);
        return;
    }

    // 1. Cinemática Diferencial
    float left = y + x;
    float right = y - x;

    // 2. Normalización para no rebasar 1.0
    float maxVal = fabs(left);
    if (fabs(right) > maxVal)
        maxVal = fabs(right);

    if (maxVal > 1.0f)
    {
        left /= maxVal;
        right /= maxVal;
    }

    // 3. Escalado al rango de motores (200-255)
    int speedLeft = scaleMotorSpeed(left);
    int speedRight = scaleMotorSpeed(right);

    // 4. Aplicación de movimiento
    leftMotor.drive(speedLeft);
    rightMotor.drive(speedRight);

    leftRearLed(LOW);
    rightRearLed(LOW);
}