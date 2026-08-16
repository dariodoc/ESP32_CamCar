#include "config.h"
#include "motor_control.h"
#include "peripherals.h"
#include "i2c_manager.h"
#include "PCF8574.h"
#include "custom_motor_driver.h"

extern PCF8574 LMCpcf8574;
extern PCF8574 RMCpcf8574;

Motor motorFL(motorFLIn1pin, motorFLIn2pin, motorFLPWMPin, motorFLoffset, leftSTBYpin, &LMCpcf8574);
Motor motorFR(motorFRIn1pin, motorFRIn2pin, motorFRPWMPin, motorFRoffset, leftSTBYpin, &RMCpcf8574);
Motor motorBL(motorBLIn1pin, motorBLIn2pin, motorBLPWMPin, motorBLoffset, leftSTBYpin, &LMCpcf8574);
Motor motorBR(motorBRIn1pin, motorBRIn2pin, motorBRPWMPin, motorBRoffset, leftSTBYpin, &RMCpcf8574);

volatile float joystickX = 0.0f;
volatile float joystickY = 0.0f;

int scaleMotorSpeed(float val)
{
    if (val == 0.0f)
        return 0;

    int minPWM = 210; // Supera la zona muerta mecánica
    int maxPWM = 255;

    int absPWM = minPWM + (int)(fabs(val) * (maxPWM - minPWM));
    return (val > 0) ? absPWM : -absPWM;
}

// int scaleMotorSpeed(float val)
// {
//     if (val == 0.0f)
//         return 0;

//     // 🚀 VELOCIDAD MÁXIMA CONSTANTE:
//     // Retorna 255 si el joystick va adelante, y -255 si va en reversa
//     return (val > 0) ? 255 : -255;
// }

void brakeAllMotors()
{
    motorFL.brake();
    motorFR.brake();
    motorBL.brake();
    motorBR.brake();

    LMCpcf8574.digitalWrite(leftSTBYpin, LOW);
    RMCpcf8574.digitalWrite(rightSTBYpin, LOW);

    leftRearLed(HIGH);
    rightRearLed(HIGH);
}

void processDifferentialDrive(float x, float y)
{
    if (x == 0.0f && y == 0.0f)
    {
        brakeAllMotors();

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
    motorFL.drive(speedLeft);
    motorFR.drive(speedRight);
    motorBL.drive(speedLeft);
    motorBR.drive(speedRight);

    leftRearLed(LOW);
    rightRearLed(LOW);
}