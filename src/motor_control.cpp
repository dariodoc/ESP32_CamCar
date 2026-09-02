#include "config.h"
#include "motor_control.h"
#include "peripherals.h"
#include "i2c_manager.h"
#include "PCF8574.h"
#include "Adafruit_PWMServoDriver.h"
#include "custom_motor_driver.h"

extern PCF8574 FMCpcf8574;
extern PCF8574 BMCpcf8574;
extern Adafruit_PWMServoDriver pca9685;

Motor motorFL(motorFLIn1pin, motorFLIn2pin, motorFLPWMPin, motorFLoffset, &FMCpcf8574, &pca9685);
Motor motorFR(motorFRIn1pin, motorFRIn2pin, motorFRPWMPin, motorFRoffset, &FMCpcf8574, &pca9685);
Motor motorBL(motorBLIn1pin, motorBLIn2pin, motorBLPWMPin, motorBLoffset, &BMCpcf8574, &pca9685);
Motor motorBR(motorBRIn1pin, motorBRIn2pin, motorBRPWMPin, motorBRoffset, &BMCpcf8574, &pca9685);

volatile float joystickX = 0.0f;
volatile float joystickY = 0.0f;

int scaleMotorSpeed(float val)
{
    if (val == 0.0f)
        return 0;

    int minPWM = 3072; // Supera la zona muerta mecánica
    int maxPWM = 4095;

    int absPWM = minPWM + (int)(fabs(val) * (maxPWM - minPWM));
    return (val > 0) ? absPWM : -absPWM;
}

void brakeAllMotors()
{
    motorFL.brake();
    motorFR.brake();
    motorBL.brake();
    motorBR.brake();

    if (lockI2C(20))
    {
        BMCpcf8574.digitalWrite(STBYpin, LOW);
        unlockI2C();
    }
    
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

    if (lockI2C(20))
    {
        BMCpcf8574.digitalWrite(STBYpin, HIGH);
        unlockI2C();
    }

    motorFL.drive(speedLeft);
    motorFR.drive(speedRight);
    motorBL.drive(speedLeft);
    motorBR.drive(speedRight);

    leftRearLed(LOW);
    rightRearLed(LOW);
}