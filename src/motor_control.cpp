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
Motor motorBL(motorBLIn1pin, motorBLIn2pin, motorBLPWMPin, motorBLoffset, &BMCpcf8574, &pca9685);
Motor motorFR(motorFRIn1pin, motorFRIn2pin, motorFRPWMPin, motorFRoffset, &FMCpcf8574, &pca9685);
Motor motorBR(motorBRIn1pin, motorBRIn2pin, motorBRPWMPin, motorBRoffset, &BMCpcf8574, &pca9685);

void brakeAllMotors()
{
    motorFL.brake();
    motorBL.brake();
    motorFR.brake();
    motorBR.brake();

    setStandbyPin(false);

    leftRearLed(HIGH);
    rightRearLed(HIGH);
}

void driveDirectRaw(int fl, int bl, int fr, int br)
{
    if (fl == 0 && bl == 0 && fr == 0 && br == 0)
    {
        brakeAllMotors();
        return;
    }

    setStandbyPin(true);

    motorFL.drive(fl);
    motorBL.drive(bl);
    motorFR.drive(fr);    
    motorBR.drive(br);

    leftRearLed(LOW);
    rightRearLed(LOW);
}