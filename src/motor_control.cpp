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

void brakeAllMotors()
{
    motorFL.brake();
    motorFR.brake();
    motorBL.brake();
    motorBR.brake();

    // Desactivar Standby mediante el registro en sombra atómico
    setStandbyPin(false);

    leftRearLed(HIGH);
    rightRearLed(HIGH);
}

// 🚀 Control directo a plena potencia usando los 4 comandos recibidos (-4095 a 4095)
void driveDirectRaw(int fl, int bl, int fr, int br)
{
    if (fl == 0 && fr == 0 && bl == 0 && br == 0)
    {
        brakeAllMotors();
        return;
    }

    // 1. Activa STBY solo si no estaba ya activo
    setStandbyPin(true);

    // 2. Envío directo del pulso PWM a cada motor
    motorFL.drive(fl);
    motorFR.drive(fr);
    motorBL.drive(bl);
    motorBR.drive(br);

    leftRearLed(LOW);
    rightRearLed(LOW);
}