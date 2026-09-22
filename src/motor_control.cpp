#include "config.h"
#include "motor_control.h"
#include "peripherals.h"
#include "i2c_manager.h"
#include <PCF8574.h>
#include <Adafruit_PWMServoDriver.h>
#include "custom_motor_driver.h"

extern PCF8574 FMCpcf8574;
extern PCF8574 BMCpcf8574;
extern Adafruit_PWMServoDriver pca9685;

Motor motorFL(motorFLIn1pin, motorFLIn2pin, motorFLPWMPin, motorFLoffset, &FMCpcf8574, &pca9685);
Motor motorBL(motorBLIn1pin, motorBLIn2pin, motorBLPWMPin, motorBLoffset, &BMCpcf8574, &pca9685);
Motor motorFR(motorFRIn1pin, motorFRIn2pin, motorFRPWMPin, motorFRoffset, &FMCpcf8574, &pca9685);
Motor motorBR(motorBRIn1pin, motorBRIn2pin, motorBRPWMPin, motorBRoffset, &BMCpcf8574, &pca9685);

void stopAllMotors()
{
    // Desactivar Standby para apagar los transistores y ahorrar energía
    setStandbyPin(false);

    leftRearLed(HIGH);
    rightRearLed(HIGH);
}

void brakeAllMotors()
{
    setStandbyPin(true);
    
    motorFL.brake();
    motorBL.brake();
    motorFR.brake();
    motorBR.brake();

    leftRearLed(HIGH);
    rightRearLed(HIGH);
}

int mapMotorValue(int rawValue)
{
    if (rawValue == 0)
        return 0;
    const int MIN_PWM = 800, MAX_PWM = 4095;
    int sign = (rawValue > 0) ? 1 : -1;
    int absVal = constrain(abs(rawValue), 210, 1500);

    if (absVal <= 210)
        return MIN_PWM * sign;
    return map(absVal, 210, 1500, MIN_PWM, MAX_PWM) * sign;
}

void driveSafe(int p1, int p2, int p3, int p4)
{
    int safeFL = mapMotorValue(p1);
    int safeBL = mapMotorValue(p2);
    int safeFR = mapMotorValue(p3);
    int safeBR = mapMotorValue(p4);

    // Permitir girar sobre su propio eje o retroceder libremente.
    // Solo bloqueamos si el movimiento neto es frontal.
    int forwardIntent = p1 + p2 + p3 + p4;
    bool isTryingToGoForward = (forwardIntent > 200);

    if (enableObstacleAvoidance && obstacleFound && isTryingToGoForward)
    {
        brakeAllMotors();
    }
    else
    {
        driveDirectRaw(safeFL, safeBL, safeFR, safeBR);
    }
}

void driveDirectRaw(int fl, int bl, int fr, int br)
{
    if (fl == 0 && bl == 0 && fr == 0 && br == 0)
    {
        stopAllMotors();
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

void driveMecanum(int angle, int speed, int rotation, int rotationSpeed)
{
    float angle_rad = angle * PI / 180.0;
    float x_trans = -speed * sin(angle_rad);
    float y_trans = speed * cos(angle_rad);
    
    float rot_rad = rotation * PI / 180.0;
    float rot = -rotationSpeed * sin(rot_rad);
    
    float fl = y_trans + x_trans + rot;
    float fr = y_trans - x_trans - rot;
    float bl = y_trans - x_trans + rot;
    float br = y_trans + x_trans - rot;
    
    // Normalize if maximum exceeds 1500
    float max_val = max(max(abs(fl), abs(fr)), max(abs(bl), abs(br)));
    if (max_val > 1500)
    {
        fl = fl / max_val * 1500;
        fr = fr / max_val * 1500;
        bl = bl / max_val * 1500;
        br = br / max_val * 1500;
    }
    
    // Check if we are moving forward to trigger obstacle avoidance if needed
    int forwardIntent = (int)(fl + bl + fr + br);
    bool isTryingToGoForward = (forwardIntent > 200 && abs(y_trans) > abs(x_trans) && y_trans > 0);

    if (enableObstacleAvoidance && obstacleFound && isTryingToGoForward)
    {
        brakeAllMotors();
    }
    else
    {
        // call mapMotorValue on each and pass to driveDirectRaw
        int safeFL = mapMotorValue((int)fl);
        int safeBL = mapMotorValue((int)bl);
        int safeFR = mapMotorValue((int)fr);
        int safeBR = mapMotorValue((int)br);
        
        driveDirectRaw(safeFL, safeBL, safeFR, safeBR);
    }
}