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

static int targetFL = 0, targetBL = 0, targetFR = 0, targetBR = 0;
static int currentFL = 0, currentBL = 0, currentFR = 0, currentBR = 0;
static bool immediateBrake = false;
static bool immediateStop = false;
static TaskHandle_t motorTaskHandle = NULL;

void motorSlewTask(void *pvParameters)
{
    const int SLEW_STEP = 300; // Incremento máximo por ciclo (Rampa de aceleración)
    for (;;)
    {
        if (immediateBrake || immediateStop)
        {
            currentFL = 0; currentBL = 0; currentFR = 0; currentBR = 0;
            targetFL = 0; targetBL = 0; targetFR = 0; targetBR = 0;
            
            if (immediateStop) {
                setStandbyPin(false); // Pin Standby LOW: Desconecta fisicamente los motores (Coast)
                motorFL.drive(0);
                motorBL.drive(0);
                motorFR.drive(0);
                motorBR.drive(0);
            } else {
                // Freno electrónico activo (Short-circuit brake)
                setStandbyPin(true);
                motorFL.brake();
                motorBL.brake();
                motorFR.brake();
                motorBR.brake();
                syncMotorsI2C();
                
                // Aplicar freno solo por 150ms para evitar sobrecorriente (Brownout)
                vTaskDelay(pdMS_TO_TICKS(150));
                
                // Luego pasar a estado libre (Coast)
                setStandbyPin(false);
                motorFL.drive(0);
                motorBL.drive(0);
                motorFR.drive(0);
                motorBR.drive(0);
            }
            leftRearLed(HIGH);
            rightRearLed(HIGH);
            
            syncMotorsI2C();
            immediateBrake = false;
            immediateStop = false;
        }
        else
        {
            bool changed = false;

            auto applyRamp = [](int &current, int target) {
                const int SLEW_STEP = 300;
                const int MIN_PWM = 819;
                
                if (current < target) {
                    if (current == 0) current = MIN_PWM;
                    else current = min(current + SLEW_STEP, target);
                }
                else if (current > target) {
                    if (current == 0) current = -MIN_PWM;
                    else current = max(current - SLEW_STEP, target);
                }
                
                // Cut-off a 0 si caemos en la zona muerta
                if (abs(current) < MIN_PWM) current = 0;
            };

            if (currentFL != targetFL) { applyRamp(currentFL, targetFL); changed = true; }
            if (currentBL != targetBL) { applyRamp(currentBL, targetBL); changed = true; }
            if (currentFR != targetFR) { applyRamp(currentFR, targetFR); changed = true; }
            if (currentBR != targetBR) { applyRamp(currentBR, targetBR); changed = true; }

            if (changed)
            {
                if (currentFL == 0 && currentBL == 0 && currentFR == 0 && currentBR == 0)
                {
                    setStandbyPin(true);
                    motorFL.brake();
                    motorBL.brake();
                    motorFR.brake();
                    motorBR.brake();
                    leftRearLed(HIGH);
                    rightRearLed(HIGH);
                }
                else
                {
                    setStandbyPin(true);
                    motorFL.drive(currentFL);
                    motorBL.drive(currentBL);
                    motorFR.drive(currentFR);
                    motorBR.drive(currentBR);
                    leftRearLed(LOW);
                    rightRearLed(LOW);
                }
                syncMotorsI2C();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz Update rate
    }
}

void setupMotors()
{
    if (motorTaskHandle == NULL)
    {
        xTaskCreatePinnedToCore(motorSlewTask, "MotorTask", 4096, NULL, 1, &motorTaskHandle, 1);
    }
}

void stopAllMotors()
{
    enableObstacleAvoidance = false;
    enableIROnlyMode = false;
    immediateStop = true;
}

void brakeAllMotors()
{
    immediateBrake = true;
}

int mapMotorValue(int rawValue)
{
    if (rawValue == 0)
        return 0;
    const int MIN_PWM = 0, MAX_PWM = 4095;
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

    driveDirectRaw(safeFL, safeBL, safeFR, safeBR);
}

void driveDirectRaw(int fl, int bl, int fr, int br)
{
    if (fl == 0 && bl == 0 && fr == 0 && br == 0)
    {
        immediateBrake = true;
        return;
    }

    targetFL = fl;
    targetBL = bl;
    targetFR = fr;
    targetBR = br;
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
    
    // call mapMotorValue on each and pass to driveDirectRaw
    int safeFL = mapMotorValue((int)fl);
    int safeBL = mapMotorValue((int)bl);
    int safeFR = mapMotorValue((int)fr);
    int safeBR = mapMotorValue((int)br);
    
    driveDirectRaw(safeFL, safeBL, safeFR, safeBR);
}