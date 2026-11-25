#include "config.h"
#include "motor_control.h"
#include "peripherals.h"
#include "i2c_manager.h"
#include <Adafruit_PWMServoDriver.h>
#include "custom_motor_driver.h"

extern Adafruit_PWMServoDriver pca9685;

Motor motorFL(motorFLIn1pin, motorFLIn2pin, motorFLPWMPin, motorFLoffset, PCF_FMC_ADDR, &pca9685);
Motor motorBL(motorBLIn1pin, motorBLIn2pin, motorBLPWMPin, motorBLoffset, PCF_BMC_ADDR, &pca9685);
Motor motorFR(motorFRIn1pin, motorFRIn2pin, motorFRPWMPin, motorFRoffset, PCF_FMC_ADDR, &pca9685);
Motor motorBR(motorBRIn1pin, motorBRIn2pin, motorBRPWMPin, motorBRoffset, PCF_BMC_ADDR, &pca9685);

static portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int targetFL = 0, targetBL = 0, targetFR = 0, targetBR = 0;
static int currentFL = 0, currentBL = 0, currentFR = 0, currentBR = 0;
static volatile bool immediateBrake = false;
static volatile bool immediateStop = false;
static volatile bool forceMotorHwResync = false;
static volatile TickType_t lastManualDriveTick = 0;
static TaskHandle_t motorTaskHandle = NULL;

void notifyMotorsI2CResetLocked()
{
    motorFL.invalidateCache();
    motorBL.invalidateCache();
    motorFR.invalidateCache();
    motorBR.invalidateCache();
    forceMotorHwResync = true;
}

void motorSlewTask(void *pvParameters)
{
    bool pendingBrake = false;
    bool pendingStop = false;
    bool lastTickHwOk = true;
    int i2cRetryCount = 0;
    int idleRefreshTicks = 0;

    for (;;)
    {
        int tFL, tBL, tFR, tBR;
        bool doBrake, doStop;
        TickType_t lastTick;

        taskENTER_CRITICAL(&motorMux);
        tFL = targetFL;
        tBL = targetBL;
        tFR = targetFR;
        tBR = targetBR;
        doBrake = immediateBrake;
        doStop = immediateStop;
        lastTick = lastManualDriveTick;
        immediateBrake = false;
        immediateStop = false;
        taskEXIT_CRITICAL(&motorMux);

        if (doStop) {
            pendingStop = true;
            pendingBrake = false;
        } else if (doBrake) {
            pendingBrake = true;
        } else if (tFL != 0 || tBL != 0 || tFR != 0 || tBR != 0) {
            pendingBrake = false;
            pendingStop = false;
        }

        // Watchdog de movimiento manual: si pasan >800ms sin keep-alive mientras gira, frenar
        if (!enableObstacleAvoidance && !enableIROnlyMode &&
            (tFL != 0 || tBL != 0 || tFR != 0 || tBR != 0 ||
             currentFL != 0 || currentBL != 0 || currentFR != 0 || currentBR != 0))
        {
            if ((xTaskGetTickCount() - lastTick) > pdMS_TO_TICKS(800))
            {
                taskENTER_CRITICAL(&motorMux);
                targetFL = 0; targetBL = 0; targetFR = 0; targetBR = 0;
                taskEXIT_CRITICAL(&motorMux);
                tFL = 0; tBL = 0; tFR = 0; tBR = 0;
                pendingBrake = true;
            }
        }

        if (pendingBrake || pendingStop)
        {
            currentFL = 0; currentBL = 0; currentFR = 0; currentBR = 0;

            bool s0, s1, s2, s3, s4, s5;
            if (pendingStop) {
                s0 = setStandbyPin(false); // Pin Standby LOW: Desconecta fisicamente los motores (Coast)
                s1 = motorFL.drive(0);
                s2 = motorBL.drive(0);
                s3 = motorFR.drive(0);
                s4 = motorBR.drive(0);
            } else {
                // Freno electrónico activo continuo
                s0 = setStandbyPin(true);
                s1 = motorFL.brake();
                s2 = motorBL.brake();
                s3 = motorFR.brake();
                s4 = motorBR.brake();
            }
            leftRearLed(HIGH);
            rightRearLed(HIGH);
            s5 = syncMotorsI2C();

            if (s0 && s1 && s2 && s3 && s4 && s5)
            {
                pendingBrake = false;
                pendingStop = false;
                lastTickHwOk = true;
                i2cRetryCount = 0;
            }
            else
            {
                lastTickHwOk = false;
                if (++i2cRetryCount >= 3)
                {
                    if (recoverI2CBus())
                    {
                        notifyMotorsI2CResetLocked();
                    }
                    i2cRetryCount = 0;
                }
            }
            idleRefreshTicks = 0;
        }
        else
        {
            bool changed = false;
            bool needResync = forceMotorHwResync;
            if (needResync) forceMotorHwResync = false;

            auto applyRamp = [](int &current, int target) {
                const int SLEW_STEP = 150;
                const int MIN_PWM = 400;

                if (target > 0 && target < MIN_PWM) target = MIN_PWM;
                else if (target < 0 && target > -MIN_PWM) target = -MIN_PWM;

                if (current < target) {
                    if (current == 0) current = MIN_PWM;
                    else current = min(current + SLEW_STEP, target);
                }
                else if (current > target) {
                    if (current == 0) current = -MIN_PWM;
                    else current = max(current - SLEW_STEP, target);
                }

                // Cut-off a 0 solo al detener o cruzar de signo
                if (abs(current) < MIN_PWM &&
                    (target == 0 || (current > 0 && target < 0) || (current < 0 && target > 0))) {
                    current = 0;
                }
            };

            if (currentFL != tFL) { applyRamp(currentFL, tFL); changed = true; }
            if (currentBL != tBL) { applyRamp(currentBL, tBL); changed = true; }
            if (currentFR != tFR) { applyRamp(currentFR, tFR); changed = true; }
            if (currentBR != tBR) { applyRamp(currentBR, tBR); changed = true; }

            if (changed || !lastTickHwOk || needResync)
            {
                bool s0, s1, s2, s3, s4, s5;
                if (currentFL == 0 && currentBL == 0 && currentFR == 0 && currentBR == 0)
                {
                    s0 = setStandbyPin(true);
                    s1 = motorFL.brake();
                    s2 = motorBL.brake();
                    s3 = motorFR.brake();
                    s4 = motorBR.brake();
                    leftRearLed(HIGH);
                    rightRearLed(HIGH);
                }
                else
                {
                    s0 = setStandbyPin(true);
                    s1 = motorFL.drive(currentFL);
                    s2 = motorBL.drive(currentBL);
                    s3 = motorFR.drive(currentFR);
                    s4 = motorBR.drive(currentBR);
                    leftRearLed(LOW);
                    rightRearLed(LOW);
                }
                s5 = syncMotorsI2C();

                if (s0 && s1 && s2 && s3 && s4 && s5)
                {
                    lastTickHwOk = true;
                    i2cRetryCount = 0;
                }
                else
                {
                    lastTickHwOk = false;
                    if (++i2cRetryCount >= 3)
                    {
                        if (recoverI2CBus())
                        {
                            notifyMotorsI2CResetLocked();
                        }
                        i2cRetryCount = 0;
                    }
                }
                idleRefreshTicks = 0;
            }
            else if (tFL == 0 && tBL == 0 && tFR == 0 && tBR == 0 &&
                     currentFL == 0 && currentBL == 0 && currentFR == 0 && currentBR == 0)
            {
                if (++idleRefreshTicks >= 50)
                {
                    syncMotorsI2C();
                    idleRefreshTicks = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz Update rate
    }
}

void setupMotors()
{
    if (motorTaskHandle == NULL)
    {
        xTaskCreatePinnedToCore(motorSlewTask, "MotorTask", 4096, NULL, 2, &motorTaskHandle, 1);
    }
}

void stopAllMotors()
{
    enableObstacleAvoidance = false;
    enableIROnlyMode = false;
    taskENTER_CRITICAL(&motorMux);
    targetFL = 0;
    targetBL = 0;
    targetFR = 0;
    targetBR = 0;
    immediateStop = true;
    immediateBrake = false;
    taskEXIT_CRITICAL(&motorMux);
}

void brakeAllMotors()
{
    taskENTER_CRITICAL(&motorMux);
    targetFL = 0;
    targetBL = 0;
    targetFR = 0;
    targetBR = 0;
    immediateBrake = true;
    taskEXIT_CRITICAL(&motorMux);
}

int mapMotorValue(int rawValue)
{
    int absRaw = abs(rawValue);
    if (absRaw <= 210)
        return 0;
    const int MIN_PWM = 400, MAX_PWM = 4095;
    int sign = (rawValue > 0) ? 1 : -1;
    int absVal = constrain(absRaw, 210, 1500);
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
    taskENTER_CRITICAL(&motorMux);
    if (fl == 0 && bl == 0 && fr == 0 && br == 0)
    {
        targetFL = 0;
        targetBL = 0;
        targetFR = 0;
        targetBR = 0;
        immediateBrake = true;
        immediateStop = false;
    }
    else
    {
        targetFL = fl;
        targetBL = bl;
        targetFR = fr;
        targetBR = br;
        immediateBrake = false;
        immediateStop = false;
        lastManualDriveTick = xTaskGetTickCount();
    }
    taskEXIT_CRITICAL(&motorMux);
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