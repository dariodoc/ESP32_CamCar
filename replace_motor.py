import os

content = open('src/motor_control.cpp', 'r', encoding='utf-8').read()
start_idx = content.find('void stopAllMotors()')
end_idx = content.find('void driveMecanum')

new_content = content[:start_idx] + '''static int targetFL = 0, targetBL = 0, targetFR = 0, targetBR = 0;
static int currentFL = 0, currentBL = 0, currentFR = 0, currentBR = 0;
static bool immediateBrake = false;
static TaskHandle_t motorTaskHandle = NULL;

void motorSlewTask(void *pvParameters)
{
    const int SLEW_STEP = 300; // Incremento máximo por ciclo (Rampa de aceleración)
    for (;;)
    {
        if (immediateBrake)
        {
            currentFL = 0; currentBL = 0; currentFR = 0; currentBR = 0;
            targetFL = 0; targetBL = 0; targetFR = 0; targetBR = 0;
            
            setStandbyPin(true);
            motorFL.brake();
            motorBL.brake();
            motorFR.brake();
            motorBR.brake();
            leftRearLed(HIGH);
            rightRearLed(HIGH);
            
            immediateBrake = false;
        }
        else
        {
            bool changed = false;

            if (currentFL < targetFL) { currentFL = min(currentFL + SLEW_STEP, targetFL); changed = true; }
            else if (currentFL > targetFL) { currentFL = max(currentFL - SLEW_STEP, targetFL); changed = true; }

            if (currentBL < targetBL) { currentBL = min(currentBL + SLEW_STEP, targetBL); changed = true; }
            else if (currentBL > targetBL) { currentBL = max(currentBL - SLEW_STEP, targetBL); changed = true; }

            if (currentFR < targetFR) { currentFR = min(currentFR + SLEW_STEP, targetFR); changed = true; }
            else if (currentFR > targetFR) { currentFR = max(currentFR - SLEW_STEP, targetFR); changed = true; }

            if (currentBR < targetBR) { currentBR = min(currentBR + SLEW_STEP, targetBR); changed = true; }
            else if (currentBR > targetBR) { currentBR = max(currentBR - SLEW_STEP, targetBR); changed = true; }

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
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz Update rate
    }
}

void setupMotors()
{
    if (motorTaskHandle == NULL)
    {
        xTaskCreatePinnedToCore(motorSlewTask, "MotorTask", 2048, NULL, 1, &motorTaskHandle, 1);
    }
}

void stopAllMotors()
{
    setStandbyPin(false);
    enableObstacleAvoidance = false;
    enableIROnlyMode = false;
    immediateBrake = true;
}

void brakeAllMotors()
{
    immediateBrake = true;
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

''' + content[end_idx:]

open('src/motor_control.cpp', 'w', encoding='utf-8').write(new_content)
