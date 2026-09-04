#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

void brakeAllMotors();
void driveDirectRaw(int fl, int bl, int fr, int br);

#endif