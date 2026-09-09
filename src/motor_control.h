#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

void brakeAllMotors();
void stopAllMotors();
void driveDirectRaw(int fl, int bl, int fr, int br);
void driveSafe(int p1, int p2, int p3, int p4);

#endif