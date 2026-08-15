#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

extern volatile float joystickX;
extern volatile float joystickY;

void processDifferentialDrive(float x, float y);
void setCarMotorsStandby(bool enable);

#endif // MOTOR_CONTROL_H