#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

extern volatile int motorSpeed;
extern volatile int currentDirection;

void moveCar(int inputValue);
void setCarMotorsStandby(bool enable);
#endif // MOTOR_CONTROL_H