/******************************************************************************
TB6612.cpp
TB6612FNG H-Bridge Motor Driver Example code
Michelle @ SparkFun Electronics
8/20/16
https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library

Uses 2 motors to show examples of the functions in the library.  This causes
a robot to do a little 'jig'.  Each movement has an equal and opposite movement
so assuming your motors are balanced the bot should end up at the same place it
started.

Resources:
TB6612 SparkFun Library

Development environment specifics:
Developed on Arduino 1.6.4
Developed with ROB-9457
******************************************************************************/

#include "SparkFun_TB6612.h"
#include <Arduino.h>

#define PCF8574_ON // Uncomment to use PCF8574 by including "PCF8574.h"
#ifdef PCF8574_ON
#include <PCF8574.h>
extern PCF8574 pcf8574;
#endif

Motor::Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin)
{
  In1 = In1pin;
  In2 = In2pin;
  PWM = PWMpin;
  Standby = STBYpin;
  Offset = offset;

#ifdef PCF8574_ON
  pcf8574.pinMode(In1, OUTPUT);
  pcf8574.pinMode(In2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pcf8574.pinMode(Standby, OUTPUT);

#else
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(Standby, OUTPUT);
#endif
}

void Motor::drive(int speed)
{
#ifdef PCF8574_ON
  pcf8574.digitalWrite(Standby, HIGH);
#else
  digitalWrite(Standby, HIGH);
#endif
  speed = speed * Offset;
  if (speed >= 0)
    fwd(speed);
  else
    rev(-speed);
}
void Motor::drive(int speed, int duration)
{
  drive(speed);
  delay(duration);
}

void Motor::fwd(int speed)
{
#ifdef PCF8574_ON
  pcf8574.digitalWrite(In1, HIGH);
  pcf8574.digitalWrite(In2, LOW);
#else
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
#endif
  analogWrite(PWM, speed);
}

void Motor::rev(int speed)
{
#ifdef PCF8574_ON
  pcf8574.digitalWrite(In1, LOW);
  pcf8574.digitalWrite(In2, HIGH);
#else
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
#endif
  analogWrite(PWM, speed);
}

void Motor::brake()
{
#ifdef PCF8574_ON
  pcf8574.digitalWrite(Standby, LOW);
  pcf8574.digitalWrite(In1, HIGH);
  pcf8574.digitalWrite(In2, HIGH);
#else
  digitalWrite(Standby, LOW);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, HIGH);
#endif
  analogWrite(PWM, 0);
}

void Motor::standby()
{
#ifdef PCF8574_ON
  pcf8574.digitalWrite(Standby, LOW);
#else
  digitalWrite(Standby, LOW);
#endif
}

void forward(Motor motor1, Motor motor2, int speed)
{
  motor1.drive(speed);
  motor2.drive(speed);
}
// void forward(Motor motor1, Motor motor2)
// {
//   motor1.drive(DEFAULTSPEED);
//   motor2.drive(DEFAULTSPEED);
// }

void back(Motor motor1, Motor motor2, int speed)
{
  int temp = abs(speed);
  motor1.drive(-temp);
  motor2.drive(-temp);
}
// void back(Motor motor1, Motor motor2)
// {
//   motor1.drive(-DEFAULTSPEED);
//   motor2.drive(-DEFAULTSPEED);
// }
void left(Motor left, Motor right, int speed)
{
  int temp = abs(speed);
  left.drive(-temp);
  right.drive(temp);
}
void right(Motor left, Motor right, int speed)
{
  int temp = abs(speed) ;
  left.drive(temp);
  right.drive(-temp);
}
void brake(Motor motor1, Motor motor2)
{
  motor1.brake();
  motor2.brake();
}
void forwardleft(Motor left, Motor right, int speed)
{
  int temp = abs(speed) / 2;
  left.drive(temp);
  right.drive(speed);
}
void forwardright(Motor left, Motor right, int speed)
{
  int temp = abs(speed) / 2;
  left.drive(speed);
  right.drive(temp);
}
void backleft(Motor left, Motor right, int speed)
{
  int temp = abs(speed) / 2;
  left.drive(-temp);
  right.drive(-speed);
}
void backright(Motor left, Motor right, int speed)
{
  int temp = abs(speed) / 2;
  left.drive(-speed);
  right.drive(-temp);
}