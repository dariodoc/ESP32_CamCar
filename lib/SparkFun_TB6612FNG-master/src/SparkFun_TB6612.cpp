/******************************************************************************
TB6612.cpp
TB6612FNG H-Bridge Motor Driver Example code (Refactorizado con Mutex FreeRTOS)
******************************************************************************/

#include "SparkFun_TB6612.h"
#include <Arduino.h>

#define PCF8574_ON // Uncomment to use PCF8574 by including "PCF8574.h"
#ifdef PCF8574_ON
#include <PCF8574.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
extern bool lockI2C(TickType_t timeoutMs = 20);
extern void unlockI2C();
extern PCF8574 leftmotorscontrolpcf8574;
extern PCF8574 rightmotorscontrolpcf8574;
#endif

Motor::Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin)
{
  In1 = In1pin;
  In2 = In2pin;
  PWM = PWMpin;
  Offset = offset;
  Standby = STBYpin;

  pinMode(PWM, OUTPUT);
}

void Motor::drive(int speed)
{
  speed = speed * Offset;
  if (speed >= 0)
    fwd(speed);
  else
    rev(-speed);
}

void Motor::fwd(int speed)
{
#ifdef PCF8574_ON
  if (lockI2C(20))
  {
    leftmotorscontrolpcf8574.digitalWrite(In1, HIGH);
    leftmotorscontrolpcf8574.digitalWrite(In2, LOW);
    unlockI2C();
  }
#else
  digitalWrite(Standby, HIGH);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
#endif

  analogWrite(PWM, speed);
}

void Motor::drive(int speed, int duration)
{
  drive(speed);
  delay(duration);
}

void Motor::rev(int speed)
{
#ifdef PCF8574_ON
  if (lockI2C(20))
  {
    leftmotorscontrolpcf8574.digitalWrite(In1, LOW);
    leftmotorscontrolpcf8574.digitalWrite(In2, HIGH);
    unlockI2C();
  }
#else
  digitalWrite(Standby, HIGH);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
#endif

  analogWrite(PWM, speed);
}

void Motor::brake()
{
#ifdef PCF8574_ON
  if (lockI2C(20))
  {

    leftmotorscontrolpcf8574.digitalWrite(In1, HIGH);
    leftmotorscontrolpcf8574.digitalWrite(In2, HIGH);
    unlockI2C();
  }
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
  if (lockI2C(20))
  {
    leftmotorscontrolpcf8574.digitalWrite(Standby, LOW);
    unlockI2C();
  }
#else
  digitalWrite(Standby, LOW);
#endif
}

void forward(Motor &motor1, Motor &motor2, int speed)
{
  motor1.drive(speed);
  motor2.drive(speed);
}

void back(Motor &motor1, Motor &motor2, int speed)
{
  int temp = abs(speed);
  motor1.drive(-temp);
  motor2.drive(-temp);
}

void left(Motor &left, Motor &right, int speed)
{
  int temp = abs(speed);
  left.drive(-temp);
  right.drive(temp);
}

void right(Motor &left, Motor &right, int speed)
{
  int temp = abs(speed);
  left.drive(temp);
  right.drive(-temp);
}

void brake(Motor &motor1, Motor &motor2)
{
  motor1.brake();
  motor2.brake();
}

void forwardleft(Motor &left, Motor &right, int speed)
{
  int temp = abs(speed) / 2;
  left.drive(temp);
  right.drive(speed);
}

void forwardright(Motor &left, Motor &right, int speed)
{
  int temp = abs(speed) / 2;
  left.drive(speed);
  right.drive(temp);
}

void backleft(Motor &left, Motor &right, int speed)
{
  int temp = abs(speed) / 2;
  left.drive(-temp);
  right.drive(-speed);
}

void backright(Motor &left, Motor &right, int speed)
{
  int temp = abs(speed) / 2;
  left.drive(-speed);
  right.drive(-temp);
}