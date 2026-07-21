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
extern PCF8574 motorcontrolpcf8574;
#endif

Motor::Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin)
{
  In1 = In1pin;
  In2 = In2pin;
  PWM = PWMpin;
  Standby = STBYpin;
  Offset = offset;

  // 🔥 BLINDAJE DE FRECUENCIA PARA ESP32 🔥
  if (PWM == 1)
  {
    ledcSetup(5, 5000, 8); // Canal 5, 5000Hz, 8-bits (0-255)
    ledcAttachPin(PWM, 5);
  }
  else if (PWM == 3)
  {
    ledcSetup(6, 5000, 8); // Canal 6, 5000Hz, 8-bits (0-255)
    ledcAttachPin(PWM, 6);
  }
  else
  {
    pinMode(PWM, OUTPUT);
  }
}

void setMotorsStandby(bool enable)
{
#ifdef PCF8574_ON
  if (lockI2C(20))
  {
    // Supongamos que P2 es tu pin de Standby en el PCF
    motorcontrolpcf8574.digitalWrite(P2, enable ? HIGH : LOW);
    unlockI2C();
  }
#else
  digitalWrite(STANDBY_PIN, enable ? HIGH : LOW);
#endif
}

void Motor::drive(int speed)
{
  // Ya NO tomamos Mutex aquí para Standby
  speed = speed * Offset;
  if (speed >= 0)
    fwd(speed);
  else
    rev(-speed);
}

void Motor::fwd(int speed)
{
#ifdef PCF8574_ON
  // Tomamos el Mutex UNA SOLA VEZ para levantar Standby y los pines de dirección
  if (lockI2C(20))
  {

    motorcontrolpcf8574.digitalWrite(In1, HIGH);
    motorcontrolpcf8574.digitalWrite(In2, LOW);
    unlockI2C();
  }
#else
  digitalWrite(Standby, HIGH);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
#endif

  if (PWM == 1)
    ledcWrite(5, speed);
  else if (PWM == 3)
    ledcWrite(6, speed);
}

// void Motor::drive(int speed, int duration)
// {
//   drive(speed);
//   delay(duration);
// }

void Motor::rev(int speed)
{
#ifdef PCF8574_ON
  // 🔒 Un solo bloqueo para Standby + In1 + In2
  if (lockI2C(20))
  {

    motorcontrolpcf8574.digitalWrite(In1, LOW);
    motorcontrolpcf8574.digitalWrite(In2, HIGH);
    unlockI2C();
  }
#else
  digitalWrite(Standby, HIGH);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
#endif

  // PWM directo por hardware (fuera del bus I2C)
  if (PWM == 1)
    ledcWrite(5, speed);
  else if (PWM == 3)
    ledcWrite(6, speed);
}

void Motor::brake()
{
#ifdef PCF8574_ON
  // Bloqueamos I2C una sola vez para las 3 salidas del PCF8574
  if (lockI2C(20))
  {

    motorcontrolpcf8574.digitalWrite(In1, HIGH);
    motorcontrolpcf8574.digitalWrite(In2, HIGH);
    unlockI2C();
  }
#else
  digitalWrite(Standby, LOW);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, HIGH);
#endif

  if (PWM == 1)
    ledcWrite(5, 0);
  else if (PWM == 3)
    ledcWrite(6, 0);
}

// void Motor::standby()
// {
// #ifdef PCF8574_ON
//   if (lockI2C(20))
//   {
//     motorcontrolpcf8574.digitalWrite(Standby, LOW);
//     unlockI2C();
//   }
// #else
//   digitalWrite(Standby, LOW);
// #endif
// }

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