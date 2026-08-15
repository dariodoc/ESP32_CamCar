/******************************************************************************
TB6612.cpp
Refactorizado con Mutex FreeRTOS y puntero dinámico a PCF8574
******************************************************************************/

#include "SparkFun_TB6612.h"
#include <Arduino.h>

#define PCF8574_ON
#ifdef PCF8574_ON
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
extern bool lockI2C(TickType_t timeoutMs = 20);
extern void unlockI2C();
#endif

// Constructor base (GPIO directos / Fallback)
Motor::Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin)
{
  In1 = In1pin;
  In2 = In2pin;
  PWM = PWMpin;
  Offset = offset;
  Standby = STBYpin;

  pinMode(PWM, OUTPUT);
}

// 🚀 Constructor con soporte para PCF8574 vía Puntero
Motor::Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin, PCF8574 *pcfDev)
{
  In1 = In1pin;
  In2 = In2pin;
  PWM = PWMpin;
  Offset = offset;
  Standby = STBYpin;
  pcf = pcfDev; // Guarda la dirección de memoria del PCF asignado

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
  if (pcf != NULL && lockI2C(20))
  {
    pcf->digitalWrite(Standby, HIGH);
    pcf->digitalWrite(In1, HIGH);
    pcf->digitalWrite(In2, LOW);
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
  if (pcf != NULL && lockI2C(20))
  {
    pcf->digitalWrite(Standby, HIGH);
    pcf->digitalWrite(In1, LOW);
    pcf->digitalWrite(In2, HIGH);
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
  if (pcf != NULL && lockI2C(20))
  {
    pcf->digitalWrite(Standby, LOW);
    pcf->digitalWrite(In1, HIGH);
    pcf->digitalWrite(In2, HIGH);
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
  if (pcf != NULL && lockI2C(20))
  {
    pcf->digitalWrite(Standby, LOW);
    unlockI2C();
  }
#else
  digitalWrite(Standby, LOW);
#endif
}

// Sobregargas de funciones globales
void forward(Motor &motor1, Motor &motor2, int speed)
{
  motor1.drive(speed);
  motor2.drive(speed);
}
void forward(Motor &motor1, Motor &motor2) { forward(motor1, motor2, DEFAULTSPEED); }
void back(Motor &motor1, Motor &motor2, int speed)
{
  int temp = abs(speed);
  motor1.drive(-temp);
  motor2.drive(-temp);
}
void back(Motor &motor1, Motor &motor2) { back(motor1, motor2, DEFAULTSPEED); }
void left(Motor &motor1, Motor &motor2, int speed)
{
  int temp = abs(speed);
  motor1.drive(-temp);
  motor2.drive(temp);
}
void left(Motor &motor1, Motor &motor2) { left(motor1, motor2, DEFAULTSPEED); }
void right(Motor &motor1, Motor &motor2, int speed)
{
  int temp = abs(speed);
  motor1.drive(temp);
  motor2.drive(-temp);
}
void right(Motor &motor1, Motor &motor2) { right(motor1, motor2, DEFAULTSPEED); }
void brake(Motor &motor1, Motor &motor2)
{
  motor1.brake();
  motor2.brake();
}
