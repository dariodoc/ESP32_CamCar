#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

void initI2CManager();
bool lockI2C(TickType_t timeoutMs = 20);
void unlockI2C();

#endif // I2C_MANAGER_H