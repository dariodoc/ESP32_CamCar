import os

content = open('src/custom_motor_driver.cpp', 'r', encoding='utf-8', errors='ignore').read()
content = content.replace('uint16_t startTick = (PWM * 256) % 4096;', 'uint16_t startTick = (PWM * 1024) % 4096;')
open('src/custom_motor_driver.cpp', 'w', encoding='utf-8').write(content)

content2 = open('src/motor_control.cpp', 'r', encoding='utf-8', errors='ignore').read()
content2 = content2.replace('xTaskCreatePinnedToCore(motorSlewTask, "MotorTask", 2048, NULL, 1, &motorTaskHandle, 1);', 'xTaskCreatePinnedToCore(motorSlewTask, "MotorTask", 4096, NULL, 1, &motorTaskHandle, 1);')
open('src/motor_control.cpp', 'w', encoding='utf-8').write(content2)
