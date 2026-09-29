import os

content = open('src/motor_control.cpp', 'r', encoding='utf-8', errors='ignore').read()
content = content.replace('const int MIN_PWM = 819, MAX_PWM = 4095;', 'const int MIN_PWM = 0, MAX_PWM = 4095;')
open('src/motor_control.cpp', 'w', encoding='utf-8').write(content)
