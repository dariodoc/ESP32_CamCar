import os

content = open('src/peripherals.cpp', 'r', encoding='utf-8', errors='ignore').read()

# Add Wire.setTimeOut(20) after Wire.begin
target = 'Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);'
replacement = 'Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);\n    Wire.setTimeOut(20); // Prevent I2C deadlock from motor EMI\n    Wire.setClock(100000); // Lower I2C speed to 100kHz for robustness'

content = content.replace(target, replacement)

open('src/peripherals.cpp', 'w', encoding='utf-8').write(content)
