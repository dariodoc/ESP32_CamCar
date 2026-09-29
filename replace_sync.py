import os

content = open('src/motor_control.cpp', 'r', encoding='utf-8').read()

# Replace motorSlewTask to call syncMotorsI2C at the end
start_idx = content.find('void motorSlewTask(void *pvParameters)')
end_idx = content.find('void setupMotors()')

slew_content = content[start_idx:end_idx]

slew_content = slew_content.replace('                else\n                {\n                    setStandbyPin(true);\n                    motorFL.drive(currentFL);\n                    motorBL.drive(currentBL);\n                    motorFR.drive(currentFR);\n                    motorBR.drive(currentBR);\n                    leftRearLed(LOW);\n                    rightRearLed(LOW);\n                }', '                else\n                {\n                    setStandbyPin(true);\n                    motorFL.drive(currentFL);\n                    motorBL.drive(currentBL);\n                    motorFR.drive(currentFR);\n                    motorBR.drive(currentBR);\n                    leftRearLed(LOW);\n                    rightRearLed(LOW);\n                }\n                syncMotorsI2C();')
slew_content = slew_content.replace('            immediateBrake = false;\n        }', '            syncMotorsI2C();\n            immediateBrake = false;\n        }')

new_content = content[:start_idx] + slew_content + content[end_idx:]

open('src/motor_control.cpp', 'w', encoding='utf-8').write(new_content)
