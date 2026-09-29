import os

content = open('src/motor_control.cpp', 'r', encoding='utf-8').read()

# Replace mapMotorValue's MIN_PWM
content = content.replace('const int MIN_PWM = 800, MAX_PWM = 4095;', 'const int MIN_PWM = 819, MAX_PWM = 4095;')

# Replace motorSlewTask
start_idx = content.find('            if (currentFL < targetFL) { currentFL = min(currentFL + SLEW_STEP, targetFL); changed = true; }')
end_idx = content.find('            if (changed)')

slew_logic = '''            auto applyRamp = [](int &current, int target) {
                const int SLEW_STEP = 300;
                const int MIN_PWM = 819;
                
                if (current < target) {
                    if (current == 0) current = MIN_PWM;
                    else current = min(current + SLEW_STEP, target);
                }
                else if (current > target) {
                    if (current == 0) current = -MIN_PWM;
                    else current = max(current - SLEW_STEP, target);
                }
                
                // Cut-off a 0 si caemos en la zona muerta
                if (abs(current) < MIN_PWM) current = 0;
            };

            if (currentFL != targetFL) { applyRamp(currentFL, targetFL); changed = true; }
            if (currentBL != targetBL) { applyRamp(currentBL, targetBL); changed = true; }
            if (currentFR != targetFR) { applyRamp(currentFR, targetFR); changed = true; }
            if (currentBR != targetBR) { applyRamp(currentBR, targetBR); changed = true; }

'''

new_content = content[:start_idx] + slew_logic + content[end_idx:]

open('src/motor_control.cpp', 'w', encoding='utf-8').write(new_content)
