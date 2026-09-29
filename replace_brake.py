import os
import re

content = open('src/motor_control.cpp', 'r', encoding='utf-8', errors='ignore').read()

new_code = """
        if (immediateBrake || immediateStop)
        {
            currentFL = 0; currentBL = 0; currentFR = 0; currentBR = 0;
            targetFL = 0; targetBL = 0; targetFR = 0; targetBR = 0;
            
            if (immediateStop) {
                setStandbyPin(false); // Pin Standby LOW: Desconecta fisicamente los motores (Coast)
                motorFL.drive(0);
                motorBL.drive(0);
                motorFR.drive(0);
                motorBR.drive(0);
            } else {
                // Freno electrónico activo (Short-circuit brake)
                setStandbyPin(true);
                motorFL.brake();
                motorBL.brake();
                motorFR.brake();
                motorBR.brake();
                syncMotorsI2C();
                
                // Aplicar freno solo por 150ms para evitar sobrecorriente (Brownout)
                vTaskDelay(pdMS_TO_TICKS(150));
                
                // Luego pasar a estado libre (Coast)
                setStandbyPin(false);
                motorFL.drive(0);
                motorBL.drive(0);
                motorFR.drive(0);
                motorBR.drive(0);
            }
            leftRearLed(HIGH);
            rightRearLed(HIGH);
            
            syncMotorsI2C();
            immediateBrake = false;
            immediateStop = false;
        }
"""

content = re.sub(r'if \(immediateBrake \|\| immediateStop\)\s*\{.*?immediateStop = false;\s*\}', new_code.strip(), content, flags=re.DOTALL)

open('src/motor_control.cpp', 'w', encoding='utf-8').write(content)
print("Done.")
