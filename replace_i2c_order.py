import os
import re

content = open('src/custom_motor_driver.cpp', 'r', encoding='utf-8', errors='ignore').read()

new_code = """
        if (pcf == &FMCpcf8574)
        {
            if (stateIn1 == HIGH)
                fmcPcfShadow |= (1 << In1);
            else
                fmcPcfShadow &= ~(1 << In1);

            if (stateIn2 == HIGH)
                fmcPcfShadow |= (1 << In2);
            else
                fmcPcfShadow &= ~(1 << In2);

            // ?? MASCARA ATOMICA DE ENTRADAS: Forzar los pines 0, 1, 2 y 3 siempre a 1 (HIGH)
            fmcPcfShadow |= 0x0F;
            
            // ESCRIBIR I2C PRIMERO (DIRECCION ANTES QUE POTENCIA)
            Wire.beginTransmission(0x20);
            Wire.write(fmcPcfShadow);
            Wire.endTransmission();
        }
        else
        {
            if (stateIn1 == HIGH)
                bmcPcfShadow |= (1 << In1);
            else
                bmcPcfShadow &= ~(1 << In1);

            if (stateIn2 == HIGH)
                bmcPcfShadow |= (1 << In2);
            else
                bmcPcfShadow &= ~(1 << In2);

            // ?? Respetar pin 5 del BMC
            bmcPcfShadow |= (1 << 5);
            
            // ESCRIBIR I2C PRIMERO (DIRECCION ANTES QUE POTENCIA)
            Wire.beginTransmission(0x24);
            Wire.write(bmcPcfShadow);
            Wire.endTransmission();
        }

        uint16_t PWM = (speed >= 4095) ? 4095 : speed;
"""

content = re.sub(r'if \(pcf == &FMCpcf8574\)\s*\{.*?uint16_t PWM = \(speed >= 4095\) \? 4095 : speed;', new_code.strip(), content, flags=re.DOTALL)

open('src/custom_motor_driver.cpp', 'w', encoding='utf-8').write(content)
print("Done.")
