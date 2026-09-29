import os
import re

content = open('src/custom_motor_driver.cpp', 'r', encoding='utf-8', errors='ignore').read()

new_code = """
void Motor::setMotorState(int stateIn1, int stateIn2, int speed)
{
    // Bypass: Si el estado no ha cambiado, no enviamos nada por I2C
    if (stateIn1 == lastStateIn1 && stateIn2 == lastStateIn2 && speed == lastSpeed)
    {
        return;
    }

    if (lockI2C(20))
    {
        lastStateIn1 = stateIn1;
        lastStateIn2 = stateIn2;
        lastSpeed = speed;

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
        }

        uint16_t PWM = (speed >= 4095) ? 4095 : speed;
        // Desfase perfecto basado en el pin del PWM (0, 1, 2 o 3)
        // Separa los 4 motores por 1024 ticks exactos para evitar superposicion de picos
        uint16_t startTick = (pwm * 1024) % 4096; 

        if (PWM >= 4095) {
            pca->setPWM(pwm, 4096, 0); // FULLY ON
        } else if (PWM == 0) {
            pca->setPWM(pwm, 0, 4096); // FULLY OFF
        } else {
            pca->setPWM(pwm, startTick, (startTick + PWM) % 4096);
        }

        unlockI2C();
    }
}
"""

content = re.sub(r'void Motor::setMotorState\(int stateIn1, int stateIn2, int speed\).*?unlockI2C\(\);\s*\}', new_code.strip(), content, flags=re.DOTALL)

open('src/custom_motor_driver.cpp', 'w', encoding='utf-8').write(content)
print("Done.")
