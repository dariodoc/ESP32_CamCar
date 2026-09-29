import os

content = open('src/custom_motor_driver.cpp', 'r', encoding='utf-8').read()
start_idx = content.find('void Motor::setMotorState(int stateIn1, int stateIn2, int speed)')
end_idx = content.find('void Motor::fwd(int speed)')

new_content = content[:start_idx] + '''void syncMotorsI2C()
{
    if (lockI2C(20))
    {
        // 1. Enviar estado de motores delanteros (FMC)
        Wire.beginTransmission(0x20);
        Wire.write(fmcPcfShadow);
        Wire.endTransmission();

        // 2. Enviar estado de motores traseros y STBY (BMC)
        Wire.beginTransmission(0x24);
        Wire.write(bmcPcfShadow);
        Wire.endTransmission();

        unlockI2C();
    }
}

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
        else if (pcf == &BMCpcf8574)
        {
            if (stateIn1 == HIGH)
                bmcPcfShadow |= (1 << In1);
            else
                bmcPcfShadow &= ~(1 << In1);

            if (stateIn2 == HIGH)
                bmcPcfShadow |= (1 << In2);
            else
                bmcPcfShadow &= ~(1 << In2);
        }

        // ?? ANTI-BROWNOUT MAGIA: Desfasar el pulso PWM de cada motor!
        uint16_t startTick = (PWM * 256) % 4096;
        uint16_t endTick = (startTick + speed) % 4096;

        if (speed == 4095) { // 100% duty cycle especial
            pca->setPWM(PWM, 4096, 0); 
        } else if (speed == 0) { // 0% duty cycle especial
            pca->setPWM(PWM, 0, 4096);
        } else {
            pca->setPWM(PWM, startTick, endTick);
        }
        
        unlockI2C();
    }
}

''' + content[end_idx:]

# Remove transmission from setStandbyPin
start_stby = new_content.find('void setStandbyPin(bool enable)')
end_stby = new_content.find('Motor::Motor(int In1pin')

stby_content = new_content[start_stby:end_stby]
stby_content = stby_content.replace('        Wire.beginTransmission(0x24);\n', '')
stby_content = stby_content.replace('        Wire.write(bmcPcfShadow);\n', '')
stby_content = stby_content.replace('        Wire.endTransmission();\n', '')

new_content = new_content[:start_stby] + stby_content + new_content[end_stby:]

open('src/custom_motor_driver.cpp', 'w', encoding='utf-8').write(new_content)
