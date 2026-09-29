import os

content = open('src/motor_control.cpp', 'r', encoding='utf-8').read()

# Add immediateStop flag
content = content.replace('static bool immediateBrake = false;', 'static bool immediateBrake = false;\nstatic bool immediateStop = false;')

# Replace motorSlewTask's immediateBrake logic
start_brake = content.find('        if (immediateBrake)')
end_brake = content.find('        else\n        {\n            bool changed = false;')

brake_logic = '''        if (immediateBrake || immediateStop)
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
                setStandbyPin(true); // Pin Standby HIGH: Habilita el puente H para freno electrnico
                motorFL.brake();
                motorBL.brake();
                motorFR.brake();
                motorBR.brake();
            }
            leftRearLed(HIGH);
            rightRearLed(HIGH);
            
            syncMotorsI2C();
            immediateBrake = false;
            immediateStop = false;
        }
'''

content = content[:start_brake] + brake_logic + content[end_brake:]

# Update stopAllMotors()
start_stop = content.find('void stopAllMotors()\n{')
end_stop = content.find('void brakeAllMotors()')

stop_logic = '''void stopAllMotors()
{
    enableObstacleAvoidance = false;
    enableIROnlyMode = false;
    immediateStop = true;
}

'''
content = content[:start_stop] + stop_logic + content[end_stop:]

open('src/motor_control.cpp', 'w', encoding='utf-8').write(content)
