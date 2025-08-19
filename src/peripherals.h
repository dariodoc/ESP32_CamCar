#ifndef PERIPHERALS_H
#define PERIPHERALS_H

void setupPeripherals();
void ledIndicator(int blinkTimes, int delayTimeMS);
void ledIndicator(int state);
void leftBackLed(int state);
void rightBackLed(int state);
void playMelody(void *parameters);
void obstacleAvoidanceMode(void *parameters);

#endif // PERIPHERALS_H