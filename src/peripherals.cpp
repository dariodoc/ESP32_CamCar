#include "config.h"
#include "peripherals.h"
#include "motor_control.h" // Para llamar a moveCar(STOP)
#include "PCF8574.h"

// Definición de objetos y variables de periféricos
PCF8574 pcf8574(0x20, 14, 15);
Servo panServo;
Servo tiltServo;
bool enableLight = false;
bool melodyOn = false;
bool enableObstacleAvoidance = false;
bool obstacleFound = false;
TaskHandle_t playMelodyTask;
TaskHandle_t obstacleAvoidanceModeTask;


void setupPeripherals() {
    pinMode(builtinLedPin, OUTPUT);
    digitalWrite(builtinLedPin, HIGH); // LED OFF

    ledcDetachPin(buzzerPin); // Apagar buzzer por si acaso

    panServo.attach(panPin);
    tiltServo.attach(tiltPin);
    panServo.write(panCenter);
    tiltServo.write(tiltCenter);

    pcf8574.pinMode(P5, INPUT);  // Sensor de distancia
    pcf8574.pinMode(P6, OUTPUT); // LED trasero izquierdo
    pcf8574.pinMode(P7, OUTPUT); // LED trasero derecho
    pinMode(lightPin, OUTPUT);   // Luces frontales

    if (!pcf8574.begin()) {
        #ifdef DEBUG
        Serial.println("PCF8574 FAILED");
        #endif
    }
    ledIndicator(3, 250); // Indicar que el coche ha arrancado
}

void ledIndicator(int blinkTimes, int delayTimeMS) {
    for (int i = 0; i < blinkTimes; i++) {
        digitalWrite(builtinLedPin, LOW); // LED ON
        toneToPlay(buzzerPin, buzzerChannel, NOTE_G5, delayTimeMS);
        digitalWrite(builtinLedPin, HIGH); // LED OFF
        vTaskDelay(pdMS_TO_TICKS(delayTimeMS));
    }
}

void ledIndicator(int state) {
    digitalWrite(builtinLedPin, !state); // LOW enciende el LED
}

void leftBackLed(int state) {
    pcf8574.digitalWrite(P6, !state); // LOW enciende el LED
}

void rightBackLed(int state) {
    pcf8574.digitalWrite(P7, !state); // LOW enciende el LED
}

void playMelody(void *parameters) {
    #ifdef DEBUG
    Serial.printf("playMelody() running on core: %d\n", xPortGetCoreID());
    #endif
    gameOfThrones(buzzerPin, buzzerChannel);
    melodyOn = false;
    ledcDetachPin(buzzerPin);
    vTaskDelete(playMelodyTask);
}

void obstacleAvoidanceMode(void *parameters) {
    #ifdef DEBUG
    Serial.printf("obstacleAvoidanceMode() running on core: %d\n", xPortGetCoreID());
    #endif
    for (;;) {
        int detect = pcf8574.digitalRead(P5);
        if (detect == LOW && (currentDirection == FORWARD || currentDirection == FORWARDLEFT || currentDirection == FORWARDRIGHT)) {
            obstacleFound = true;
            moveCar(STOP);
            toneToPlay(buzzerPin, buzzerChannel, NOTE_G5, 200);
        } else {
            obstacleFound = false;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}