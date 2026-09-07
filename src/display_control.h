#ifndef DISPLAY_CONTROL_H
#define DISPLAY_CONTROL_H

#include <Arduino.h>

// Banderas de estado para enviar a la pantalla
enum DisplayState
{
    DISPLAY_BOOT,
    DISPLAY_CONNECTING_WIFI,
    DISPLAY_PORTAL_ACTIVE,
    DISPLAY_CONNECTED,
    DISPLAY_OBSTACLE_ALERT,
    DISPLAY_CLEAR_ALERT
};

// Estructura de mensaje para la cola de FreeRTOS
struct DisplayMessage
{
    DisplayState state;
    char textExtra[32]; // Para enviar cadenas dinámicas (ej: IP o SSID)
};

void initDisplayTask();
void updateDisplayState(DisplayState state, const char *extraText = "");

#endif // DISPLAY_CONTROL_H