#include "config.h"
#ifdef WS_MAX_QUEUED_MESSAGES
#undef WS_MAX_QUEUED_MESSAGES
#endif
#define WS_MAX_QUEUED_MESSAGES 1

#include "wifi_server.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include <WiFi.h>
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include "esp_bt.h"
#include <Update.h>
#include <Preferences.h> // 👈 Librería nativa NVS

extern AsyncWebSocket wsCamera;

AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");

int carInputClientId = 0;
// volatile int targetDirection = STOP;
volatile unsigned long lastCommandTime = 0;

// Instancia de Preferences para NVS
Preferences preferences;

// Parámetros de la interfaz web
const char *PARAM_INPUT_1 = "ssid";
const char *PARAM_INPUT_2 = "pass";
const char *PARAM_INPUT_3 = "ip";
const char *PARAM_INPUT_4 = "gateway";

String ssid, pass, ip, gateway;

void cleanupWSClients()
{
    wsCarInput.cleanupClients();
    wsCamera.cleanupClients(); // 👈 Agrega esta línea para liberar memoria de sockets de cámara
}

// Escaneo BSSID para conectar al nodo con mejor señal en red mesh
void scanAndConnectToBestAP(const char *targetSSID, const char *password)
{
    int8_t bestRSSI = -100;
    uint8_t bestBSSID[6];
    int bestChannel = 0;
    bool found = false;

#ifdef DEBUG
    Serial.println(" Scanning networks...");
#endif

    int n = WiFi.scanNetworks(false, true);
    if (n > 0)
    {
        for (int i = 0; i < n; ++i)
        {
            if (WiFi.SSID(i) == targetSSID)
            {
#ifdef DEBUG
                Serial.printf(" Found: %s | Ch: %d | RSSI: %d dBm\n", WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i));
#endif
                if (WiFi.RSSI(i) > bestRSSI)
                {
                    bestRSSI = WiFi.RSSI(i);
                    memcpy(bestBSSID, WiFi.BSSID(i), 6);
                    bestChannel = WiFi.channel(i);
                    found = true;
                }
            }
        }
    }

    if (!found)
    {
#ifdef DEBUG
        Serial.println(" Desired SSID not found. Trying standard connection...");
#endif
        WiFi.begin(targetSSID, password);
    }
    else
    {
#ifdef DEBUG
        Serial.printf(" Connecting to best AP node (RSSI: %d dBm) on Ch %d\n", bestRSSI, bestChannel);
#endif
        WiFi.begin(targetSSID, password, bestChannel, bestBSSID);
    }

    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000)
    {
        ledIndicator(1, 250);
    }

    WiFi.scanDelete();
}

// Manejador del WebSocket
// Estructura binaria de 8 bytes
struct __attribute__((__packed__)) DifferentialInput
{
    float x;
    float y;
};

extern volatile float joystickX;
extern volatile float joystickY;

void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT)
    {
        if (carInputClientId != 0 && carInputClientId != client->id())
        {
            AsyncWebSocketClient *oldClient = server->client(carInputClientId);
            if (oldClient)
                oldClient->close();
        }

        carInputClientId = client->id();
        joystickX = 0.0f;
        joystickY = 0.0f;
    }
    else if (type == WS_EVT_DISCONNECT)
    {
        if (client->id() == carInputClientId)
        {
            carInputClientId = 0;
            joystickX = 0.0f;
            joystickY = 0.0f;

            // 1. Detener periféricos y frenar hardware
            enableLaser = false;
            turnLaserOn(enableLaser);
            centerServos();

            if (melodyOn)
            {
                melodyOn = false;
                ledcWriteTone(buzzerChannel, 0);
            }
            if (enableObstacleAvoidance)
            {
                enableObstacleAvoidance = false;
                obstacleFound = false;
            }

            cleanupWSClients(); // 👈 Limpieza explícita de clientes WebSocket para liberar memoria y sockets de red
        }
    }
    else if (type == WS_EVT_DATA && len > 0)
    {
        if (client->id() != carInputClientId)
            return;

        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (!(info->final && info->index == 0 && info->len == len))
            return;

        lastCommandTime = millis();

        // 🚀 SI RECIBE 8 BYTES: Es comando binario de Joystick
        if (len == sizeof(DifferentialInput))
        {
            DifferentialInput *input = (DifferentialInput *)data;
            joystickX = input->x;
            joystickY = input->y;
        }
        // 🚀 SI RECIBE COMANDOS DE TEXTO/ACCESORIOS (L, P, T, C, H, O)
        else
        {
            const uint8_t cmd = data[0];
            const uint8_t val = (len > 1) ? data[1] : 0;

            switch (cmd)
            {
            case 'L':
                enableLaser = !enableLaser;
                turnLaserOn(enableLaser);
                break;
            case 'P':
                panDirection = val;
                break;
            case 'T':
                tiltDirection = val;
                break;
            case 'C':
                centerServos();
                break;
            case 'H':
                melodyOn = !melodyOn;
                if (melodyOn)
                    xTaskNotifyGive(playMelodyTaskHandle);
                else
                    ledcWriteTone(buzzerChannel, 0);
                break;
            case 'O':
                enableObstacleAvoidance = !enableObstacleAvoidance;
                if (enableObstacleAvoidance)
                    xTaskNotifyGive(obstacleAvoidanceModeTaskHandle);
                else
                {
                    obstacleFound = false;
                    joystickX = 0.0f;
                    joystickY = 0.0f;
                }
                break;
            }
        }
    }
}

void initArduinoOTA()
{
    // Configuración obligatoria de ArduinoOTA
    ArduinoOTA.onStart([]()
                       { String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem"; });
    ArduinoOTA.begin(); // 👈 Sin esto el ESP32 no escuchará peticiones en el puerto 3232
}

void initWiFi()
{
    // Mantenemos SPIFFS activo únicamente para la interfaz web (.html, .css)
    if (!SPIFFS.begin(true))
    {
#ifdef DEBUG
        Serial.println("Error mounting SPIFFS");
#endif
    }

    WiFi.setSleep(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    // 🚀 LECTURA DESDE NVS (Preferences) EN LUGAR DE SPIFFS
    preferences.begin("wifi-config", true); // Abre espacio "wifi-config" en modo solo lectura
    ssid = preferences.getString("ssid", "");
    pass = preferences.getString("pass", "");
    ip = preferences.getString("ip", "");
    gateway = preferences.getString("gateway", "");
    preferences.end();

    WiFi.mode(WIFI_AP_STA);

    if (!ssid.isEmpty())
    {
        if (!ip.isEmpty() && !gateway.isEmpty())
        {
            IPAddress localIP, localGateway, subnet(255, 255, 255, 0);
            localIP.fromString(ip);
            localGateway.fromString(gateway);
            WiFi.config(localIP, localGateway, subnet);
        }

        scanAndConnectToBestAP(ssid.c_str(), pass.c_str());
        WiFi.setAutoReconnect(true);

        if (WiFi.status() == WL_CONNECTED)
        {
            WiFi.mode(WIFI_STA);
            if (MDNS.begin("cameracar"))
                MDNS.addService("http", "tcp", 80);
            ledIndicator(5, 50);
        }
        else
        {
            WiFi.softAP("ESP-CAMERA-CAR", "carbondioxide");
        }
    }
    else
    {
        WiFi.softAP("ESP-CAMERA-CAR", "carbondioxide");
    }

    // Rutas del servidor Web (Archivos estáticos desde SPIFFS)
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/index.html", "text/html");
        response->addHeader("Cache-Control", "max-age=86400");
        request->send(response); });

    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/style.css", "text/css");
        response->addHeader("Cache-Control", "max-age=86400");
        request->send(response); });

    server.on("/wifimanager", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/wifimanager.html", "text/html");
        response->addHeader("Cache-Control", "max-age=86400");
        request->send(response); });

    server.on("/wifimanager.css", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/wifimanager.css", "text/css");
        response->addHeader("Cache-Control", "max-age=86400");
        request->send(response); });

    // 🚀 GUARDADO EN NVS AL RECIBIR FORMULARIO POST
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
              {
        preferences.begin("wifi-config", false); // Abre en modo lectura/escritura

        int params = request->params();
        for (int i = 0; i < params; i++) {
            const AsyncWebParameter* p = request->getParam(i);
            if (p->isPost()) {
                if (p->name() == PARAM_INPUT_1) { preferences.putString("ssid", p->value().c_str()); }
                if (p->name() == PARAM_INPUT_2) { preferences.putString("pass", p->value().c_str()); }
                if (p->name() == PARAM_INPUT_3) { preferences.putString("ip", p->value().c_str()); }
                if (p->name() == PARAM_INPUT_4) { preferences.putString("gateway", p->value().c_str()); }
            }
        }
        
        preferences.end();

        request->send(200, "text/plain", "Credenciales guardadas en NVS. El ESP32 se reiniciara...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP.restart(); });

    server.serveStatic("/", SPIFFS, "/");

    wsCarInput.onEvent(onCarInputWebSocketEvent);
    server.addHandler(&wsCarInput);

    initCameraWebSocket(&server);

    server.begin();
    initArduinoOTA();

    btStop();
    esp_bt_controller_disable();
}