#include "config.h"
#include "wifi_server.h"
#include "camera_setup.h" // 👈 Proporciona la declaración de startCameraServer()
#include "motor_control.h"
#include "peripherals.h"
#include <WiFi.h>
#include "AsyncTCP.h"
#define WS_MAX_QUEUED_MESSAGES 1
#include "ESPAsyncWebServer.h"
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include "esp_bt.h"
#include <Update.h>

AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");

int carInputClientId = 0;
// unsigned long lastCommandReceived = 0;
volatile int targetDirection = STOP;

String ssid, pass, ip, gateway;
const char *ssidPath = "/ssid.txt", *passPath = "/pass.txt", *ipPath = "/ip.txt", *gatewayPath = "/gateway.txt";

void cleanupWSClients()
{
    wsCarInput.cleanupClients();
}

// Optimización directa del manejador WebSocket
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
        setCarMotorsStandby(true);
    }
    else if (type == WS_EVT_DISCONNECT)
    {
        if (client->id() == carInputClientId)
            carInputClientId = 0;
        targetDirection = STOP;
        digitalWrite(lightPin, LOW);
        enableLight = false;
        targetPan = 75;
        targetTilt = 90;
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
        setCarMotorsStandby(false);
    }
    else if (type == WS_EVT_DATA && len > 0)
    {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (!(info->final && info->index == 0 && info->len == len))
            return;

        // Lectura rápida sin asignaciones dinámicas
        const uint8_t cmd = data[0];
        const uint8_t val = (len > 1) ? data[1] : 0;

        switch (cmd)
        {
        case 'M':
            targetDirection = val;
            break;
        case 'S':
            motorSpeed = map(val, 1, 5, 200, 255);
            break;
        case 'L':
            enableLight = !enableLight;
            digitalWrite(lightPin, enableLight);
            break;
        case 'P':
            targetPan = val;
            break;
        case 'T':
            targetTilt = val;
            break;
        case 'C':
            targetPan = 75;
            targetTilt = 90;
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
                targetDirection = STOP;
            }
            break;
        }
    }
}

String readFile(fs::FS &fs, const char *path)
{
    File file = fs.open(path);
    if (!file || file.isDirectory())
        return String();

    String fileContent = file.readStringUntil('\n');
    fileContent.trim(); // 👈 Elimina \r, \n y espacios
    file.close();
    return fileContent;
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
    File file = fs.open(path, FILE_WRITE);
    if (file)
    {
        file.print(message);
        file.close();
    }
}

void initWiFi()
{
    // 1. Desactivar el ahorro de energía del Wi-Fi (Latencia ultra baja)
    WiFi.setSleep(false);

    // 2. Maximizar la potencia de transmisión del módulo Wi-Fi (20 dBm = 100 mW)
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    ssid = readFile(SPIFFS, ssidPath);
    pass = readFile(SPIFFS, passPath);

    // Limpia espacios en blanco o saltos de línea leídos del archivo txt
    ssid.trim();
    pass.trim();

    if (!ssid.isEmpty())
    {
        WiFi.mode(WIFI_STA); // Evita WIFI_AP_STA inicial si ya hay credenciales
        WiFi.begin(ssid.c_str(), pass.c_str());

        unsigned long startTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startTime < 8000)
        {
            ledIndicator(1, 250);
        }

        if (WiFi.status() == WL_CONNECTED)
        {
            if (MDNS.begin("cameracar"))
                MDNS.addService("http", "tcp", 80);
            ledIndicator(5, 50);
        }
        else
        {
            WiFi.mode(WIFI_AP);
            WiFi.softAP("ESP-CAMERA-CAR", "carbondioxide");
        }
    }
    else
    {
        WiFi.mode(WIFI_AP);
        WiFi.softAP("ESP-CAMERA-CAR", "carbondioxide");
    }

    // Configuración de rutas estáticas en Flash (Usando Gzip si es posible)
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html", "text/html"); });

    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/style.css", "text/css"); });

    wsCarInput.onEvent(onCarInputWebSocketEvent);
    server.addHandler(&wsCarInput);

    server.begin();

    // Apagar Bluetooth libera RAM e impide interferencias en la antena de 2.4 GHz
    btStop();
    esp_bt_controller_disable();
}