#include "config.h"
// 🚀 Limpiar definición previa si existe y aplicar la tuya ANTES de incluir AsyncWebServer
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

AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");

int carInputClientId = 0;
volatile int targetDirection = STOP;
volatile unsigned long lastCommandTime = 0;

// Parámetros de la interfaz web
const char *PARAM_INPUT_1 = "ssid";
const char *PARAM_INPUT_2 = "pass";
const char *PARAM_INPUT_3 = "ip";
const char *PARAM_INPUT_4 = "gateway";

String ssid, pass, ip, gateway;
const char *ssidPath = "/ssid.txt";
const char *passPath = "/pass.txt";
const char *ipPath = "/ip.txt";
const char *gatewayPath = "/gateway.txt";

void cleanupWSClients()
{
    wsCarInput.cleanupClients();
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
void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT)
    {
        // 🚀 Si entra una nueva conexión (por recargar F5 o reconexión),
        // cerramos el cliente viejo si existía.
        if (carInputClientId != 0 && carInputClientId != client->id())
        {
            AsyncWebSocketClient *oldClient = server->client(carInputClientId);
            if (oldClient)
                oldClient->close();
        }

        // Registramos el nuevo ID de cliente y REACTIVAMOS los motores obligatoriamente
        carInputClientId = client->id();
        targetDirection = STOP;
        setCarMotorsStandby(true); // 🚀 Vuelve a despertar el chip TB6612 (STBY = HIGH)
    }
    else if (type == WS_EVT_DISCONNECT)
    {
        // 🚀 CRÍTICO: Solo apagamos los motores si el cliente que se desconecta
        // es el cliente activo ACTUAL, no una sesión vieja que se cerró tarde.
        if (client->id() == carInputClientId)
        {
            carInputClientId = 0;
            targetDirection = STOP;
            enableLaser = false;
            turnLaserOn(enableLaser);

            // Detener y centrar servos
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

            // Ponemos los motores en standby solo si ya no hay nadie conectado
            setCarMotorsStandby(false);
        }
    }
    else if (type == WS_EVT_DATA && len > 0)
    {
        // Aseguramos que solo el cliente activo pueda mandar órdenes a los motores
        if (client->id() != carInputClientId)
            return;

        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (!(info->final && info->index == 0 && info->len == len))
            return;

        const uint8_t cmd = data[0];
        const uint8_t val = (len > 1) ? data[1] : 0;

        lastCommandTime = millis();

        // 🚀 SEGURIDAD EXTRA: Asegurar que los motores estén activos al recibir un comando de movimiento
        setCarMotorsStandby(true);

        switch (cmd)
        {
        case 'K': // Heartbeat
            break;
        case 'M':
            targetDirection = val;
            break;
        case 'S':
            motorSpeed = map(val, 1, 5, 200, 255);
            break;
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
    fileContent.trim();
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
    if (!SPIFFS.begin(true))
    {
#ifdef DEBUG
        Serial.println("Error mounting SPIFFS");
#endif
    }

    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    ssid = readFile(SPIFFS, ssidPath);
    pass = readFile(SPIFFS, passPath);
    ip = readFile(SPIFFS, ipPath);
    gateway = readFile(SPIFFS, gatewayPath);

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

    // Rutas del servidor Web
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

    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
              {
        int params = request->params();
        for (int i = 0; i < params; i++) {
            const AsyncWebParameter* p = request->getParam(i);
            if (p->isPost()) {
                if (p->name() == PARAM_INPUT_1) { writeFile(SPIFFS, ssidPath, p->value().c_str()); }
                if (p->name() == PARAM_INPUT_2) { writeFile(SPIFFS, passPath, p->value().c_str()); }
                if (p->name() == PARAM_INPUT_3) { writeFile(SPIFFS, ipPath, p->value().c_str()); }
                if (p->name() == PARAM_INPUT_4) { writeFile(SPIFFS, gatewayPath, p->value().c_str()); }
            }
        }
        request->send(200, "text/plain", "Credenciales guardadas. El ESP32 se reiniciara...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP.restart(); });

    server.serveStatic("/", SPIFFS, "/");

    wsCarInput.onEvent(onCarInputWebSocketEvent);
    server.addHandler(&wsCarInput);

    initCameraWebSocket(&server);

    server.begin();

    btStop();
    esp_bt_controller_disable();
}