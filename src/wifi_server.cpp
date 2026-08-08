#include "config.h"
#include "wifi_server.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include "esp_bt.h"
#include <Update.h>
#include "SparkFun_TB6612.h"

extern void startCameraServer();

// --- Definición de variables y objetos del servidor ---
AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");
int carInputClientId = 0;

// Variables de WiFi Manager
String ssid, pass, ip, gateway;
const char *ssidPath = "/ssid.txt", *passPath = "/pass.txt", *ipPath = "/ip.txt", *gatewayPath = "/gateway.txt";
const char *PARAM_INPUT_1 = "ssid", *PARAM_INPUT_2 = "pass", *PARAM_INPUT_3 = "ip", *PARAM_INPUT_4 = "gateway";

// --- Prototipos de funciones internas ---
void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
String readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void scanAndConnectToBestAP(const char *targetSSID, const char *password);

volatile int targetDirection = 0;

void sendTelemetryTask(void *parameters)
{
    char telemetryJson[128];

    for (;;)
    {
        if (wsCarInput.count() > 0)
        {
            snprintf(telemetryJson, sizeof(telemetryJson),
                     "{\"rssi\":%d,\"obstacle\":%s}",
                     WiFi.RSSI(),
                     obstacleFound ? "true" : "false");

            wsCarInput.textAll(telemetryJson);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void cleanupWSClients()
{
    wsCarInput.cleanupClients();
}

void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT)
    {
        if (carInputClientId != 0 && carInputClientId != client->id())
        {
            AsyncWebSocketClient *oldClient = server->client(carInputClientId);
            if (oldClient)
            {
                oldClient->close();
            }
        }
        carInputClientId = client->id();
#ifdef DEBUG
        Serial.printf("WS Control client #%u connected\n", client->id());
#endif
        setMotorsStandby(true);
    }
    else if (type == WS_EVT_DISCONNECT)
    {
        if (client->id() == carInputClientId)
        {
            carInputClientId = 0;
        }
#ifdef DEBUG
        Serial.printf("WS Control client #%u disconnected\n", client->id());
#endif
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
        setMotorsStandby(false);
    }
    else if (type == WS_EVT_DATA && len > 0)
    {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (!(info->final && info->index == 0 && info->len == len))
        {
            return;
        }

        static unsigned long lastWsTime = 0;
        unsigned long currentWsTime = millis();

        if (currentWsTime - lastWsTime < 50)
        {
            return;
        }
        lastWsTime = currentWsTime;

        char command = data[0];
        int value = (len > 1) ? data[1] : 0;
        switch (command)
        {
        case 'M':
            targetDirection = value;
            break;
        case 'S':
            motorSpeed = map(value, 1, 5, 200, 255);
            break;
        case 'L':
            enableLight = !enableLight;
            digitalWrite(lightPin, enableLight);
            break;
        case 'P':
            targetPan = value;
            break;
        case 'T':
            targetTilt = value;
            break;
        case 'C':
            targetPan = 75;
            targetTilt = 90;
            break;
        case 'H':
            melodyOn = !melodyOn;
            if (melodyOn)
            {
                xTaskNotifyGive(playMelodyTask);
            }
            else
            {
                ledcWriteTone(buzzerChannel, 0);
            }
            break;
        case 'O':
            enableObstacleAvoidance = !enableObstacleAvoidance;
            if (enableObstacleAvoidance)
            {
                xTaskNotifyGive(obstacleAvoidanceModeTask);
            }
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
    String fileContent;
    if (file.available())
        fileContent = file.readStringUntil('\n');
    return fileContent;
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
    File file = fs.open(path, FILE_WRITE);
    if (file)
        file.print(message);
}

void scanAndConnectToBestAP(const char *targetSSID, const char *password)
{
    int8_t bestRSSI = -100;
    uint8_t bestBSSID[6];
    int bestChannel = 0;
    bool found = false;

    int n = WiFi.scanNetworks(false, true);
    if (n == 0) return;

    for (int i = 0; i < n; ++i)
    {
        if (WiFi.SSID(i) == targetSSID)
        {
            if (WiFi.RSSI(i) > bestRSSI)
            {
                bestRSSI = WiFi.RSSI(i);
                memcpy(bestBSSID, WiFi.BSSID(i), 6);
                bestChannel = WiFi.channel(i);
                found = true;
            }
        }
    }

    WiFi.scanDelete();

    if (!found)
    {
        WiFi.begin(targetSSID, password);
    }
    else
    {
        WiFi.begin(targetSSID, password, bestChannel, bestBSSID);
    }
}

void initWiFi()
{
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

        unsigned long startTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000)
        {
            ledIndicator(1, 250);
        }

        if (WiFi.status() == WL_CONNECTED)
        {
            WiFi.mode(WIFI_STA);

            if (MDNS.begin("cameracar"))
                MDNS.addService("http", "tcp", 80);
            ledIndicator(10, 50);
            ledIndicator(HIGH);
        }
        else
        {
            WiFi.mode(WIFI_AP);
            WiFi.setTxPower(WIFI_POWER_19_5dBm);
            WiFi.softAP("ESP-CAMERA-CAR", "carbondioxide");
        }
    }
    else
    {
        WiFi.mode(WIFI_AP);
        WiFi.setTxPower(WIFI_POWER_15dBm);
        WiFi.softAP("ESP-CAMERA-CAR", "carbondioxide");
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html", "text/html"); });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/style.css", "text/css"); });
    server.on("/wifimanager", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/wifimanager.html", "text/html"); });
    server.on("/wifimanager.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/wifimanager.css", "text/css"); });

    server.on("/update/spiffs", HTTP_POST, [](AsyncWebServerRequest *request)
              {
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
    response->addHeader("Connection", "close");
    request->send(response); }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
              {
    if (index == 0) {
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_SPIFFS)) {}
    }
    if (len) {
        Update.write(data, len);
    }
    if (final) {
        if (Update.end(true)) {
            delay(1000);
            ESP.restart();
        }
    } });

    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
              {
        if (request->hasParam(PARAM_INPUT_1, true)) writeFile(SPIFFS, ssidPath, request->getParam(PARAM_INPUT_1, true)->value().c_str());
        if (request->hasParam(PARAM_INPUT_2, true)) writeFile(SPIFFS, passPath, request->getParam(PARAM_INPUT_2, true)->value().c_str());
        if (request->hasParam(PARAM_INPUT_3, true)) writeFile(SPIFFS, ipPath, request->getParam(PARAM_INPUT_3, true)->value().c_str());
        if (request->hasParam(PARAM_INPUT_4, true)) writeFile(SPIFFS, gatewayPath, request->getParam(PARAM_INPUT_4, true)->value().c_str());
        request->send(200, "text/plain", "Done. ESP will restart.");
        delay(3000);
        ESP.restart(); });

    server.onNotFound([](AsyncWebServerRequest *request)
                      { request->send(404, "text/plain", "File Not Found"); });

    wsCarInput.onEvent(onCarInputWebSocketEvent);
    server.addHandler(&wsCarInput);

    ArduinoOTA.setMdnsEnabled(false);
    ArduinoOTA.begin();

    server.begin();

    startCameraServer();

    btStop();
    esp_bt_controller_disable();
}