#include "config.h"
#include "wifi_server.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include "esp_bt.h" // Para apagar Bluetooth
#include <Update.h>

// --- Definición de variables y objetos del servidor ---
AsyncWebServer server(80);
AsyncWebSocket wsCamera("/Camera");
AsyncWebSocket wsCarInput("/CarInput");
int cameraClientId = 0;

TaskHandle_t sendCameraPictureTask;
TaskHandle_t cleanupWSClientsTask;

// Variables de WiFi Manager
String ssid, pass, ip, gateway;
const char *ssidPath = "/ssid.txt", *passPath = "/pass.txt", *ipPath = "/ip.txt", *gatewayPath = "/gateway.txt";
const char *PARAM_INPUT_1 = "ssid", *PARAM_INPUT_2 = "pass", *PARAM_INPUT_3 = "ip", *PARAM_INPUT_4 = "gateway";

// --- Prototipos de funciones internas ---
void onCameraWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
String readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void scanAndConnectToBestAP(const char *targetSSID, const char *password);
void cleanupWSClients_task(void *parameters);

// --- Tareas de Red (OTA, Limpieza, Telemetría) ---
void arduinoOTA_task(void *parameters)
{
    ArduinoOTA.setMdnsEnabled(false);
    ArduinoOTA.begin();
    for (;;)
    {
        ArduinoOTA.handle();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void cleanupWSClients_task(void *parameters)
{
    for (;;)
    {
        wsCamera.cleanupClients();
        wsCarInput.cleanupClients();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void sendTelemetryTask(void *parameters)
{
    // Crea un búfer de caracteres de tamaño fijo. 128 bytes es más que suficiente.
    char telemetryJson[128];

    for (;;)
    {
        if (wsCarInput.count() > 0)
        {

            // Usa snprintf para construir el JSON de forma segura en el búfer
            snprintf(telemetryJson, sizeof(telemetryJson),
                     "{\"rssi\":%d,\"obstacle\":%s}",
                     WiFi.RSSI(),
                     obstacleFound ? "true" : "false");

            // Envía el contenido del búfer
            wsCarInput.textAll(telemetryJson);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
// --- Handlers de WebSocket ---
void onCameraWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT)
    {
#ifdef DEBUG
        Serial.printf("WS Camera client #%u connected\n", client->id());
#endif
        cameraClientId = client->id();
        xTaskCreatePinnedToCore(sendCameraPicture, "sendCameraPicture", STACK_SIZE, NULL, 2, &sendCameraPictureTask, CONFIG_ARDUINO_RUNNING_CORE);
        xTaskCreatePinnedToCore(cleanupWSClients_task, "cleanupWSClients", STACK_SIZE, NULL, 1, &cleanupWSClientsTask, 0);
    }
    else if (type == WS_EVT_DISCONNECT)
    {
#ifdef DEBUG
        Serial.printf("WS Camera client #%u disconnected\n", client->id());
#endif
        cameraClientId = 0;
        if (sendCameraPictureTask != NULL)
            vTaskDelete(sendCameraPictureTask);
        if (cleanupWSClientsTask != NULL)
            vTaskDelete(cleanupWSClientsTask);
    }
}

void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT)
    {
#ifdef DEBUG
        Serial.printf("WS Control client #%u connected\n", client->id());
#endif
        leftBackLed(HIGH);
        rightBackLed(HIGH);
    }
    else if (type == WS_EVT_DISCONNECT)
    {
#ifdef DEBUG
        Serial.printf("WS Control client #%u disconnected\n", client->id());
#endif
        moveCar(STOP);
        digitalWrite(lightPin, LOW);
        enableLight = false;
        panServo.write(panCenter);
        tiltServo.write(tiltCenter);
        if (melodyOn)
        {
            vTaskDelete(playMelodyTask);
            ledcDetachPin(buzzerPin);
            melodyOn = false;
        }
        if (enableObstacleAvoidance)
        {
            vTaskDelete(obstacleAvoidanceModeTask);
            enableObstacleAvoidance = false;
            obstacleFound = false;
        }
    }
    else if (type == WS_EVT_DATA && len > 0)
    {
        char command = data[0];              // El primer byte es el comando
        int value = (len > 1) ? data[1] : 0; // El segundo byte es el valor, si existe

        switch (command)
        {
        case 'M':
            if (!obstacleFound)
                moveCar(value);
            break;
        case 'S':
            motorSpeed = map(value, 1, 5, 200, 255);
            break;
        case 'L':
            enableLight = !enableLight;
            digitalWrite(lightPin, enableLight);
            break;
        case 'P':
            panServo.write(value);
            break;
        case 'T':
            tiltServo.write(value);
            break;
        case 'C':
            panServo.write(panCenter);
            tiltServo.write(tiltCenter);
            break;
        case 'H': // Horn (Melody)
            melodyOn = !melodyOn;
            if (melodyOn)
            {
                xTaskCreatePinnedToCore(playMelody, "playMelody", STACK_SIZE, NULL, 1, &playMelodyTask, CONFIG_ARDUINO_RUNNING_CORE);
            }
            else
            {
                vTaskDelete(playMelodyTask);
                ledcDetachPin(buzzerPin);
            }
            break;
        case 'O': // Obstacle Avoidance
            enableObstacleAvoidance = !enableObstacleAvoidance;
            if (enableObstacleAvoidance)
            {
                xTaskCreatePinnedToCore(obstacleAvoidanceMode, "obstacleAvoidanceMode", STACK_SIZE, NULL, 2, &obstacleAvoidanceModeTask, CONFIG_ARDUINO_RUNNING_CORE);
            }
            else
            {
                vTaskDelete(obstacleAvoidanceModeTask);
                obstacleFound = false;
                moveCar(STOP);
            }
            break;
        }
    }
}

// --- Funciones de Archivos y Conexión WiFi Inteligente ---
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

#ifdef DEBUG
    Serial.println("🔍 Scanning for the best AP...");
#endif

    int n = WiFi.scanNetworks(false, true);
    if (n == 0)
    {
#ifdef DEBUG
        Serial.println("❌ No networks found.");
#endif
        return;
    }

    for (int i = 0; i < n; ++i)
    {
        if (WiFi.SSID(i) == targetSSID)
        {
#ifdef DEBUG
            Serial.printf("📶 Found: %s | Ch: %d | RSSI: %d dBm\n", WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i));
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

    WiFi.scanDelete();

    if (!found)
    {
#ifdef DEBUG
        Serial.println("⚠️ Desired SSID not found. Trying a normal connection...");
#endif
        WiFi.begin(targetSSID, password);
    }
    else
    {
#ifdef DEBUG
        Serial.printf("🔗 Connecting to the best node (RSSI: %d dBm) on channel %d\n", bestRSSI, bestChannel);
#endif
        WiFi.begin(targetSSID, password, bestChannel, bestBSSID);
    }
}

// --- Función Principal de Inicialización ---
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
#ifdef DEBUG
            Serial.println("\n✅ WiFi Connected!");
            Serial.print("   IP address: ");
            Serial.println(WiFi.localIP());
#endif
            if (MDNS.begin("cameracar"))
                MDNS.addService("http", "tcp", 80);
            ledIndicator(10, 50);
            ledIndicator(HIGH);
        }
        else
        {
#ifdef DEBUG
            Serial.println("\n❌ WiFi connection failed. AP Mode is active.");
#endif
        }
    }

    // Configura el AP como respaldo y para configuración
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.softAP("ESP-CAMERA-CAR", "carbondioxide");
#ifdef DEBUG
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
#endif

    // Rutas del Servidor Web
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html", "text/html"); });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/style.css", "text/css"); });
    server.on("/wifimanager", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/wifimanager.html", "text/html"); });
    server.on("/wifimanager.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/wifimanager.css", "text/css"); });
    // --- AÑADE ESTE BLOQUE COMPLETO PARA LA SUBIDA DE SPIFFS POR OTA ---
    server.on("/update/spiffs", HTTP_POST, [](AsyncWebServerRequest *request)
              {
    // La respuesta al final de la carga
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
    response->addHeader("Connection", "close");
    request->send(response); }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
              {
    // Inicia la actualización si es el primer paquete de datos
    if (index == 0) {
#ifdef DEBUG
        Serial.println("Actualización de SPIFFS iniciada.");
#endif
        // Inicia el proceso de actualización. U_SPIFFS significa "actualizar el SPIFFS".
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_SPIFFS)) {
#ifdef DEBUG
            Update.printError(Serial);
#endif
        }
    }

    // Escribe el paquete de datos en la memoria flash
    if (len) {
        Update.write(data, len);
    }

    // Si es el último paquete, finaliza la actualización
    if (final) {
        if (Update.end(true)) {
#ifdef DEBUG
            Serial.println("¡Actualización de SPIFFS completada!");
#endif
            delay(1000);
            ESP.restart();
        } else {
#ifdef DEBUG
            Update.printError(Serial);
#endif
        }
    } });
    // --- FIN DEL BLOQUE ---

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

    // Adjunta los Handlers de WebSocket
    wsCamera.onEvent(onCameraWebSocketEvent);
    server.addHandler(&wsCamera);
    wsCarInput.onEvent(onCarInputWebSocketEvent);
    server.addHandler(&wsCarInput);

    server.begin();

    // Optimizaciones de energía
    WiFi.setSleep(true);
    btStop();
    esp_bt_controller_disable();
}