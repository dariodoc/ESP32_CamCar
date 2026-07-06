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
int carInputClientId = 0; // <--- AÑADE ESTA LÍNEA

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
void servoControlTask(void *parameters);

volatile int targetPan = 75;
volatile int targetTilt = 90;
volatile int targetDirection = 0; // 0 es STOP

TaskHandle_t servoControlTaskHandle = NULL;

// --- NUEVO: Tarea dedicada para mover los servos de forma síncrona ---
void servoControlTask(void *parameters)
{
    // Inicializamos las posiciones actuales en el centro al arrancar
    static int currentPan = 75;
    static int currentTilt = 90;

    // Configura cuántos grados máximo se puede mover el servo por ciclo (menor número = más suave y menos consumo)
    const int maxStep = 2;

    for (;;)
    {
        // --- Suavizado de PASO para PAN ---
        if (currentPan != targetPan)
        {
            int diff = targetPan - currentPan;
            if (abs(diff) <= maxStep)
            {
                currentPan = targetPan; // Si está muy cerca, llega al objetivo
            }
            else
            {
                currentPan += (diff > 0) ? maxStep : -maxStep; // Se mueve a pasos sutiles
            }
            panServo.write(currentPan);
        }

        // --- Suavizado de PASO para TILT ---
        if (currentTilt != targetTilt)
        {
            int diff = targetTilt - currentTilt;
            if (abs(diff) <= maxStep)
            {
                currentTilt = targetTilt;
            }
            else
            {
                currentTilt += (diff > 0) ? maxStep : -maxStep;
            }
            tiltServo.write(currentTilt);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Bajamos a 20ms para compensar la suavidad de los pasos
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
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// --- Handlers de WebSocket ---
void onCameraWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
        // --- NUEVO: ELIMINADOR DE CLONES (Previene el colapso por Refresh) ---
        // Si ya había alguien conectado y entra un ID nuevo, matamos al viejo de inmediato.
        if (cameraClientId != 0 && cameraClientId != client->id())
        {
            AsyncWebSocketClient *oldClient = server->client(cameraClientId);
            if (oldClient)
            {
                oldClient->close(); // Cortamos la RAM de la conexión fantasma
            }
        }

        // Ahora sí, le damos la bienvenida al cliente nuevo
        cameraClientId = client->id();

#ifdef DEBUG
        Serial.printf("Camera client connected: %u\n", cameraClientId);
#endif
        break;

    case WS_EVT_DISCONNECT:
        // --- BLINDAJE DE DESCONEXIÓN ---
        // IMPORTANTE: Solo liberamos el ID si el que se desconecta es el cliente ACTIVO.
        // Si no ponemos este "if", un zombi viejo podría borrar el ID del cliente nuevo.
        if (client->id() == cameraClientId)
        {
            cameraClientId = 0;
        }

#ifdef DEBUG
        Serial.printf("Camera client disconnected: %u\n", client->id());
#endif
        break;
    }
}

void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT)
    {
        // --- NUEVO: ELIMINADOR DE CLONES PARA EL JOYSTICK ---
        if (carInputClientId != 0 && carInputClientId != client->id())
        {
            AsyncWebSocketClient *oldClient = server->client(carInputClientId);
            if (oldClient)
            {
                oldClient->close(); // Liberamos el socket viejo inmediatamente
            }
        }
        carInputClientId = client->id();
#ifdef DEBUG
        Serial.printf("WS Control client #%u connected\n", client->id());
#endif
    }
    else if (type == WS_EVT_DISCONNECT)
    {
        // --- BLINDAJE DE DESCONEXIÓN ---
        if (client->id() == carInputClientId)
        {
            carInputClientId = 0;
        }
#ifdef DEBUG
        Serial.printf("WS Control client #%u disconnected\n", client->id());
#endif
        targetDirection = STOP; // <--- Delegado a la tarea síncrona
        digitalWrite(lightPin, LOW);
        enableLight = false;

        // Actualiza las variables de destino al centro en lugar de escribir directo
        targetPan = 75;  // panCenter
        targetTilt = 90; // tiltCenter

        if (melodyOn)
        {
            melodyOn = false;
            ledcWriteTone(buzzerChannel, 0); // Solo silenciamos
        }
        if (enableObstacleAvoidance)
        {
            enableObstacleAvoidance = false;
            obstacleFound = false;
        }
    }
    else if (type == WS_EVT_DATA && len > 0)
    {
        // --- 1. ESCUDO ANTI-FRAGMENTACIÓN ---
        // Verificamos que el paquete sea un mensaje completo y no un fragmento roto.
        // Leer un fragmento roto causa un Core Panic inmediato en esta librería.
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (!(info->final && info->index == 0 && info->len == len))
        {
            return; // Descartamos la basura digital
        }

        // --- 2. ESCUDO ANTI-DDOS (Freno de Hardware) ---
        // Ignoramos cualquier comando si hace menos de 50 milisegundos que recibimos el anterior.
        // Esto limita el tráfico a un máximo seguro de 20 comandos por segundo.
        static unsigned long lastWsTime = 0;
        unsigned long currentWsTime = millis();

        if (currentWsTime - lastWsTime < 50)
        {
            return; // Bloqueamos el ataque de la laptop
        }
        lastWsTime = currentWsTime;

        // --- 3. PROCESAMIENTO SEGURO ---
        char command = data[0];              // El primer byte es el comando
        int value = (len > 1) ? data[1] : 0; // El segundo byte es el valor
        switch (command)
        {
        case 'M':
            // if (!obstacleFound)
            targetDirection = value; // <--- Delegado a la tarea síncrona
            break;
        case 'S':
            motorSpeed = map(value, 1, 5, 200, 255);
            break;
        case 'L':
            enableLight = !enableLight;
            digitalWrite(lightPin, enableLight);
            break;
        case 'P':
            targetPan = value; // Asigna el objetivo, la tarea se encarga del movimiento
            break;
        case 'T':
            targetTilt = value; // Asigna el objetivo, la tarea se encarga del movimiento
            break;
        case 'C':
            targetPan = 75;  // panCenter
            targetTilt = 90; // tiltCenter
            break;
        case 'H': // Horn (Melody)
            melodyOn = !melodyOn;
            if (melodyOn)
            {
                // En lugar de crear, solo despertamos a la tarea
                xTaskNotifyGive(playMelodyTask);
            }
            else
            {
                // Silenciamos si el usuario lo desactiva
                ledcWriteTone(buzzerChannel, 0);
            }
            break;
        case 'O': // Obstacle Avoidance
            enableObstacleAvoidance = !enableObstacleAvoidance;
            if (enableObstacleAvoidance)
            {
                // Despertamos a la tarea del sensor
                xTaskNotifyGive(obstacleAvoidanceModeTask);
            }
            else
            {
                // Limpiamos banderas. En su siguiente ciclo (30ms), la tarea se dormirá sola.
                obstacleFound = false;
                targetDirection = STOP;
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
#endif
            // 👇 NUEVO: Apagamos el AP interno para que la antena descanse
            WiFi.mode(WIFI_STA);

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
            // 👇 NUEVO: Solo encendemos el AP si falló la red principal
            WiFi.mode(WIFI_AP);
            WiFi.setTxPower(WIFI_POWER_15dBm);
            WiFi.softAP("ESP-CAMERA-CAR", "carbondioxide");
#ifdef DEBUG
            Serial.print("AP IP address: ");
            Serial.println(WiFi.softAPIP());
#endif
        }
    }
    else
    {
        // 👇 EL BLINDAJE FINAL: Si la memoria está vacía (Primer arranque)
#ifdef DEBUG
        Serial.println("\n⚠️ No SSID saved. Forcing AP Mode.");
#endif
        WiFi.mode(WIFI_AP);
        WiFi.setTxPower(WIFI_POWER_15dBm);
        WiFi.softAP("ESP-CAMERA-CAR", "carbondioxide");
    }

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

    ArduinoOTA.setMdnsEnabled(false);
    ArduinoOTA.begin();

    server.begin();

    // Optimizaciones de energía
    // WiFi.setSleep(false);
    btStop();
    esp_bt_controller_disable();

    // Reactivar protección de voltaje para evitar corrupción de flash
    // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1);
}