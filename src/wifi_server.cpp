#include "config.h"
#include "wifi_server.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include "custom_motor_driver.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <esp_camera.h>
#include "display_control.h"
#include <esp_bt.h>
#include <ArduinoOTA.h>
#include "Melodies.h"
#include <esp_wifi.h>

#include <lwip/sockets.h>
#include <lwip/netdb.h>

WebServer webServer(80);
DNSServer dnsServer;
Preferences preferences;

volatile bool videoFlag = false;
TaskHandle_t cmdServerTaskHandle = NULL;
TaskHandle_t playMelodyTaskHandle = NULL;
TaskHandle_t obstacleAvoidanceModeTaskHandle = NULL;
TimerHandle_t dmsTimer = NULL;

// ----------------------------------------------------------------------
// CORE 1: STREAMING UDP (EL OJO DEL ROBOT)
// ----------------------------------------------------------------------
void cameraStreamTaskUDP(void *pvParameters)
{
    int serverFd = socket(AF_INET, SOCK_DGRAM, 0); // UDP
    if (serverFd < 0)
    {
#ifdef DEBUG
        Serial.println("[VIDEO] ❌ Error creando socket UDP");
#endif
        vTaskDelete(NULL);
    }

    // Configurar non-blocking para leer el handshake sin frenar la cámara
    fcntl(serverFd, F_SETFL, O_NONBLOCK);

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(8000);

    if (bind(serverFd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        close(serverFd);
        vTaskDelete(NULL);
    }

    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    bool hasClient = false;
    uint32_t frameId = 0;

    for (;;)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            hasClient = false;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Revisar si hay un paquete entrante (handshake "START" o heartbeat)
        char recvBuf[16];
        int n = recvfrom(serverFd, recvBuf, sizeof(recvBuf)-1, 0, (struct sockaddr *)&clientAddr, &clientAddrLen);
        if (n > 0)
        {
            recvBuf[n] = '\0';
            if (strncmp(recvBuf, "START", 5) == 0)
            {
                if (!hasClient) {
#ifdef DEBUG
                    Serial.println("[VIDEO] 🟢 Cliente UDP registrado en el puerto 8000.");
#endif
                }
                hasClient = true;
                videoFlag = true; // IMPORTANT: Force stream to start!
            }
            else if (strncmp(recvBuf, "STOP", 4) == 0)
            {
                hasClient = false;
#ifdef DEBUG
                Serial.println("[VIDEO] 🔴 Cliente UDP desconectado.");
#endif
            }
        }

        if (videoFlag && hasClient)
        {
            camera_fb_t *fb = esp_camera_fb_get();

            if (fb)
            {
                bool isValid = false;
                if (fb->len > 2000 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8)
                {
                    isValid = true;
                }

                if (isValid)
                {
                    uint16_t chunkSize = 1400;
                    uint16_t totalChunks = (fb->len + chunkSize - 1) / chunkSize;
                    uint32_t magic = 0x5649444F; // "VIDO"

                    for (uint16_t chunkId = 0; chunkId < totalChunks; chunkId++)
                    {
                        uint16_t currentChunkLen = chunkSize;
                        if (chunkId == totalChunks - 1) {
                            currentChunkLen = fb->len - (chunkId * chunkSize);
                        }

                        uint8_t packet[1414];
                        // Header (14 bytes)
                        memcpy(packet, &magic, 4);
                        memcpy(packet + 4, &frameId, 4);
                        memcpy(packet + 8, &chunkId, 2);
                        memcpy(packet + 10, &totalChunks, 2);
                        memcpy(packet + 12, &currentChunkLen, 2);

                        // Payload
                        memcpy(packet + 14, fb->buf + (chunkId * chunkSize), currentChunkLen);

                        sendto(serverFd, packet, 14 + currentChunkLen, 0, (struct sockaddr *)&clientAddr, clientAddrLen);
                        
                        // Pausa de 1ms exacta sin ceder el tick de FreeRTOS (que podria durar 10ms)
                        delayMicroseconds(1000);
                    }
                    frameId++;
                }
                
                esp_camera_fb_return(fb); 
            }
            // taskYIELD() cede el procesador brevemente para evitar que el Watchdog se queje
            taskYIELD();
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

// ----------------------------------------------------------------------
// CORE 1: SERVIDOR TCP DE COMANDOS Y CEREBRO MOTRIZ (Puerto 4000)
// ----------------------------------------------------------------------
void cmdServerTask(void *pvParameters)
{
    int serverFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (serverFd < 0)
        vTaskDelete(NULL);

    int enable = 1;
    setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(5000);

    if (bind(serverFd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        close(serverFd);
        vTaskDelete(NULL);
    }
    listen(serverFd, 1);

    int flags = fcntl(serverFd, F_GETFL, 0);
    fcntl(serverFd, F_SETFL, flags | O_NONBLOCK);

    TickType_t lastCmdTime = xTaskGetTickCount();
    const TickType_t TIMEOUT_TICKS = pdMS_TO_TICKS(1500);

    for (;;)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            stopAllMotors();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        struct sockaddr_in clientAddr;
        socklen_t clientAddrLen = sizeof(clientAddr);
        int clientFd = accept(serverFd, (struct sockaddr *)&clientAddr, &clientAddrLen);

        if (clientFd < 0)
        {
            // No hay clientes pendientes
            // Solo pausamos para no asfixiar a las otras tareas (el OTA se maneja en el loop principal)
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (clientFd >= 0)
        {
#ifdef DEBUG
            Serial.println("\n[CMD] 🟢 Cliente conectado al puerto 5000 (Comandos).");
#endif
            int nodelay = 1;
            setsockopt(clientFd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(int));

            struct timeval recvTimeout = {0, 100000};
            setsockopt(clientFd, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(recvTimeout));

            int keepAlive = 1, keepIdle = 2, keepInterval = 1, keepCount = 3;
            setsockopt(clientFd, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
            setsockopt(clientFd, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
            setsockopt(clientFd, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
            setsockopt(clientFd, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

            char rxBuffer[1024];
            int rxIndex = 0;
            char tempChunk[512];

            while (WiFi.status() == WL_CONNECTED)
            {
                int bytesRead = recv(clientFd, tempChunk, sizeof(tempChunk) - 1, 0);

                if (bytesRead > 0)
                {
                    tempChunk[bytesRead] = '\0';
                    
                    // Añadir al buffer estático circular
                    for (int i = 0; i < bytesRead; i++) {
                        if (rxIndex < 1023) {
                            rxBuffer[rxIndex++] = tempChunk[i];
                        } else {
                            rxIndex = 0; // Overflow de buffer, reiniciar
                            break;
                        }
                    }
                    rxBuffer[rxIndex] = '\0';

                    char* newlinePtr;
                    while ((newlinePtr = strchr(rxBuffer, '\n')) != NULL)
                    {
                        *newlinePtr = '\0'; // Reemplazar \n por \0
                        
                        char* line = rxBuffer;
                        
                        // Trim '\r' si existe
                        int lineLen = strlen(line);
                        if (lineLen > 0 && line[lineLen - 1] == '\r') {
                            line[lineLen - 1] = '\0';
                        }

                        if (strlen(line) > 0)
                        {
                            char* localCmd[8];
                            int localParam[8] = {0};
                            
                            char* saveptr;
                            char* token = strtok_r(line, "#", &saveptr);
                            int i = 0;
                            while (token != NULL && i < 8) {
                                localCmd[i] = token;
                                localParam[i] = atoi(token);
                                token = strtok_r(NULL, "#", &saveptr);
                                i++;
                            }

                            if (i > 0)
                            {
                                if (strcmp(localCmd[0], "CMD_SERVO") == 0)
                                {
                                    if (localParam[1] == 0)
                                        setPanAngle(180 - localParam[2]);
                                    else if (localParam[1] == 1)
                                        setTiltAngle(localParam[2]);
                                }
                                else if (strcmp(localCmd[0], "CMD_CAMERA") == 0)
                                {
                                    if (localParam[1] == panCenter && localParam[2] == tiltCenter)
                                        centerServos();
                                    else
                                    {
                                        setPanAngle(localParam[1]);
                                        setTiltAngle(localParam[2]);
                                    }
                                }
                                else if (strcmp(localCmd[0], "CMD_VIDEO") == 0)
                                {
                                    videoFlag = (localParam[1] == 1);
                                }
                                else if (strcmp(localCmd[0], "CMD_BUZZER") == 0)
                                {
                                    if (localParam[1] == 1) // Botón presionado en la app
                                    {
                                        if (melodyOn) {
                                            // Si ya estaba sonando, apagarla
                                            melodyOn = false;
                                            ledcWriteTone(buzzerChannel, 0);
                                        } else {
                                            // Si no estaba sonando, iniciarla
                                            melodyOn = true;
                                            if (playMelodyTaskHandle != NULL)
                                                xTaskNotifyGive(playMelodyTaskHandle);
                                        }
                                    }
                                }
                                else if (strcmp(localCmd[0], "CMD_LIGHT") == 0)
                                {
                                    if (localParam[1] == 1) {
                                        enableLaser = !enableLaser; // Toggle para evadir bug de la app
                                    } else {
                                        enableLaser = false; // Por si algún día manda el 0
                                    }
                                    turnLaserOn(enableLaser);
                                }
                                else if (strcmp(localCmd[0], "CMD_LED_MOD") == 0)
                                {
                                    if (localParam[1] == 2)
                                    {
                                        enableObstacleAvoidance = true;
                                        enableIROnlyMode = false;
                                        if (obstacleAvoidanceModeTaskHandle != NULL)
                                            xTaskNotifyGive(obstacleAvoidanceModeTaskHandle);
                                    }
                                    else if (localParam[1] == 3)
                                    {
                                        enableObstacleAvoidance = false;
                                        enableIROnlyMode = true;
                                        if (obstacleAvoidanceModeTaskHandle != NULL)
                                            xTaskNotifyGive(obstacleAvoidanceModeTaskHandle);
                                    }
                                    else
                                    {
                                        enableObstacleAvoidance = false;
                                        enableIROnlyMode = false;
                                    }
                                }
                                else if (strcmp(localCmd[0], "CMD_MODE") == 0)
                                {
                                    if (strcmp(localCmd[1], "three") == 0)
                                    {
                                        enableObstacleAvoidance = true;
                                        if (obstacleAvoidanceModeTaskHandle != NULL)
                                            xTaskNotifyGive(obstacleAvoidanceModeTaskHandle);
                                    }
                                    else
                                    {
                                        enableObstacleAvoidance = false;
                                    }
                                }
                                else if (strcmp(localCmd[0], "CMD_MOTOR") == 0)
                                {
                                    if (!enableObstacleAvoidance && !enableIROnlyMode)
                                        driveSafe(localParam[1], localParam[2], localParam[3], localParam[4]);
                                }
                                else if (strcmp(localCmd[0], "CMD_M_MOTOR") == 0 || strcmp(localCmd[0], "CMD_CAR_ROTATE") == 0)
                                {
                                    if (!enableObstacleAvoidance && !enableIROnlyMode)
                                        driveMecanum(localParam[1], localParam[2], localParam[3], localParam[4]);
                                }
                            }
                        }

                        // Desplazar lo que quede en el buffer hacia el principio
                        int remaining = rxIndex - (newlinePtr - rxBuffer) - 1;
                        if (remaining > 0) {
                            memmove(rxBuffer, newlinePtr + 1, remaining);
                            rxIndex = remaining;
                            rxBuffer[rxIndex] = '\0';
                        } else {
                            rxIndex = 0;
                            rxBuffer[0] = '\0';
                        }
                    }
                }
                else if (bytesRead == 0)
                {
#ifdef DEBUG
                    Serial.println("[CMD] ℹ️ Conexión cerrada normalmente por la app.");
#endif
                    break;
                }
                else
                {
                    if (errno != EWOULDBLOCK && errno != EAGAIN)
                    {
#ifdef DEBUG
                        Serial.printf("[CMD] ❌ Socket roto. errno: %d\n", errno);
#endif
                        if (errno == 113 || errno == 104 || errno == 128)
                        {
#ifdef DEBUG
                            Serial.println("🚨 RED MUERTA. FRENANDO.");
#endif
                            stopAllMotors();
                        }
                        break;
                    }
                }

                // El Dead Man's Switch (DMS Timer) y la Máquina de Estados WiFi 
                // se encargan ahora de la seguridad en segundo plano de manera autónoma.
            }

            stopAllMotors();
            xTimerStop(dmsTimer, 0); // Stop DMS timer
            close(clientFd);
#ifdef DEBUG
            Serial.println("[CMD] 🔴 Puerto 5000 cerrado y libre.");
#endif
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ----------------------------------------------------------------------
// GESTIÓN DE CONFIGURACIÓN Y PORTAL CAUTIVO
// ----------------------------------------------------------------------
void handleRoot()
{
    if (SPIFFS.exists("/wifimanager.html"))
    {
        File file = SPIFFS.open("/wifimanager.html", "r");
        webServer.streamFile(file, "text/html; charset=utf-8");
        file.close();
    }
    else
    {
        webServer.send(404, "text/plain", "Archivo wifimanager.html no encontrado");
    }
}

void handleCSS()
{
    if (SPIFFS.exists("/wifimanager.css"))
    {
        File file = SPIFFS.open("/wifimanager.css", "r");
        webServer.streamFile(file, "text/css");
        file.close();
    }
    else
    {
        webServer.send(404, "text/plain", "CSS no encontrado");
    }
}

void handleScan()
{
    int n = WiFi.scanNetworks();
    String json = "[";
    for (int i = 0; i < n; ++i)
    {
        if (i > 0) json += ",";
        json += "{\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
    }
    json += "]";
    webServer.send(200, "application/json", json);
}

void handleSave()
{
    String reqSSID = webServer.arg("ssid");
    String reqPASS = webServer.arg("pass");
    String reqIP = webServer.arg("ip");
    String reqGW = webServer.arg("gateway");

    preferences.begin("wifi_config", false);
    preferences.putString("ssid", reqSSID);
    preferences.putString("pass", reqPASS);
    preferences.putString("ip", reqIP);
    preferences.putString("gw", reqGW);
    preferences.end();

    webServer.send(200, "text/html", "<html><body><h1>Guardado!</h1></body></html>");
    delay(2000);
    ESP.restart();
}

void captivePortalTask(void *pvParameters)
{
    int currentLedState = 0;
    TickType_t lastBlink = xTaskGetTickCount();
    for (;;)
    {
        dnsServer.processNextRequest();
        webServer.handleClient();
        if ((xTaskGetTickCount() - lastBlink) >= pdMS_TO_TICKS(500))
        {
            currentLedState = !currentLedState;
            ledIndicator(currentLedState);
            lastBlink = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void startCaptivePortal()
{
    if (!SPIFFS.begin(true))
    {
#ifdef DEBUG
        Serial.println("❌ Fallo SPIFFS");
#endif
    }

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    IPAddress apIP(192, 168, 4, 1);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    dnsServer.start(53, "*", apIP);

    webServer.on("/", handleRoot);
    webServer.on("/wifimanager.css", handleCSS);
    webServer.on("/scan", HTTP_GET, handleScan);
    webServer.on("/save", HTTP_POST, handleSave);
    webServer.onNotFound(handleRoot);
    webServer.begin();
    ArduinoOTA.begin();

    // Lanzar como tarea FreeRTOS para que setup() termine y OTA funcione
    xTaskCreatePinnedToCore(captivePortalTask, "CaptivePortal", 4096, NULL, 1, NULL, 0);
}

void dmsTimerCallback(TimerHandle_t xTimer)
{
#ifdef DEBUG
    Serial.println("🚨 DMS ACTIVADO: No se recibieron comandos. Apagando motores.");
#endif
    stopAllMotors();
    melodyOn = false;
    ledcWriteTone(buzzerChannel, 0);
}

volatile bool isWiFiConnected = false;

void onWiFiEvent(WiFiEvent_t event)
{
    if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) 
    {
        isWiFiConnected = true;
        updateDisplayState(DISPLAY_CONNECTED, WiFi.localIP().toString().c_str());
    }
    else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
    {
        if (isWiFiConnected) 
        {
#ifdef DEBUG
            Serial.println("🚨 EVENTO WIFI: Desconectado. Cortando motores e intentando reconexión...");
#endif
            stopAllMotors();
            melodyOn = false;
            ledcWriteTone(buzzerChannel, 0);
            isWiFiConnected = false;
            updateDisplayState(DISPLAY_CONNECTING_WIFI, "Reconectando...");
            WiFi.reconnect();
        }
    }
}

void initWiFi()
{
    // Inicializar el Dead Man's Switch Timer (1000ms)
    dmsTimer = xTimerCreate("DMSTimer", pdMS_TO_TICKS(1000), pdFALSE, (void *)0, dmsTimerCallback);

    ledIndicator(0);
    WiFi.persistent(false);
    WiFi.setSleep(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_17dBm); // Aumentado a 17dBm para mejorar el rango y latencia (FPS)
    btStop();
    esp_bt_controller_disable();

    // Conectar el evento de red de máxima prioridad
    WiFi.onEvent(onWiFiEvent);

    preferences.begin("wifi_config", true);
    String storedSSID = preferences.getString("ssid", "");
    String storedPASS = preferences.getString("pass", "");
    String storedIP = preferences.getString("ip", "");
    String storedGW = preferences.getString("gw", "");
    preferences.end();

    if (storedSSID.length() == 0)
    {
        updateDisplayState(DISPLAY_PORTAL_ACTIVE);
        startCaptivePortal();
        return; // Detener la inicialización de WiFi STA
    }

    IPAddress staticIP, gateway, subnet(255, 255, 255, 0), dns(8, 8, 8, 8);
    if (storedIP.length() > 0 && storedGW.length() > 0)
    {
        staticIP.fromString(storedIP);
        gateway.fromString(storedGW);
        WiFi.config(staticIP, gateway, subnet, dns);
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); // EVITAR desconexiones por ahorro de energía
    WiFi.setAutoReconnect(true);
    updateDisplayState(DISPLAY_CONNECTING_WIFI, storedSSID.c_str());
    
    // Connect to the best AP in a mesh network
    WiFi.begin(storedSSID.c_str(), storedPASS.c_str());
    wifi_config_t wifi_config;
    esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
    wifi_config.sta.bssid_set = 0;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL; // Roaming al BSSID con mejor señal
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 12)
    {
        ledIndicator(1, 80);
        vTaskDelay(pdMS_TO_TICKS(1000));
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        WiFi.disconnect(); // Detener intentos de conexión en segundo plano
        updateDisplayState(DISPLAY_PORTAL_ACTIVE);
        startCaptivePortal();
        return;
    }

    updateDisplayState(DISPLAY_CONNECTED, WiFi.localIP().toString().c_str());
    ledIndicator(2, 60);

    xTaskCreatePinnedToCore(cameraStreamTaskUDP, "CamUDPStream", 1024 * 4, NULL, 1, NULL, 1);

    if (obstacleAvoidanceModeTaskHandle == NULL)
    {
        xTaskCreatePinnedToCore(obstacleAvoidanceMode, "ObstacleTask", 1024 * 3, NULL, 3, &obstacleAvoidanceModeTaskHandle, 1);
    }

    xTaskCreatePinnedToCore(cmdServerTask, "CmdServerTask", 1024 * 4, NULL, 2, &cmdServerTaskHandle, 1);
    ArduinoOTA.begin();
    ledIndicator(1);
}