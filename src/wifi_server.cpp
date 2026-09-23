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
// CORE 0: STREAMING TCP (EL OJO DEL ROBOT)
// ----------------------------------------------------------------------
void cameraStreamTaskTCP(void *pvParameters)
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
    serverAddr.sin_port = htons(8000);

    if (bind(serverFd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        close(serverFd);
        vTaskDelete(NULL);
    }
    listen(serverFd, 1);

    const TickType_t FRAME_TARGET_TIME = pdMS_TO_TICKS(66); // ~15 FPS

    for (;;)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        struct sockaddr_in clientAddr;
        socklen_t clientAddrLen = sizeof(clientAddr);
        int clientFd = accept(serverFd, (struct sockaddr *)&clientAddr, &clientAddrLen);

        if (clientFd >= 0)
        {
#ifdef DEBUG
            Serial.println("\n[VIDEO] 🟢 Cliente conectado al puerto 8000 (Video).");
#endif
            videoFlag = true; // Auto-start streaming for Raspberry Pi app

            // 🚀 RESTAURADO TCP_NODELAY: La app de Android asume que el header de 4 bytes 
            // llega en un paquete separado. Habilitar Nagle fusionaba los paquetes y rompía 
            // el parser de Android (77396 fps bug).
            int nodelay = 1;
            setsockopt(clientFd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(int));
            
            // 🚀 RESTAURADO SO_SNDBUF: 
            // Si no pedimos 32KB, el buffer por defecto de lwIP es de 5KB. Un frame JPEG pesa ~10KB-20KB.
            // Si el frame no cabe en el buffer, send() se bloquea sincrónicamente esperando que el cliente 
            // envíe TCP ACKs por Wi-Fi. Esto sumaba decenas de milisegundos de lag por frame.
            // Gracias a que ahora usamos SO_LINGER = 0, podemos usar 32KB sin miedo a agotar la RAM,
            // ya que se liberan instantáneamente al desconectar.
            int sndbuf = 32768;
            setsockopt(clientFd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

            struct timeval sendTimeout = {0, 200000};
            setsockopt(clientFd, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));

            // 🚀 FIX CRÍTICO: Prevenir "PBUF Exhaustion" en lwIP.
            // Cuando la red se congestiona, el socket se cierra. Pero por defecto, TCP entra en "TIME_WAIT"
            // y retiene 32KB de RAM del ESP32 por 2 minutos. Si te reconectas inmediatamente, el ESP32 ya
            // no tiene memoria RAM (PBUFs) para la nueva conexión, lo que causa que la red se vuelva lentísima
            // (2-5 FPS de forma permanente).
            // SO_LINGER con timeout 0 obliga a que al cerrar el socket, se envíe un RST (Reset)
            // que destruye la conexión y libera el 100% de la RAM instantáneamente. ¡Cada reconexión será fresca!
            struct linger so_linger;
            so_linger.l_onoff = 1;
            so_linger.l_linger = 0;
            setsockopt(clientFd, SOL_SOCKET, SO_LINGER, &so_linger, sizeof(so_linger));

            bool wasStreaming = false;

            while (WiFi.status() == WL_CONNECTED)
            {
                // 🚀 DETECTOR DE DESCONEXIÓN:
                // Revisa instantáneamente si la app colgó el teléfono, incluso estando en Mute.
                char peekBuf[1];
                int peekRes = recv(clientFd, peekBuf, 1, MSG_DONTWAIT);
                if (peekRes == 0)
                {
#ifdef DEBUG
                    Serial.println("[VIDEO] ℹ️ Conexión de video cerrada normalmente por la app.");
#endif
                    break;
                }

                if (videoFlag)
                {
                    if (!wasStreaming)
                    {
#ifdef DEBUG
                        Serial.println("[VIDEO] 🎥 Transmisión de frames INICIADA.");
#endif
                        wasStreaming = true;
                    }

                    camera_fb_t *fb = esp_camera_fb_get();

                    if (fb)
                    {

                        bool isValid = false;
                        if (fb->len > 2000 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8)
                        {
                            for (size_t i = fb->len - 16; i < fb->len - 1; i++)
                            {
                                if (fb->buf[i] == 0xFF && fb->buf[i + 1] == 0xD9)
                                {
                                    isValid = true;
                                    break;
                                }
                            }
                        }

                        if (isValid)
                        {
                            uint8_t header[4];
                            header[0] = (uint8_t)(fb->len & 0xFF);
                            header[1] = (uint8_t)((fb->len >> 8) & 0xFF);
                            header[2] = (uint8_t)((fb->len >> 16) & 0xFF);
                            header[3] = (uint8_t)((fb->len >> 24) & 0xFF);

                            uint8_t firstPacket[1460];
                            size_t firstPayloadSize = (fb->len > 1456) ? 1456 : fb->len;
                            memcpy(firstPacket, header, 4);
                            memcpy(firstPacket + 4, fb->buf, firstPayloadSize);
                            size_t packetSize = 4 + firstPayloadSize;

                            bool socketError = false;
                            int retries = 0;
                            const int MAX_RETRIES = 50; // 50 * 20ms = 1000ms timeout
                            size_t bytesWrittenTotal = 0;

                            while (bytesWrittenTotal < packetSize)
                            {
                                int s = send(clientFd, firstPacket + bytesWrittenTotal, packetSize - bytesWrittenTotal, MSG_NOSIGNAL);
                                if (s < 0)
                                {
                                    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOMEM)
                                    {
                                        retries++;
                                        if (retries > MAX_RETRIES) { socketError = true; break; }
                                        vTaskDelay(pdMS_TO_TICKS(20));
                                        continue;
                                    }
                                    socketError = true; break;
                                }
                                bytesWrittenTotal += s;
                                retries = 0;
                            }

                            if (!socketError)
                            {
                                size_t payloadWritten = firstPayloadSize;
                                retries = 0;
                                while (payloadWritten < fb->len)
                                {
                                    int s = send(clientFd, fb->buf + payloadWritten, fb->len - payloadWritten, MSG_NOSIGNAL);
                                    if (s < 0)
                                    {
                                        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOMEM)
                                        {
                                            retries++;
                                            if (retries > MAX_RETRIES) { socketError = true; break; }
                                            vTaskDelay(pdMS_TO_TICKS(20));
                                            continue;
                                        }
                                        socketError = true; break;
                                    }
                                    payloadWritten += s;
                                    retries = 0;
                                }
                            }

                            if (socketError)
                            {
#ifdef DEBUG
                                Serial.println("[VIDEO] ⚠️ Saturación de red severa. Cortando conexión de video.");
#endif
                                esp_camera_fb_return(fb);
                                break;
                            }
                        }
                        
                        esp_camera_fb_return(fb); 
                    }
                }
                else
                {
                    if (wasStreaming)
                    {
#ifdef DEBUG
                        Serial.println("[VIDEO] 🛑 CMD_VIDEO 0 detectado. Pausando stream (Modo Mute)...");
#endif
                        wasStreaming = false;
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }

            videoFlag = false;
            close(clientFd);
#ifdef DEBUG
            Serial.println("[VIDEO] 🔴 Puerto 8000 cerrado y libre.");
#endif
        }
        vTaskDelay(pdMS_TO_TICKS(50));
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
                                    if (localParam[1] == 1)
                                    {
                                        int freq = (localParam[2] > 0) ? localParam[2] : 2000;
                                        toneToPlay(buzzerPin, buzzerChannel, freq, 100);
                                    }
                                    else
                                        ledcWriteTone(buzzerChannel, 0);
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
                                        if (obstacleAvoidanceModeTaskHandle != NULL)
                                            xTaskNotifyGive(obstacleAvoidanceModeTaskHandle);
                                    }
                                    else
                                    {
                                        enableObstacleAvoidance = false;
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
                                    if (!enableObstacleAvoidance)
                                        driveSafe(localParam[1], localParam[2], localParam[3], localParam[4]);
                                }
                                else if (strcmp(localCmd[0], "CMD_M_MOTOR") == 0 || strcmp(localCmd[0], "CMD_CAR_ROTATE") == 0)
                                {
                                    if (!enableObstacleAvoidance)
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

void startCaptivePortal()
{
    if (!SPIFFS.begin(true))
    {
#ifdef DEBUG
        Serial.println("❌ Fallo SPIFFS");
#endif
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP-CAMERA-CAR", "carbondioxide");
    IPAddress apIP(192, 168, 4, 1);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    dnsServer.start(53, "*", apIP);

    webServer.on("/", handleRoot);
    webServer.on("/wifimanager.css", handleCSS);
    webServer.on("/scan", HTTP_GET, handleScan);
    webServer.on("/save", HTTP_POST, handleSave);
    webServer.onNotFound(handleRoot);
    webServer.begin();

    int currentLedState = 0;
    TickType_t lastBlink = xTaskGetTickCount();
    while (true)
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

void dmsTimerCallback(TimerHandle_t xTimer)
{
#ifdef DEBUG
    Serial.println("🚨 DMS ACTIVADO: No se recibieron comandos. Apagando motores.");
#endif
    stopAllMotors();
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
        startCaptivePortal();

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
        updateDisplayState(DISPLAY_PORTAL_ACTIVE);
        startCaptivePortal();
    }

    updateDisplayState(DISPLAY_CONNECTED, WiFi.localIP().toString().c_str());
    ledIndicator(2, 60);

    xTaskCreatePinnedToCore(cameraStreamTaskTCP, "CamTCPStream", 1024 * 4, NULL, 1, NULL, 0);

    if (obstacleAvoidanceModeTaskHandle == NULL)
    {
        xTaskCreatePinnedToCore(obstacleAvoidanceMode, "ObstacleTask", 1024 * 3, NULL, 3, &obstacleAvoidanceModeTaskHandle, 1);
    }

    xTaskCreatePinnedToCore(cmdServerTask, "CmdServerTask", 1024 * 4, NULL, 2, &cmdServerTaskHandle, 1);
    ArduinoOTA.begin();
    ledIndicator(1);
}