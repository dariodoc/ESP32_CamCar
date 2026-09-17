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
    serverAddr.sin_port = htons(7000);

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
            Serial.println("\n[VIDEO] 🟢 Cliente conectado al puerto 7000 (Video).");
#endif

            int nodelay = 1;
            setsockopt(clientFd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(int));

            struct timeval sendTimeout = {0, 200000};
            setsockopt(clientFd, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));

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

                TickType_t startTime = xTaskGetTickCount();

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

                            bool socketError = false;
                            int retries = 0;
                            const int MAX_RETRIES = 10;

                            int hSentTotal = 0;
                            while (hSentTotal < 4)
                            {
                                int s = send(clientFd, header + hSentTotal, 4 - hSentTotal, MSG_NOSIGNAL);
                                if (s < 0)
                                {
                                    if (errno == EAGAIN || errno == EWOULDBLOCK)
                                    {
                                        retries++;
                                        if (retries > MAX_RETRIES)
                                        {
                                            socketError = true;
                                            break;
                                        }
                                        vTaskDelay(pdMS_TO_TICKS(20));
                                        continue;
                                    }
                                    socketError = true;
                                    break;
                                }
                                hSentTotal += s;
                                retries = 0;
                            }

                            if (!socketError)
                            {
                                size_t bytesWrittenTotal = 0;
                                retries = 0;
                                while (bytesWrittenTotal < fb->len)
                                {
                                    int s = send(clientFd, fb->buf + bytesWrittenTotal, fb->len - bytesWrittenTotal, MSG_NOSIGNAL);
                                    if (s < 0)
                                    {
                                        if (errno == EAGAIN || errno == EWOULDBLOCK)
                                        {
                                            retries++;
                                            if (retries > MAX_RETRIES)
                                            {
                                                socketError = true;
                                                break;
                                            }
                                            vTaskDelay(pdMS_TO_TICKS(20));
                                            continue;
                                        }
                                        socketError = true;
                                        break;
                                    }
                                    bytesWrittenTotal += s;
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

                if (videoFlag)
                {
                    TickType_t elapsedTime = xTaskGetTickCount() - startTime;
                    if (elapsedTime < FRAME_TARGET_TIME)
                    {
                        vTaskDelay(FRAME_TARGET_TIME - elapsedTime);
                    }
                    else
                    {
                        vTaskDelay(pdMS_TO_TICKS(15));
                    }
                }
            }

            videoFlag = false;
            close(clientFd);
#ifdef DEBUG
            Serial.println("[VIDEO] 🔴 Puerto 7000 cerrado y libre.");
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
    serverAddr.sin_port = htons(4000);

    if (bind(serverFd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        close(serverFd);
        vTaskDelete(NULL);
    }
    listen(serverFd, 1);

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

        if (clientFd >= 0)
        {
#ifdef DEBUG
            Serial.println("\n[CMD] 🟢 Cliente conectado al puerto 4000 (Comandos).");
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

            char rxBuffer[512];
            int rxIndex = 0;
            char tempChunk[128];

            if (dmsTimer != NULL) xTimerStart(dmsTimer, 0);

            while (WiFi.status() == WL_CONNECTED)
            {
                int bytesRead = recv(clientFd, tempChunk, sizeof(tempChunk) - 1, 0);

                if (bytesRead > 0)
                {
                    tempChunk[bytesRead] = '\0';
                    
                    // Añadir al buffer estático circular
                    for (int i = 0; i < bytesRead; i++) {
                        if (rxIndex < 511) {
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
                            if (dmsTimer != NULL) xTimerReset(dmsTimer, 0);

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
                                        setPanAngle(localParam[2]);
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
                                    if (localParam[1] == 1 && localParam[2] > 0)
                                        toneToPlay(buzzerPin, buzzerChannel, localParam[2], 100);
                                    else
                                        ledcWriteTone(buzzerChannel, 0);
                                }
                                else if (strcmp(localCmd[0], "CMD_LIGHT") == 0)
                                {
                                    enableLaser = (localParam[1] == 1);
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
                                        enableObstacleAvoidance = false;
                                }
                                else if (strcmp(localCmd[0], "CMD_MOTOR") == 0)
                                {
                                    driveSafe(localParam[1], localParam[2], localParam[3], localParam[4]);
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
                            Serial.println("🚨 RED MUERTA. REINICIO DE EMERGENCIA.");
#endif
                            stopAllMotors();
                            vTaskDelay(pdMS_TO_TICKS(1000));
                            ESP.restart();
                        }
                        break;
                    }
                }

                // El Dead Man's Switch (DMS Timer) y la Máquina de Estados WiFi 
                // se encargan ahora de la seguridad en segundo plano de manera autónoma.

                vTaskDelay(pdMS_TO_TICKS(5));
            }

            stopAllMotors();
            close(clientFd);
            if (dmsTimer != NULL) xTimerStop(dmsTimer, 0);
#ifdef DEBUG
            Serial.println("[CMD] 🔴 Puerto 4000 cerrado y libre.");
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
    }
    else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
    {
        // Solo reiniciamos si ya estábamos conectados y se cayó la red
        if (isWiFiConnected) 
        {
#ifdef DEBUG
            Serial.println("🚨 EVENTO WIFI: Desconectado. Cortando motores inmediatamente...");
#endif
            stopAllMotors();
            updateDisplayState(DISPLAY_PORTAL_ACTIVE);
            ESP.restart(); 
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
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
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
    WiFi.setAutoReconnect(true);
    updateDisplayState(DISPLAY_CONNECTING_WIFI, storedSSID.c_str());
    WiFi.begin(storedSSID.c_str(), storedPASS.c_str());

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