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

// ----------------------------------------------------------------------
// CORE 0: STREAMING TCP (EL OJO DEL ROBOT)
// ----------------------------------------------------------------------
void cameraStreamTaskTCP(void *pvParameters)
{
    int serverFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (serverFd < 0) vTaskDelete(NULL);

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
            Serial.println("\n[VIDEO] 🟢 Cliente conectado al puerto 7000 (Video).");
            
            int nodelay = 1;
            setsockopt(clientFd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(int));

            struct timeval sendTimeout = {0, 200000};
            setsockopt(clientFd, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));

            bool wasStreaming = false; 

            while (WiFi.status() == WL_CONNECTED)
            {
                TickType_t startTime = xTaskGetTickCount();

                if (videoFlag)
                {
                    if (!wasStreaming) {
                        Serial.println("[VIDEO] 🎥 Transmisión de frames INICIADA.");
                    }
                    wasStreaming = true; 

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
                                    isValid = true; break;
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
                                int s = send(clientFd, header + hSentTotal, 4 - hSentTotal, 0);
                                if (s < 0)
                                {
                                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                                        retries++;
                                        if (retries > MAX_RETRIES) { socketError = true; break; }
                                        vTaskDelay(pdMS_TO_TICKS(20)); continue;
                                    }
                                    socketError = true; break;
                                }
                                hSentTotal += s; retries = 0;
                            }

                            if (!socketError)
                            {
                                size_t bytesWrittenTotal = 0;
                                retries = 0;
                                while (bytesWrittenTotal < fb->len)
                                {
                                    int s = send(clientFd, fb->buf + bytesWrittenTotal, fb->len - bytesWrittenTotal, 0);
                                    if (s < 0)
                                    {
                                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                                            retries++;
                                            if (retries > MAX_RETRIES) { socketError = true; break; }
                                            vTaskDelay(pdMS_TO_TICKS(20)); continue;
                                        }
                                        socketError = true; break;
                                    }
                                    bytesWrittenTotal += s; retries = 0;
                                }
                            }

                            if (socketError)
                            {
                                Serial.println("[VIDEO] ⚠️ Saturación de red severa. Cortando conexión de video.");
                                esp_camera_fb_return(fb);
                                break; 
                            }
                        }
                        esp_camera_fb_return(fb);
                    }
                }
                else
                {
                    if (wasStreaming) {
                        // 🚀 LA MAGIA: Solo reiniciamos la bandera, NO rompemos el bucle ni cerramos el puerto
                        Serial.println("[VIDEO] 🛑 CMD_VIDEO 0 detectado. Pausando stream (Modo Mute)...");
                        wasStreaming = false; 
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                }

                if (videoFlag) 
                {
                    TickType_t elapsedTime = xTaskGetTickCount() - startTime;
                    if (elapsedTime < FRAME_TARGET_TIME) {
                        vTaskDelay(FRAME_TARGET_TIME - elapsedTime);
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(15)); 
                    }
                }
            }

            videoFlag = false;
            close(clientFd); 
            Serial.println("[VIDEO] 🔴 Puerto 7000 cerrado y libre.");
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
    if (serverFd < 0) vTaskDelete(NULL);

    int enable = 1;
    setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(4000);

    if (bind(serverFd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        close(serverFd); vTaskDelete(NULL);
    }
    listen(serverFd, 1);

    TickType_t lastCmdTime = xTaskGetTickCount();
    const TickType_t TIMEOUT_TICKS = pdMS_TO_TICKS(1500); 

    for (;;)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            stopAllMotors(); vTaskDelay(pdMS_TO_TICKS(500)); continue;
        }

        struct sockaddr_in clientAddr;
        socklen_t clientAddrLen = sizeof(clientAddr);
        int clientFd = accept(serverFd, (struct sockaddr *)&clientAddr, &clientAddrLen);

        if (clientFd >= 0)
        {
            Serial.println("\n[CMD] 🟢 Cliente conectado al puerto 4000 (Comandos).");

            int nodelay = 1;
            setsockopt(clientFd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(int));

            struct timeval recvTimeout = {0, 100000};
            setsockopt(clientFd, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(recvTimeout));

            int keepAlive = 1, keepIdle = 2, keepInterval = 1, keepCount = 3;
            setsockopt(clientFd, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
            setsockopt(clientFd, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
            setsockopt(clientFd, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
            setsockopt(clientFd, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

            lastCmdTime = xTaskGetTickCount();
            String rxBuffer = "";
            rxBuffer.reserve(512);
            char tempChunk[128];
            bool motorsStoppedByTimeout = false;

            while (WiFi.status() == WL_CONNECTED)
            {
                int bytesRead = recv(clientFd, tempChunk, sizeof(tempChunk) - 1, 0);

                if (bytesRead > 0)
                {
                    tempChunk[bytesRead] = '\0';
                    rxBuffer += tempChunk;
                    
                    if (rxBuffer.length() > 512) { rxBuffer = ""; }

                    int newLineIdx;
                    while ((newLineIdx = rxBuffer.indexOf('\n')) >= 0)
                    {
                        String line = rxBuffer.substring(0, newLineIdx);
                        rxBuffer = rxBuffer.substring(newLineIdx + 1);
                        line.trim();

                        if (line.length() > 0)
                        {
                            lastCmdTime = xTaskGetTickCount(); 
                            motorsStoppedByTimeout = false; 

                            String localCmd[8];
                            int localParam[8] = {0};
                            int string_length = line.length();
                            String temp = line;

                            for (int i = 0; i < 8; i++)
                            {
                                int index = temp.indexOf('#');
                                if (index < 0) {
                                    if (string_length > 0) { localCmd[i] = temp; localParam[i] = temp.toInt(); }
                                    break;
                                } else {
                                    string_length -= index;
                                    localCmd[i] = temp.substring(0, index);
                                    localParam[i] = localCmd[i].toInt();
                                    temp = temp.substring(index + 1);
                                }
                            }

                            if (localCmd[0] == "CMD_SERVO") {
                                if (localParam[1] == 0) setPanAngle(localParam[2]);
                                else if (localParam[1] == 1) setTiltAngle(localParam[2]);
                            }
                            else if (localCmd[0] == "CMD_CAMERA") {
                                if (localParam[1] == panCenter && localParam[2] == tiltCenter) centerServos();
                                else { setPanAngle(localParam[1]); setTiltAngle(localParam[2]); }
                            }
                            else if (localCmd[0] == "CMD_VIDEO") {
                                videoFlag = (localParam[1] == 1);
                                // Telemetría removida para evitar spam
                            }
                            else if (localCmd[0] == "CMD_BUZZER") {
                                if (localParam[1] == 1 && localParam[2] > 0) toneToPlay(buzzerPin, buzzerChannel, localParam[2], 100);
                                else ledcWriteTone(buzzerChannel, 0);
                            }
                            else if (localCmd[0] == "CMD_LIGHT") {
                                enableLaser = (localParam[1] == 1); turnLaserOn(enableLaser);
                            }
                            else if (localCmd[0] == "CMD_LED_MOD") {
                                if (localParam[1] == 2) {
                                    enableObstacleAvoidance = true;
                                    if (obstacleAvoidanceModeTaskHandle != NULL) xTaskNotifyGive(obstacleAvoidanceModeTaskHandle);
                                } else enableObstacleAvoidance = false;
                            }
                            else if (localCmd[0] == "CMD_MOTOR") {
                                driveSafe(localParam[1], localParam[2], localParam[3], localParam[4]);
                            }
                        }
                    }
                }
                else if (bytesRead == 0)
                {
                    Serial.println("[CMD] ℹ️ Conexión cerrada normalmente por la app.");
                    break; 
                }
                else
                {
                    if (errno != EWOULDBLOCK && errno != EAGAIN)
                    {
                        Serial.printf("[CMD] ❌ Socket roto. errno: %d\n", errno);
                        if (errno == 113 || errno == 104 || errno == 128) 
                        {
                            Serial.println("🚨 RED MUERTA. REINICIO DE EMERGENCIA.");
                            stopAllMotors(); vTaskDelay(pdMS_TO_TICKS(1000)); ESP.restart();
                        }
                        break; 
                    }
                }

                TickType_t timeSinceLastCmd = xTaskGetTickCount() - lastCmdTime;
                
                if (timeSinceLastCmd > TIMEOUT_TICKS) {
                    if (!motorsStoppedByTimeout) {
                        Serial.println("[CMD] ⏱️ Motores detenidos preventivamente.");
                        motorsStoppedByTimeout = true;
                    }
                    stopAllMotors();
                }

                if (timeSinceLastCmd > pdMS_TO_TICKS(60000)) {
                    Serial.println("🚨 60s sin actividad. Reiniciando por seguridad...");
                    stopAllMotors(); vTaskDelay(pdMS_TO_TICKS(1000)); ESP.restart(); 
                }

                vTaskDelay(pdMS_TO_TICKS(5));
            }

            stopAllMotors();
            close(clientFd); 
            Serial.println("[CMD] 🔴 Puerto 4000 cerrado y libre.");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ----------------------------------------------------------------------
// GESTIÓN DE CONFIGURACIÓN Y PORTAL CAUTIVO
// ----------------------------------------------------------------------
void handleRoot()
{
    if (SPIFFS.exists("/wifimanager.html")) {
        File file = SPIFFS.open("/wifimanager.html", "r");
        webServer.streamFile(file, "text/html; charset=utf-8"); file.close();
    } else {
        webServer.send(404, "text/plain", "Archivo wifimanager.html no encontrado");
    }
}

void handleCSS()
{
    if (SPIFFS.exists("/wifimanager.css")) {
        File file = SPIFFS.open("/wifimanager.css", "r");
        webServer.streamFile(file, "text/css"); file.close();
    } else {
        webServer.send(404, "text/plain", "CSS no encontrado");
    }
}

void handleSave()
{
    String reqSSID = webServer.arg("ssid"); String reqPASS = webServer.arg("pass");
    String reqIP = webServer.arg("ip"); String reqGW = webServer.arg("gateway");

    preferences.begin("wifi_config", false);
    preferences.putString("ssid", reqSSID); preferences.putString("pass", reqPASS);
    preferences.putString("ip", reqIP); preferences.putString("gw", reqGW);
    preferences.end();

    webServer.send(200, "text/html", "<html><body><h1>Guardado!</h1></body></html>");
    delay(2000); ESP.restart();
}

void startCaptivePortal()
{
    if (!SPIFFS.begin(true)) { Serial.println("❌ Fallo SPIFFS"); }

    WiFi.mode(WIFI_AP); WiFi.softAP("ESP-CAMERA-CAR", "carbondioxide");
    IPAddress apIP(192, 168, 4, 1); WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    dnsServer.start(53, "*", apIP);

    webServer.on("/", handleRoot); webServer.on("/wifimanager.css", handleCSS);
    webServer.on("/save", HTTP_POST, handleSave); webServer.onNotFound(handleRoot);
    webServer.begin();

    int currentLedState = 0; TickType_t lastBlink = xTaskGetTickCount();
    while (true) {
        dnsServer.processNextRequest(); webServer.handleClient();
        if ((xTaskGetTickCount() - lastBlink) >= pdMS_TO_TICKS(500)) {
            currentLedState = !currentLedState; ledIndicator(currentLedState);
            lastBlink = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void initWiFi()
{
    ledIndicator(0);
    WiFi.persistent(false); WiFi.setSleep(WIFI_PS_NONE); WiFi.setTxPower(WIFI_POWER_19_5dBm);
    btStop(); esp_bt_controller_disable();

    preferences.begin("wifi_config", true);
    String storedSSID = preferences.getString("ssid", ""); String storedPASS = preferences.getString("pass", "");
    String storedIP = preferences.getString("ip", ""); String storedGW = preferences.getString("gw", "");
    preferences.end();

    if (storedSSID.length() == 0) startCaptivePortal();

    IPAddress staticIP, gateway, subnet(255, 255, 255, 0), dns(8, 8, 8, 8);
    if (storedIP.length() > 0 && storedGW.length() > 0) {
        staticIP.fromString(storedIP); gateway.fromString(storedGW);
        WiFi.config(staticIP, gateway, subnet, dns);
    }

    WiFi.mode(WIFI_STA); WiFi.setAutoReconnect(true);
    updateDisplayState(DISPLAY_CONNECTING_WIFI, storedSSID.c_str());
    WiFi.begin(storedSSID.c_str(), storedPASS.c_str());

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 12) {
        ledIndicator(1, 80); vTaskDelay(pdMS_TO_TICKS(1000)); attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        updateDisplayState(DISPLAY_PORTAL_ACTIVE); startCaptivePortal();
    }

    updateDisplayState(DISPLAY_CONNECTED, WiFi.localIP().toString().c_str());
    ledIndicator(2, 60);

    xTaskCreatePinnedToCore(cameraStreamTaskTCP, "CamTCPStream", 1024 * 4, NULL, 1, NULL, 0);

    if (obstacleAvoidanceModeTaskHandle == NULL) {
        xTaskCreatePinnedToCore(obstacleAvoidanceMode, "ObstacleTask", 1024 * 3, NULL, 3, &obstacleAvoidanceModeTaskHandle, 1);
    }

    xTaskCreatePinnedToCore(cmdServerTask, "CmdServerTask", 1024 * 4, NULL, 2, &cmdServerTaskHandle, 1);
    ArduinoOTA.begin(); ledIndicator(1);
}