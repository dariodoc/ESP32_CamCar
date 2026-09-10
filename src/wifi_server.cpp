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

// 🚀 LIBRERÍAS DE SOCKETS POSIX NATIVOS DE LwIP
#include <lwip/sockets.h>
#include <lwip/netdb.h>

WebServer webServer(80);
DNSServer dnsServer;
Preferences preferences;

volatile bool videoFlag = false;
TaskHandle_t cmdServerTaskHandle = NULL;

// ----------------------------------------------------------------------
// 🚀 PUNTO 2: ESTRUCTURA PARA DESACOPLAR LA CÁMARA DE LA RED
// ----------------------------------------------------------------------
static camera_fb_t *latestFrame = NULL;
static SemaphoreHandle_t frameMutex = NULL;

void cameraCaptureTask(void *pvParameters)
{
    const TickType_t FRAME_TARGET_TIME = pdMS_TO_TICKS(40); // Target ~25 FPS para liberar ciclos de CPU

    for (;;)
    {
        TickType_t startTime = xTaskGetTickCount();

        // Capturar frame solo si el video está activo
        if (videoFlag)
        {
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb)
            {
                if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(5)) == pdTRUE)
                {
                    if (latestFrame != NULL)
                    {
                        esp_camera_fb_return(latestFrame);
                    }
                    latestFrame = fb;
                    xSemaphoreGive(frameMutex);
                }
                else
                {
                    esp_camera_fb_return(fb);
                }
            }
        }

        TickType_t elapsedTime = xTaskGetTickCount() - startTime;
        if (elapsedTime < FRAME_TARGET_TIME)
        {
            vTaskDelay(FRAME_TARGET_TIME - elapsedTime);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10)); // Ceder explícitamente CPU a otras tareas
        }
    }
}

// ----------------------------------------------------------------------
// 🚀 PUNTO 3: TARES DE SERVIDOR CON SOCKETS NATIVOS POSIX
// ----------------------------------------------------------------------

void cmdServerTask(void *pvParameters)
{
    int serverFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (serverFd < 0)
        return;

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
        return;
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
            // 🚀 ACTIVAR TCP_NODELAY: Envío y recepción inmediata sin algoritmo de Nagle
            int nodelay = 1;
            setsockopt(clientFd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(int));

            // Timeout de recepción ultrarrápido (1 ms) para no frenar la tarea
            struct timeval recvTimeout;
            recvTimeout.tv_sec = 0;
            recvTimeout.tv_usec = 1000; // 1 ms
            setsockopt(clientFd, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(recvTimeout));

            int keepAlive = 1, keepIdle = 2, keepInterval = 1, keepCount = 2;
            setsockopt(clientFd, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
            setsockopt(clientFd, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
            setsockopt(clientFd, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
            setsockopt(clientFd, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

            lastCmdTime = xTaskGetTickCount();
            String rxBuffer = "";
            rxBuffer.reserve(256);

            char tempChunk[128];

            while (WiFi.status() == WL_CONNECTED)
            {
                // 🚀 LECTURA EN RÁFARGA: Lee hasta 128 bytes de un solo golpe
                int bytesRead = recv(clientFd, tempChunk, sizeof(tempChunk) - 1, 0);

                if (bytesRead > 0)
                {
                    tempChunk[bytesRead] = '\0'; // Asegurar fin de cadena C
                    rxBuffer += tempChunk;

                    // Procesar todas las líneas completas recibidas en el buffer
                    int newLineIdx;
                    while ((newLineIdx = rxBuffer.indexOf('\n')) >= 0)
                    {
                        String line = rxBuffer.substring(0, newLineIdx);
                        rxBuffer = rxBuffer.substring(newLineIdx + 1);
                        line.trim();

                        if (line.length() > 0)
                        {
                            lastCmdTime = xTaskGetTickCount();

                            String localCmd[8];
                            int localParam[8] = {0};
                            int string_length = line.length();
                            String temp = line;

                            for (int i = 0; i < 8; i++)
                            {
                                int index = temp.indexOf('#');
                                if (index < 0)
                                {
                                    if (string_length > 0)
                                    {
                                        localCmd[i] = temp;
                                        localParam[i] = temp.toInt();
                                    }
                                    break;
                                }
                                else
                                {
                                    string_length -= index;
                                    localCmd[i] = temp.substring(0, index);
                                    localParam[i] = localCmd[i].toInt();
                                    temp = temp.substring(index + 1);
                                }
                            }

                            if (localCmd[0] == "CMD_SERVO")
                            {
                                if (localParam[1] == 0)
                                    setPanAngle(localParam[2]);
                                else if (localParam[1] == 1)
                                    setTiltAngle(localParam[2]);
                            }
                            else if (localCmd[0] == "CMD_CAMERA")
                            {
                                if (localParam[1] == panCenter && localParam[2] == tiltCenter)
                                    centerServos();
                                else
                                {
                                    setPanAngle(localParam[1]);
                                    setTiltAngle(localParam[2]);
                                }
                            }
                            else if (localCmd[0] == "CMD_VIDEO")
                            {
                                videoFlag = (localParam[1] == 1);
                            }
                            else if (localCmd[0] == "CMD_BUZZER")
                            {
                                if (localParam[1] == 1 && localParam[2] > 0)
                                    toneToPlay(buzzerPin, buzzerChannel, localParam[2], 100);
                                else
                                    ledcWriteTone(buzzerChannel, 0);
                            }
                            else if (localCmd[0] == "CMD_LIGHT")
                            {
                                enableLaser = (localParam[1] == 1);
                                turnLaserOn(enableLaser);
                            }
                            else if (localCmd[0] == "CMD_LED_MOD")
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
                            else if (localCmd[0] == "CMD_MOTOR")
                            {
                                driveSafe(localParam[1], localParam[2], localParam[3], localParam[4]);
                            }
                        }
                    }
                }
                else if (bytesRead == 0)
                {
                    break; // Cliente cerrado
                }
                else
                {
                    if (errno != EWOULDBLOCK && errno != EAGAIN)
                    {
                        break; // Error real de red
                    }
                }

                // Paro de seguridad por inactividad
                if ((xTaskGetTickCount() - lastCmdTime) > TIMEOUT_TICKS)
                {
                    stopAllMotors();
                }

                vTaskDelay(pdMS_TO_TICKS(5)); // Delay mínimo para respuesta en tiempo real
            }

            stopAllMotors();
            shutdown(clientFd, SHUT_RDWR);
            close(clientFd);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void cameraStreamTaskTCP(void *pvParameters)
{
    int serverFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (serverFd < 0)
        return;

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
        return;
    }
    listen(serverFd, 1);

    const TickType_t FRAME_TARGET_TIME = pdMS_TO_TICKS(33);

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
            int nodelay = 1;
            setsockopt(clientFd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(int));

            // 🚀 TIMEOUT DE ESCRITURA ESTRICTO DE 100ms
            struct timeval sendTimeout;
            sendTimeout.tv_sec = 0;
            sendTimeout.tv_usec = 100000; // 100 ms max para despachar frame
            setsockopt(clientFd, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));

            int keepAlive = 1, keepIdle = 2, keepInterval = 1, keepCount = 2;
            setsockopt(clientFd, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
            setsockopt(clientFd, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
            setsockopt(clientFd, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
            setsockopt(clientFd, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

            while (WiFi.status() == WL_CONNECTED)
            {
                TickType_t startTime = xTaskGetTickCount();

                if (videoFlag)
                {
                    camera_fb_t *fbToSend = NULL;

                    // Extraer de forma segura la foto del Punto 2
                    if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                    {
                        fbToSend = latestFrame;
                        latestFrame = NULL; // Asignamos posesión del pointer
                        xSemaphoreGive(frameMutex);
                    }

                    if (fbToSend)
                    {
                        uint32_t jpg_buf_len = fbToSend->len;

                        uint8_t header[4];
                        header[0] = (uint8_t)(jpg_buf_len & 0xFF);
                        header[1] = (uint8_t)((jpg_buf_len >> 8) & 0xFF);
                        header[2] = (uint8_t)((jpg_buf_len >> 16) & 0xFF);
                        header[3] = (uint8_t)((jpg_buf_len >> 24) & 0xFF);

                        // Envío nativo POSIX
                        int sentHeader = send(clientFd, header, 4, 0);

                        if (sentHeader == 4)
                        {
                            int sentBody = send(clientFd, fbToSend->buf, jpg_buf_len, 0);

                            // Si el envío falla o expira el timeout de 100ms, abortamos
                            if (sentBody < 0)
                            {
                                esp_camera_fb_return(fbToSend);
                                break;
                            }
                        }
                        else
                        {
                            esp_camera_fb_return(fbToSend);
                            break;
                        }

                        // Liberación de la memoria DMA
                        esp_camera_fb_return(fbToSend);
                    }
                }

                TickType_t elapsedTime = xTaskGetTickCount() - startTime;
                if (elapsedTime < FRAME_TARGET_TIME)
                {
                    vTaskDelay(FRAME_TARGET_TIME - elapsedTime);
                }
                else
                {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            }

            // 🚀 PURGA ATÓMICA DE CÁMARA Y RED
            shutdown(clientFd, SHUT_RDWR);
            close(clientFd);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ----------------------------------------------------------------------
// 📂 MANEJADORES DEL PORTAL WEB Y CONFIGURACIÓN DE RED
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
        webServer.send(404, "text/plain", "Archivo wifimanager.html no encontrado en data/");
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

    webServer.send(200, "text/html", "<html><body><h1>Configuracion guardada!</h1><p>El robot se reiniciara para conectarse a " + reqSSID + "...</p></body></html>");
    delay(2000);
    ESP.restart();
}

void startCaptivePortal()
{
    if (!SPIFFS.begin(true))
    {
#ifdef DEBUG
        TelnetStream.println("❌ Fallo al montar SPIFFS\r");
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

void initWiFi()
{
    ledIndicator(0);

    // Inicializar Mutex para el búfer de fotos desacoplado
    frameMutex = xSemaphoreCreateMutex();

    WiFi.persistent(false);
    WiFi.setSleep(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    btStop();
    esp_bt_controller_disable();

    preferences.begin("wifi_config", true);
    String storedSSID = preferences.getString("ssid", "");
    String storedPASS = preferences.getString("pass", "");
    String storedIP = preferences.getString("ip", "");
    String storedGW = preferences.getString("gw", "");
    preferences.end();

    if (storedSSID.length() == 0)
    {
        startCaptivePortal();
    }

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

#ifdef DEBUG
    TelnetStream.begin();
#endif

    updateDisplayState(DISPLAY_CONNECTED, WiFi.localIP().toString().c_str());
    ledIndicator(2, 60);

    // 🚀 LER TAREA INDEPENDIENTE DE CAPTURA DE CÁMARA (Core 0, Prioridad 4)
    xTaskCreatePinnedToCore(cameraCaptureTask, "CamCaptureTask", 1024 * 3, NULL, 1, NULL, 0);

    // 🚀 CREAR TAREAS DE SERVIDORES DE RED POSIX
    xTaskCreatePinnedToCore(cmdServerTask, "CmdServerTask", 1024 * 4, NULL, 2, &cmdServerTaskHandle, 1);
    xTaskCreatePinnedToCore(cameraStreamTaskTCP, "CamTCPStream", 1024 * 4, NULL, 3, NULL, 0);

    ArduinoOTA.begin();
    ledIndicator(1);
}