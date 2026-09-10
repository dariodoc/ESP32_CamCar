#include "config.h"
#include "wifi_server.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include "custom_motor_driver.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <esp_camera.h>
#include "display_control.h"
#include <esp_bt.h>
#include <ArduinoOTA.h>
#include "Melodies.h"

WiFiServer server_Cmd(4000);
WiFiServer server_Camera(7000);
WebServer webServer(80);
DNSServer dnsServer;
Preferences preferences;

volatile bool videoFlag = false;
TaskHandle_t cmdServerTaskHandle = NULL;

void cmdServerTask(void *pvParameters)
{
    TickType_t lastCmdTime = xTaskGetTickCount();
    const TickType_t TIMEOUT_TICKS = pdMS_TO_TICKS(1500);

    for (;;)
    {
        WiFiClient client = server_Cmd.accept();
        if (client)
        {
            client.setTimeout(50);
            lastCmdTime = xTaskGetTickCount();

            while (client.connected())
            {
                if (client.available())
                {
                    bool hasNewMotorCmd = false;
                    int lastMotorParams[4] = {0, 0, 0, 0};

                    while (client.available())
                    {
                        String temp = client.readStringUntil('\n');
                        temp.trim();

                        if (temp.length() > 0)
                        {
                            lastCmdTime = xTaskGetTickCount();

                            String localCmd[8];
                            int localParam[8] = {0};
                            int string_length = temp.length();

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
                                // Guardamos directo los enteros extraídos sin concatenar cadenas
                                lastMotorParams[0] = localParam[1];
                                lastMotorParams[1] = localParam[2];
                                lastMotorParams[2] = localParam[3];
                                lastMotorParams[3] = localParam[4];
                                hasNewMotorCmd = true;
                            }
                        }
                    }

                    // Enviar directo a driveSafe sin pasar por sscanf
                    if (hasNewMotorCmd)
                    {
                        driveSafe(lastMotorParams[0], lastMotorParams[1], lastMotorParams[2], lastMotorParams[3]);
                    }
                }
                else
                {
                    if ((xTaskGetTickCount() - lastCmdTime) > TIMEOUT_TICKS)
                    {
                        stopAllMotors();
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(10));
            }
            client.stop();
            stopAllMotors();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void cameraStreamTaskTCP(void *pvParameters)
{
    const TickType_t FRAME_TARGET_TIME = pdMS_TO_TICKS(33); // Target: 30 FPS

    for (;;)
    {
        WiFiClient client = server_Camera.accept();
        if (client)
        {
            client.setNoDelay(true);
            client.setTimeout(3); // 🚀 Timeout agresivo de 3ms para reaccionar al instante si la red flaquea

            while (client.connected())
            {
                TickType_t startTime = xTaskGetTickCount();

                if (videoFlag)
                {
                    camera_fb_t *fb = esp_camera_fb_get();
                    if (fb)
                    {
                        uint32_t jpg_buf_len = fb->len;

                        uint8_t header[4];
                        header[0] = (uint8_t)(jpg_buf_len & 0xFF);
                        header[1] = (uint8_t)((jpg_buf_len >> 8) & 0xFF);
                        header[2] = (uint8_t)((jpg_buf_len >> 16) & 0xFF);
                        header[3] = (uint8_t)((jpg_buf_len >> 24) & 0xFF);

                        // Envío atómico del encabezado
                        if (client.write(header, 4) == 4)
                        {
                            uint8_t *buf = fb->buf;
                            size_t bytesLeft = jpg_buf_len;

                            while (bytesLeft > 0 && client.connected())
                            {
                                size_t chunkSize = (bytesLeft > 1460) ? 1460 : bytesLeft;
                                size_t written = client.write(buf, chunkSize);

                                if (written == 0)
                                {
                                    // Si la radio no pudo despachar los bytes, abortamos el frame para no congelar
                                    break;
                                }

                                buf += written;
                                bytesLeft -= written;
                            }
                        }

                        // Liberación síncrona inmediata para el DMA
                        esp_camera_fb_return(fb);
                    }
                }

                // Control de ritmo estable (Pacing)
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
            client.stop();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// 📂 MANEJADORES DEL PORTAL WEB CONFIGURADOR (CARGADOS DESDE SPIFFS)
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
    // Justo antes de WiFi.begin(...)
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
        // Al inicio de startCaptivePortal()
        updateDisplayState(DISPLAY_PORTAL_ACTIVE);
        startCaptivePortal();
    }

#ifdef DEBUG
    TelnetStream.begin();
#endif

    // Justo después de obtener la IP
    updateDisplayState(DISPLAY_CONNECTED, WiFi.localIP().toString().c_str());
    ledIndicator(2, 60);

    server_Cmd.begin(4000);
    server_Camera.begin(7000);

    xTaskCreatePinnedToCore(cmdServerTask, "CmdServerTask", 1024 * 4, NULL, 2, &cmdServerTaskHandle, 1);
    xTaskCreatePinnedToCore(cameraStreamTaskTCP, "CamTCPStream", 1024 * 4, NULL, 3, NULL, 0);

    ArduinoOTA.begin();
    ledIndicator(1);
}