#include "config.h"
#include "wifi_server.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include "custom_motor_driver.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <esp_camera.h>
#include "esp_bt.h"
#include <ArduinoOTA.h>
#include "Melodies.h"

WiFiServer server_Cmd(4000);
WiFiServer server_Camera(7000);

volatile bool videoFlag = false;
TaskHandle_t cmdServerTaskHandle = NULL;

static int lastFL = -9999, lastBL = -9999, lastFR = -9999, lastBR = -9999;

int mapMotorValue(int rawValue)
{
    if (rawValue == 0)
        return 0;

    const int MIN_PWM = 800;
    const int MAX_PWM = 4095;

    int sign = (rawValue > 0) ? 1 : -1;
    int absVal = abs(rawValue);

    if (absVal <= 210)
        return MIN_PWM * sign;
    if (absVal >= 4095)
        return MAX_PWM * sign;

    absVal = constrain(absVal, 210, 4095);
    int mappedPWM = map(absVal, 210, 4095, MIN_PWM, MAX_PWM);

    return mappedPWM * sign;
}

void cmdServerTask(void *pvParameters)
{
    TickType_t lastCmdTime = xTaskGetTickCount();
    // 🚀 MARGEN DE SEGURIDAD AMPLIADO: 1500ms para evitar falsos frenados al sostener la posición
    const TickType_t TIMEOUT_TICKS = pdMS_TO_TICKS(1500);

    for (;;)
    {
        WiFiClient client = server_Cmd.accept();
        if (client)
        {
            client.setTimeout(50);
            lastCmdTime = xTaskGetTickCount();

#ifdef DEBUG
            TelnetStream.println("🕹️ Cliente de Comandos conectado (Puerto 4000)");
#endif
            while (client.connected())
            {
                if (client.available())
                {
                    String lastInputString = "";
                    bool zeroBrakeFound = false;

                    // Inspección inteligente del búfer TCP
                    while (client.available())
                    {
                        String temp = client.readStringUntil('\n');
                        temp.trim();

                        if (temp.length() > 0)
                        {
                            // Si en la cola TCP aparece un comando explícito de freno, lo priorizamos
                            if (temp.startsWith("CMD_MOTOR#0#0#0#0"))
                            {
                                zeroBrakeFound = true;
                                lastInputString = temp;
                                break; // Prioridad absoluta: procesar freno inmediatamente
                            }
                            lastInputString = temp; // De lo contrario, conservamos la trama más reciente
                        }
                    }

                    if (lastInputString.length() > 0)
                    {
                        lastCmdTime = xTaskGetTickCount(); // Reset del reloj de seguridad al recibir datos

                        String localCmd[8];
                        int localParam[8] = {0};

                        int string_length = lastInputString.length();
                        for (int i = 0; i < 8; i++)
                        {
                            int index = lastInputString.indexOf('#');
                            if (index < 0)
                            {
                                if (string_length > 0)
                                {
                                    localCmd[i] = lastInputString;
                                    localParam[i] = lastInputString.toInt();
                                }
                                break;
                            }
                            else
                            {
                                string_length -= index;
                                localCmd[i] = lastInputString.substring(0, index);
                                localParam[i] = localCmd[i].toInt();
                                lastInputString = lastInputString.substring(index + 1);
                            }
                        }

                        // 1. Control de Motores
                        if (localCmd[0] == "CMD_MOTOR")
                        {
                            int safeFL = mapMotorValue(localParam[1]);
                            int safeBL = mapMotorValue(localParam[2]);
                            int safeFR = mapMotorValue(localParam[3]);
                            int safeBR = mapMotorValue(localParam[4]);

                            // Si se detectó el freno (0,0,0,0) en cualquier parte del búfer
                            if (zeroBrakeFound || (safeFL == 0 && safeBL == 0 && safeFR == 0 && safeBR == 0))
                            {
                                brakeAllMotors();
                                setStandbyPin(false);
                                lastFL = lastBL = lastFR = lastBR = 0;
                            }
                            else if (safeFL != lastFL || safeBL != lastBL || safeFR != lastFR || safeBR != lastBR)
                            {
                                driveDirectRaw(safeFL, safeBL, safeFR, safeBR);
                                lastFL = safeFL;
                                lastBL = safeBL;
                                lastFR = safeFR;
                                lastBR = safeBR;
                            }
                        }
                        else if (localCmd[0] == "CMD_SERVO")
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
                    }
                }
                else
                {
                    // Watchdog de seguridad ampliado a 1.5s
                    if ((xTaskGetTickCount() - lastCmdTime) > TIMEOUT_TICKS)
                    {
                        if (lastFL != 0 || lastBL != 0 || lastFR != 0 || lastBR != 0)
                        {
                            brakeAllMotors();
                            setStandbyPin(false);
                            lastFL = lastBL = lastFR = lastBR = 0;
                        }
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(10));
            }
            client.stop();

            brakeAllMotors();
            setStandbyPin(false);
            lastFL = lastBL = lastFR = lastBR = -9999;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void cameraStreamTaskTCP(void *pvParameters)
{
    for (;;)
    {
        WiFiClient client = server_Camera.accept();
        if (client)
        {
            while (client.connected())
            {
                if (videoFlag)
                {
                    camera_fb_t *fb = esp_camera_fb_get();
                    if (fb)
                    {
                        uint32_t jpg_buf_len = fb->len;
                        uint8_t *jpg_buf = fb->buf;

                        uint8_t slen[4];
                        slen[0] = (uint8_t)(jpg_buf_len & 0xFF);
                        slen[1] = (uint8_t)((jpg_buf_len >> 8) & 0xFF);
                        slen[2] = (uint8_t)((jpg_buf_len >> 16) & 0xFF);
                        slen[3] = (uint8_t)((jpg_buf_len >> 24) & 0xFF);

                        client.write(slen, 4);
                        client.write(jpg_buf, jpg_buf_len);
                        esp_camera_fb_return(fb);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(35));
            }
            client.stop();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void initWiFi()
{
    ledIndicator(0);

    WiFi.persistent(false);
    WiFi.setSleep(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    IPAddress staticIP(192, 168, 0, 202);
    IPAddress gateway(192, 168, 0, 1);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress dns(8, 8, 8, 8);

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    WiFi.config(staticIP, gateway, subnet, dns);
    WiFi.begin("Tractorex", "9983476198");

    while (WiFi.status() != WL_CONNECTED)
    {
        ledIndicator(1, 80);
        vTaskDelay(pdMS_TO_TICKS(1920));

        if (WiFi.status() == WL_CONNECT_FAILED || WiFi.status() == WL_DISCONNECTED)
        {
            WiFi.begin("Tractorex", "9983476198");
        }
    }

#ifdef DEBUG
    TelnetStream.begin();
#endif

    ledIndicator(2, 60);

    server_Cmd.begin(4000);
    server_Camera.begin(7000);

    xTaskCreatePinnedToCore(cmdServerTask, "CmdServerTask", 1024 * 4, NULL, 2, &cmdServerTaskHandle, 1);
    xTaskCreatePinnedToCore(cameraStreamTaskTCP, "CamTCPStream", 1024 * 4, NULL, 1, NULL, 0);

    ArduinoOTA.begin();

    btStop();
    esp_bt_controller_disable();

    ledIndicator(1);
}