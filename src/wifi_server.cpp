#include "config.h"
#include "wifi_server.h"
#include "camera_setup.h"
#include "motor_control.h"
#include "peripherals.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <esp_camera.h>
#include "esp_bt.h"
#include <ArduinoOTA.h>
#include "Melodies.h"

WiFiServer server_Cmd(4000);
WiFiServer server_Camera(7000);

String CmdArray[8];
int paramters[8];
volatile bool videoFlag = false;

void Get_Command(String inputStringTemp)
{
    int string_length = inputStringTemp.length();
    for (int i = 0; i < 8; i++)
    {
        int index = inputStringTemp.indexOf('#');
        if (index < 0)
        {
            if (string_length > 0)
            {
                CmdArray[i] = inputStringTemp;
                paramters[i] = inputStringTemp.toInt();
            }
            break;
        }
        else
        {
            string_length -= index;
            CmdArray[i] = inputStringTemp.substring(0, index);
            paramters[i] = CmdArray[i].toInt();
            inputStringTemp = inputStringTemp.substring(index + 1);
        }
    }
}

void loopCmdServer()
{
    WiFiClient client = server_Cmd.accept();
    if (client)
    {
#ifdef DEBUG
        Serial.println("🕹️ Cliente de Comandos conectado (Puerto 4000)");
#endif
        while (client.connected())
        {
            if (client.available())
            {
                String inputStringTemp = client.readStringUntil('\n');
                inputStringTemp.trim();

                Get_Command(inputStringTemp);

                // --- MAPPING DE COMANDOS APP FREENOVE A TU HARDWARE --- //

                // 1. Control de Motores (TB6612FNG via PCF8574 + PWM)
                if (CmdArray[0] == "CMD_MOTOR")
                {
                    int rawLeft = paramters[1]; // Rango app: -4095 a 4095
                    int rawRight = paramters[3];

                    float normX = 0.0f;
                    float normY = (float)rawLeft / 4095.0f;

                    if (rawLeft != rawRight)
                    {
                        normX = (float)(rawLeft - rawRight) / 4095.0f;
                    }

                    processDifferentialDrive(normX, normY);
                }

                // 2. Control de Servos Pan / Tilt (Mapeo directo desde la App Freenove)
                if (CmdArray[0] == "CMD_SERVO")
                {
                    int servoID = paramters[1]; // 0 = Pan, 1 = Tilt
                    int angle = paramters[2];   // Ángulo enviado por la App (0 - 180)

                    if (servoID == 0)
                    {
                        setPanAngle(angle);
                    }
                    else if (servoID == 1)
                    {
                        setTiltAngle(angle);
                    }
                }
                else if (CmdArray[0] == "CMD_CAMERA")
                {
                    // La App envía CMD_CAMERA#panAngle#tiltAngle al mover el joystick de la cámara
                    if (paramters[1] == panCenter && paramters[2] == tiltCenter)
                    {
                        centerServos();
                    }
                    else
                    {
                        setPanAngle(paramters[1]);
                        setTiltAngle(paramters[2]);
                    }
                }

                // 3. Control de Streaming de Video
                if (CmdArray[0] == "CMD_VIDEO")
                {
                    videoFlag = (paramters[1] == 1);
                }

                // 4. Buzzer Pasivo
                if (CmdArray[0] == "CMD_BUZZER")
                {
                    bool enable = (paramters[1] == 1);
                    int freq = paramters[2];
                    if (enable && freq > 0)
                    {
                        toneToPlay(buzzerPin, buzzerChannel, freq, 100);
                    }
                    else
                    {
                        ledcWriteTone(buzzerChannel, 0);
                    }
                }

                // 5. Control de Luz / Láser
                if (CmdArray[0] == "CMD_LIGHT")
                {
                    bool state = (paramters[1] == 1);
                    enableLaser = state;
                    turnLaserOn(enableLaser);
                }

                // 6. Control del Modo Esquivar Obstáculos / Tracking
                if (CmdArray[0] == "CMD_TRACK")
                {
                    bool state = (paramters[1] == 1);
                    enableObstacleAvoidance = state;

                    if (enableObstacleAvoidance)
                    {
                        if (obstacleAvoidanceModeTaskHandle != NULL)
                        {
                            xTaskNotifyGive(obstacleAvoidanceModeTaskHandle);
                        }
                    }
                    else
                    {
                        processDifferentialDrive(0.0f, 0.0f);
                    }
                }

                // Limpieza de búferes
                for (int i = 0; i < 8; i++)
                {
                    CmdArray[i] = "";
                    paramters[i] = 0;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        client.stop();

        // Parada de seguridad cuando la app se desconecta
        processDifferentialDrive(0.0f, 0.0f);
    }
}

void cameraStreamTaskTCP(void *pvParameters)
{
    for (;;)
    {
        WiFiClient client = server_Camera.accept();
        if (client)
        {
#ifdef DEBUG
            Serial.println("📷 Cliente de cámara conectado vía TCP (Puerto 7000)");
#endif
            while (client.connected())
            {
                if (videoFlag)
                {
                    camera_fb_t *fb = esp_camera_fb_get();
                    if (fb)
                    {
                        uint32_t jpg_buf_len = fb->len;
                        uint8_t *jpg_buf = fb->buf;

                        // Header de 4 bytes con el tamaño del frame (Little-Endian)
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
                vTaskDelay(pdMS_TO_TICKS(30)); // ~25 FPS
            }
            client.stop();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void initWiFi()
{
    WiFi.persistent(false);
    WiFi.setSleep(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    // Configuración de IP Estática en Modo STA
    IPAddress staticIP(192, 168, 0, 202);
    IPAddress gateway(192, 168, 0, 1);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress dns(8, 8, 8, 8);

    WiFi.mode(WIFI_STA);

    if (!WiFi.config(staticIP, gateway, subnet, dns))
    {
#ifdef DEBUG
        Serial.println("❌ Fallo al configurar IP Estática en STA");
#endif
    }

    WiFi.begin("Tractorex", "9983476198");

#ifdef DEBUG
    Serial.print("Conectando a la red Tractorex");
#endif

    while (WiFi.status() != WL_CONNECTED)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
#ifdef DEBUG
        Serial.print(".");
#endif
    }

#ifdef DEBUG
    Serial.println("\n✅ Wi-Fi Conectado!");
    Serial.print("IP del coche: ");
    Serial.println(WiFi.localIP());
#endif

    server_Cmd.begin(4000);
    server_Camera.begin(7000);

    // Tarea de streaming de video en el Core 0
    xTaskCreatePinnedToCore(
        cameraStreamTaskTCP,
        "CamTCPStream",
        1024 * 4,
        NULL,
        3,
        NULL,
        0);

    ArduinoOTA.begin();

    // Desactiva Bluetooth para liberar RAM
    btStop();
    esp_bt_controller_disable();
}