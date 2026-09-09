#include "display_control.h"
#include "config.h"
#include "i2c_manager.h"
#include "custom_motor_driver.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Wire.h>

#define TFT_SCLK 14
#define TFT_MOSI 13
#define TFT_DC 15
#define TFT_CS -1
#define TFT_RST -1

static Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
static QueueHandle_t displayQueue = NULL;

// Función de renderizado de pantallas
static void renderScreen(const DisplayMessage &msg)
{
    switch (msg.state)
    {
    case DISPLAY_BOOT:
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_BLUE);
        tft.setTextSize(1);
        tft.setCursor(10, 10);
        tft.println("TRACTOREX BOT");
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(10, 30);
        tft.println("Iniciando sistema...");
        break;

    case DISPLAY_CONNECTING_WIFI:
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_YELLOW);
        tft.setCursor(10, 30);
        tft.println("Conectando Wi-Fi...");
        if (strlen(msg.textExtra) > 0)
        {
            tft.setTextColor(ST77XX_WHITE);
            tft.setCursor(10, 45);
            tft.println(msg.textExtra);
        }
        break;

    case DISPLAY_PORTAL_ACTIVE:
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_ORANGE);
        tft.setCursor(10, 10);
        tft.println("MODO CONFIG (AP)");
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(10, 30);
        tft.println("SSID: ESP-CAMERA-CAR");
        tft.setCursor(10, 45);
        tft.println("IP: 192.168.4.1");
        break;

    case DISPLAY_CONNECTED:
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_GREEN);
        tft.setCursor(10, 30);
        tft.println("Wi-Fi CONECTADO");
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(10, 45);
        tft.printf("IP: %s", msg.textExtra);
        break;

    case DISPLAY_OBSTACLE_ALERT:
        tft.fillRect(0, 60, 160, 20, ST77XX_RED);
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(10, 66);
        tft.println("! OBSTACULO DETECTADO !");
        break;

    case DISPLAY_CLEAR_ALERT:
        tft.fillRect(0, 60, 160, 20, ST77XX_BLACK);
        break;
    }
}

static void hardResetDisplayViaPCF()
{
    // 1. Asegurar HIGH inicial
    setPcfDisplayResetPin(true);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. Pulso LOW sostenido para garantizar descarga completa de la línea RST
    setPcfDisplayResetPin(false);
    vTaskDelay(pdMS_TO_TICKS(150));

    // 3. Subir a HIGH y dar tiempo al oscilador RC del ST7735 para arrancar
    setPcfDisplayResetPin(true);
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void displayTask(void *pvParameters)
{
    // 1. Forzar un reset físico seguro por I2C
    hardResetDisplayViaPCF();

    // 2. Reiniciar el bus SPI del ESP32 por software
    SPI.end();
    vTaskDelay(pdMS_TO_TICKS(50));
    SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

    // 3. Inicializar la librería Adafruit
    tft.initR(INITR_MINI160x80_PLUGIN);

    // 4. Bajar temporalmente la velocidad SPI a 4 MHz para asegurar recepción limpia de comandos de inicialización
    tft.setSPISpeed(4000000);

    // 5. Enviar secuencia limpia de orientación e inversión
    tft.setRotation(3);
    tft.invertDisplay(false);
    tft.fillScreen(ST77XX_BLACK);

    // 6. Subir velocidad SPI a 16 MHz para un renderizado rápido en operación normal
    tft.setSPISpeed(16000000);

    // 7. Mostrar primer frame
    DisplayMessage initMsg = {DISPLAY_BOOT, ""};
    renderScreen(initMsg);

    DisplayMessage rxMsg;
    for (;;)
    {
        if (xQueueReceive(displayQueue, &rxMsg, portMAX_DELAY) == pdTRUE)
        {
            renderScreen(rxMsg);
        }
    }
}

void initDisplayTask()
{
    displayQueue = xQueueCreate(5, sizeof(DisplayMessage));

    if (displayQueue != NULL)
    {
        xTaskCreatePinnedToCore(
            displayTask,
            "DisplayTask",
            2048,
            NULL,
            1,
            NULL,
            0);
    }
}

void updateDisplayState(DisplayState state, const char *extraText)
{
    if (displayQueue != NULL)
    {
        DisplayMessage msg;
        msg.state = state;
        if (extraText != NULL)
        {
            strncpy(msg.textExtra, extraText, sizeof(msg.textExtra) - 1);
            msg.textExtra[sizeof(msg.textExtra) - 1] = '\0';
        }
        else
        {
            msg.textExtra[0] = '\0';
        }

        xQueueSend(displayQueue, &msg, 0);
    }
}