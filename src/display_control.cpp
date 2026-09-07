#include "display_control.h"
#include "config.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// Asignación de pines: 100% limpia sin compartición de líneas
#define TFT_SCLK 14  // SCL
#define TFT_MOSI 13  // SDA
#define TFT_DC   15  // DC
#define TFT_CS   -1  // Amarrado físicamente a GND
#define TFT_RST  -1  // Amarrado físicamente a 3.3V

#ifndef ST77XX_YELLOW
  #define ST77XX_YELLOW 0xFFE0
#endif

#ifndef ST77XX_ORANGE
  #define ST77XX_ORANGE 0xFD20
#endif

static Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
static QueueHandle_t displayQueue = NULL;

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

static void displayTask(void *pvParameters)
{
    // 1. Pausa amplia de 1 segundo para asegurar que la rampa de voltaje de la fuente del robot llegue a 3.3V estables
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 2. Pre-inicializar bus SPI
    tft.initR(INITR_MINI160x80_PLUGIN);

    // 3. 🚀SECUENCIA MANUAL DE DESPERTAR SEGÚN DATASHEET ST7735S
    tft.sendCommand(ST77XX_SWRESET);  // 0x01: Reset de Software
    vTaskDelay(pdMS_TO_TICKS(150));   // Espera obligatoria (mínimo 120ms)

    tft.sendCommand(ST77XX_SLPOUT);   // 0x11: Salir de Sleep Mode
    vTaskDelay(pdMS_TO_TICKS(150));   // Espera obligatoria (mínimo 120ms)

    tft.sendCommand(ST77XX_DISPON);   // 0x29: Encender pantalla
    vTaskDelay(pdMS_TO_TICKS(50));

    // 4. Inicialización definitiva de la librería con registros limpios
    tft.initR(INITR_MINI160x80_PLUGIN);
    tft.setRotation(3);
    tft.invertDisplay(false);

    // 5. Mostrar primer frame
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
            0
        );
    }
}

void updateDisplayState(DisplayState state, const char *extraText)
{
    if (displayQueue != NULL)
    {
        DisplayMessage msg;
        msg.state = state;
        strncpy(msg.textExtra, extraText, sizeof(msg.textExtra) - 1);
        msg.textExtra[sizeof(msg.textExtra) - 1] = '\0';

        xQueueSend(displayQueue, &msg, 0);
    }
}