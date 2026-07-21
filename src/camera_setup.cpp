#include "config.h"
#include "camera_setup.h"
#include <esp_camera.h>

const static int psramLimit = 10000;

void setupCamera()
{

#ifdef DEBUG
    Serial.printf("setupCamera() running on core: %d\n", xPortGetCoreID());
#endif

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_4;
    config.ledc_timer = LEDC_TIMER_2;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    if (psramFound())
    {
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.frame_size = FRAMESIZE_HVGA;
        config.jpeg_quality = 24;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
        heap_caps_malloc_extmem_enable(psramLimit);
    }
    else
    {
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 24; // <--- ✅ SUBIR A 24 PARA EVITAR COLAPSO
        config.fb_count = 1;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
#ifdef DEBUG
        Serial.printf("Camera init failed with error 0x%x", err);
#endif
        return;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL)
    {
        s->set_whitebal(s, 1); // Auto balance de blancos
        s->set_awb_gain(s, 1);
        s->set_wb_mode(s, 0);
        s->set_exposure_ctrl(s, 1); // Auto exposición
        s->set_aec2(s, 0);          // Deshabilitar algoritmo DSP extra para ganar CPU
        s->set_bpc(s, 1);           // Corrección de pixeles negros
        s->set_wpc(s, 1);           // Corrección de pixeles blancos
    }
}

void sendCameraPicture(void *parameters)
{
    camera_fb_t *fb = nullptr;
    int consecutiveFailures = 0;
    int blockedCounter = 0;

    for (;;)
    {
        // 1. Si no hay cliente activo, esperamos
        if (cameraClientId == 0)
        {
            blockedCounter = 0;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        AsyncWebSocketClient *client = wsCamera.client(cameraClientId);

        // 2. Si el cliente existe pero su cola de envío está llena
        if (!client || !client->canSend())
        {
            blockedCounter++;
            if (blockedCounter > 40) // Si se atora más de 800ms
            {
                wsCamera.cleanupClients(); // Forzamos limpieza de buffers pendientes
                blockedCounter = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        blockedCounter = 0; // Si puede enviar, reseteamos el contador

        // 3. Protección de RAM crítica (umbral ajustado a 20KB)
        if (ESP.getFreeHeap() < 20000)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 4. Captura de Frame
        fb = esp_camera_fb_get();
        if (!fb)
        {
            consecutiveFailures++;
            if (consecutiveFailures >= 10)
            {
                sensor_t *s = esp_camera_sensor_get();
                if (s)
                    s->reset(s);
                consecutiveFailures = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        consecutiveFailures = 0;

        // 5. Envío de Frame y liberación de memoria
        client->binary(fb->buf, fb->len);
        esp_camera_fb_return(fb);

        // Ritmo de transmisión estable (~8 FPS)
        vTaskDelay(pdMS_TO_TICKS(125));
    }
}