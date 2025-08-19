#include "config.h"
#include "camera_setup.h"
#include <esp_camera.h>

const static int psramLimit = 10000;

void setupCamera() {
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

    if (psramFound()) {
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.frame_size = FRAMESIZE_HVGA;
        config.jpeg_quality = 15;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
        heap_caps_malloc_extmem_enable(psramLimit);
    } else {
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        #ifdef DEBUG
        Serial.printf("Camera init failed with error 0x%x", err);
        #endif
        return;
    }
}

void sendCameraPicture(void *parameters)
{
    camera_fb_t *fb = nullptr;

    for (;;) // Bucle infinito
    {
        // Si no hay ningún cliente conectado, la tarea se duerme un rato y vuelve a comprobar.
        if (cameraClientId == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue; // Vuelve al inicio del bucle
        }

        // Si hay un cliente, captura y envía un fotograma
        fb = esp_camera_fb_get();
        if (!fb)
        {
#ifdef DEBUG
            Serial.println(F("Fallo al capturar fotograma"));
#endif
            continue;
        }

        // Busca al cliente por su ID y le envía la imagen
        AsyncWebSocketClient *client = wsCamera.client(cameraClientId);
        if (client && client->canSend())
        {
            client->binary(fb->buf, fb->len);
        }

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(1)); // Pequeña pausa para no saturar la red
    }
}