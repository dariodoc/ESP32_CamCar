#include "camera_setup.h"
#include "config.h"
#include <esp_camera.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"

extern AsyncWebServer server; // Instancia global compartida en puerto 80

AsyncWebSocket wsCamera("/CameraStream");
const static int psramLimit = 4096;

void streamCameraFrame()
{
    // Solo procesa y envía si hay clientes conectados viendo el video
    if (wsCamera.count() == 0)
        return;

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
        return;

    // Transmite los bytes binarios del JPEG directamente si el socket tiene espacio
    if (wsCamera.availableForWriteAll())
    {
        wsCamera.binaryAll(fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);
}

// Tarea en Core 0 para capturar y enviar cuadros por WebSocket
void cameraStreamTask(void *pvParameters)
{
    for (;;)
    {
        streamCameraFrame();
        // Cede brevemente el control a la pila Wi-Fi en Core 0
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void initCameraWebSocket()
{
    // Agrega el handler del WebSocket al servidor asíncrono
    server.addHandler(&wsCamera);

    // Lanza la tarea de transmisión anclada explícitamente al CORE 0
    xTaskCreatePinnedToCore(
        cameraStreamTask,
        "CamWSStream",
        4096,
        NULL,
        2,
        NULL,
        0 // 👈 CORE 0
    );
}

void setupCamera()
{
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
        config.frame_size = FRAMESIZE_HVGA; // 
        config.jpeg_quality = 30;
        config.fb_count = 3;                   // 3 búferes en PSRAM
        config.grab_mode = CAMERA_GRAB_LATEST; // Retener solo el cuadro más reciente
        heap_caps_malloc_extmem_enable(psramLimit);
    }
    else
    {
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 24;
        config.fb_count = 1;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
        return;

    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL)
    {
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_wb_mode(s, 0);
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 0);     
        s->set_ae_level(s, 1); 
        s->set_bpc(s, 1);      
        s->set_wpc(s, 1);      
        s->set_raw_gma(s, 1);  
        s->set_lenc(s, 0);
    }
}