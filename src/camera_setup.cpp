#include "camera_setup.h"
#include "config.h"
#include <esp_camera.h>

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
    config.xclk_freq_hz = 16500000; // 🚀 División exacta LEDC: elimina saltos de fotogramas del sensor
    config.pixel_format = PIXFORMAT_JPEG;

    if (psramFound())
    {
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.frame_size = FRAMESIZE_QVGA;   // 320x240
        config.jpeg_quality = 22;             // 🚀 Reduce el peso promedio de ~10KB a ~6KB por frame
        config.fb_count = 2;                  // Búfer doble
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
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
        // Orientación de la imagen
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);

        // Control de color y contraste base
        s->set_brightness(s, 0); // Normal (-2 a 2)
        s->set_contrast(s, 0);   // Normal (-2 a 2)
        s->set_saturation(s, 1); // Ligera saturación para colores más vivos (+1)

        // Control de Exposición y Ganancia
        s->set_whitebal(s, 1); // Balance de blancos automático (AWB) activado
        s->set_awb_gain(s, 1); // Ganancia AWB activada
        s->set_wb_mode(s, 0);  // Modo AWB: Auto (0: Auto, 1: Sunny, 2: Cloudy, 3: Office, 4: Home)

        s->set_exposure_ctrl(s, 1); // AEC activado
        s->set_aec2(s, 1);          // AEC DSP inteligente activado (mejora transición rápida)
        s->set_ae_level(s, 0);      // Nivel de exposición centrado (-2 a 2)

        s->set_gain_ctrl(s, 1);                // AGC activado
        s->set_gainceiling(s, GAINCEILING_4X); // 🚀 Límite de ganancia a 4x para evitar ruido excesivo

        // Corrección de lente y visión limpia
        s->set_bpc(s, 1);  // Corrección de píxeles defectuosos activada
        s->set_wpc(s, 1);  // Corrección de píxeles blancos activada
        s->set_lenc(s, 1); // Corrección de sombreado de lente (Lens Correction)
    }
}