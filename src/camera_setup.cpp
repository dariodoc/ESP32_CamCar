#include <Arduino.h>

// Remapeamos los nombres de http_parser.h antes de que se incluyan para evitar la colisión con ESPAsyncWebServer
#define HTTP_GET ESP_HTTP_GET
#define HTTP_POST ESP_HTTP_POST
#define HTTP_DELETE ESP_HTTP_DELETE
#define HTTP_PUT ESP_HTTP_PUT
#define HTTP_PATCH ESP_HTTP_PATCH
#define HTTP_HEAD ESP_HTTP_HEAD
#define HTTP_OPTIONS ESP_HTTP_OPTIONS

#include <esp_camera.h>
#include <esp_http_server.h>

// Restauramos el estado de los macros
#undef HTTP_GET
#undef HTTP_POST
#undef HTTP_DELETE
#undef HTTP_PUT
#undef HTTP_PATCH
#undef HTTP_HEAD
#undef HTTP_OPTIONS

#include "config.h"
#include "camera_setup.h"

const static int psramLimit = 4096;
httpd_handle_t stream_httpd = NULL;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true)
    {
        fb = esp_camera_fb_get();
        if (!fb)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;

        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK)
        {
            size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }

        esp_camera_fb_return(fb);
        fb = NULL;

        if (res != ESP_OK)
        {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // ~30 FPS
    }

    return res;
}

void startCameraServer()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81;
    config.ctrl_port = 81;

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = (httpd_method_t)ESP_HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL};

    if (httpd_start(&stream_httpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}

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
        config.jpeg_quality = 14;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
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
    {
#ifdef DEBUG
        Serial.printf("Camera init failed with error 0x%x\n", err);
#endif
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL)
    {
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_wb_mode(s, 0);
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 0);
        s->set_bpc(s, 1);
        s->set_wpc(s, 1);
    }
}