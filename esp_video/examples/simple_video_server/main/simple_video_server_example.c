/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/select.h>
#include <sys/errno.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "protocol_examples_common.h"
#include "mdns.h"
#include "lwip/inet.h"
#include "lwip/apps/netbiosns.h"
#include "example_video_common.h"
#include "shell.h"

#define EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER  CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER

#define EXAMPLE_JPEG_ENC_QUALITY            CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY

#define EXAMPLE_MDNS_INSTANCE               CONFIG_EXAMPLE_MDNS_INSTANCE
#define EXAMPLE_MDNS_HOST_NAME              CONFIG_EXAMPLE_MDNS_HOST_NAME

#define EXAMPLE_PART_BOUNDARY               CONFIG_EXAMPLE_HTTP_PART_BOUNDARY

// Optional JPEG m2m stage (V4L2): number of CAP buffers to map
#define M2M_JPEG_CAP_COUNT  CONFIG_EXAMPLE_M2M_CAP_COUNT

static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" EXAMPLE_PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" EXAMPLE_PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

#if CONFIG_ESP_VIDEO_ENABLE_HW_JPEG_VIDEO_DEVICE
// Forward declaration for V4L2 JPEG control helper
static esp_err_t set_codec_control(int fd, uint32_t ctrl_class, uint32_t id, int32_t value);
#endif

extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[] asm("_binary_index_html_gz_end");
extern const uint8_t loading_jpg_gz_start[] asm("_binary_loading_jpg_gz_start");
extern const uint8_t loading_jpg_gz_end[] asm("_binary_loading_jpg_gz_end");
extern const uint8_t favicon_ico_gz_start[] asm("_binary_favicon_ico_gz_start");
extern const uint8_t favicon_ico_gz_end[] asm("_binary_favicon_ico_gz_end");
extern const uint8_t assets_index_js_gz_start[] asm("_binary_index_js_gz_start");
extern const uint8_t assets_index_js_gz_end[] asm("_binary_index_js_gz_end");
extern const uint8_t assets_index_css_gz_start[] asm("_binary_index_css_gz_start");
extern const uint8_t assets_index_css_gz_end[] asm("_binary_index_css_gz_end");

/**
 * @brief Web cam control structure
 */
typedef struct frame_item {
    uint8_t slot;        // index into out_bufs
    uint32_t size;       // encoded size
} frame_item_t;

typedef struct web_cam_video {
    int fd;
    uint8_t index;
    const char *dev_name;

    example_encoder_handle_t encoder_handle;

    uint8_t *buffer[EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER];
    uint32_t buffer_size;
    uint32_t buffer_count;

    // Producer/consumer encoded buffers
    uint8_t **out_bufs;         // array of pointers (size out_buf_count)
    uint32_t out_buf_size;      // per-buffer capacity
    uint8_t  out_buf_count;     // number of output buffers (queue depth)
    QueueHandle_t frame_q;      // produced frames (frame_item_t)
    QueueHandle_t free_q;       // free slots (uint8_t)
    TaskHandle_t producer_task; // producer task handle
    SemaphoreHandle_t stream_lock; // allow only one streaming consumer per video

    // Diagnostics
    uint32_t stats_published;
    uint32_t stats_drops;
    uint32_t stats_dq_errors;
    uint32_t stats_recoveries;
    uint32_t stats_restart_fail;
    uint32_t stats_select_timeouts;
    int64_t  last_pub_us;
    uint32_t expected_fps;
    uint32_t meas_frames_window;
    int64_t  meas_window_start_us;

    // Sensor timing diagnostics
    uint32_t dq_fps_samples;
    int64_t  dq_last_us;

    // Optional JPEG m2m device (hardware encoder via V4L2)
    int       m2m_fd;
    uint8_t  *m2m_cap_buf[M2M_JPEG_CAP_COUNT];
    size_t    m2m_cap_len[M2M_JPEG_CAP_COUNT];
    uint8_t   m2m_cap_count;
    uint8_t   use_m2m_jpeg; // 1 if configured and initialized

    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint8_t jpeg_quality;

    uint32_t frame_rate;

    SemaphoreHandle_t sem;

    uint32_t support_control_jpeg_quality   : 1;
} web_cam_video_t;

typedef struct web_cam {
    uint8_t video_count;
    web_cam_video_t video[0];
} web_cam_t;

typedef struct web_cam_video_config {
    const char *dev_name;
    uint32_t buffer_count;
} web_cam_video_config_t;

typedef struct request_desc {
    int index;
} request_desc_t;

static const char *TAG = "example";

// Forward declaration for recovery routine
static esp_err_t restart_video_stream(web_cam_video_t *video);

// Forward declarations for producer model
static void producer_task(void *arg);

static bool is_valid_web_cam(web_cam_video_t *video)
{
    return video->fd != -1;
}

static esp_err_t decode_request(web_cam_t *web_cam, httpd_req_t *req, request_desc_t *desc)
{
    esp_err_t ret;
    int index = -1;
    char buffer[32];

    if ((ret = httpd_req_get_url_query_str(req, buffer, sizeof(buffer))) != ESP_OK) {
        return ret;
    }
    ESP_LOGD(TAG, "source: %s", buffer);

    for (int i = 0; i < web_cam->video_count; i++) {
        char source_str[16];

        if (snprintf(source_str, sizeof(source_str), "source=%d", i) <= 0) {
            return ESP_FAIL;
        }

        if (strcmp(buffer, source_str) == 0) {
            index = i;
            break;
        }
    }
    if (index == -1) {
        return ESP_ERR_INVALID_ARG;
    }

    desc->index = index;
    return ESP_OK;
}

static esp_err_t capture_video_image(httpd_req_t *req, web_cam_video_t *video, bool is_jpeg)
{
    (void)is_jpeg; // we always return JPEG from the producer
    frame_item_t item = {0};
    // wait up to 1s for a produced frame
    if (xQueueReceive(video->frame_q, &item, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = httpd_resp_send(req, (char *)video->out_bufs[item.slot], item.size);
    // return slot to free list
    uint8_t slot = item.slot;
    xQueueSend(video->free_q, &slot, portMAX_DELAY);
    return err;
}

static char *get_cameras_json(web_cam_t *web_cam)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *cameras = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "cameras", cameras);

    for (int i = 0; i < web_cam->video_count; i++) {
        char src_str[32];

        if (!is_valid_web_cam(&web_cam->video[i])) {
            continue;
        }

        cJSON *camera = cJSON_CreateObject();
        cJSON_AddNumberToObject(camera, "index", i);
        assert(snprintf(src_str, sizeof(src_str), ":%d/stream", i + 81) > 0);
        cJSON_AddStringToObject(camera, "src", src_str);
        cJSON_AddNumberToObject(camera, "currentFrameRate", web_cam->video[i].frame_rate);
        cJSON_AddNumberToObject(camera, "currentImageFormat", 0);
        assert(snprintf(src_str, sizeof(src_str), "JPEG %" PRIu32 "x%" PRIu32, web_cam->video[i].width, web_cam->video[i].height) > 0);
        cJSON_AddStringToObject(camera, "currentImageFormatDescription", src_str);

        if (web_cam->video[i].support_control_jpeg_quality) {
            cJSON_AddNumberToObject(camera, "currentQuality", web_cam->video[i].jpeg_quality);
        }

        cJSON *current_resolution = cJSON_CreateObject();
        cJSON_AddNumberToObject(current_resolution, "width", web_cam->video[i].width);
        cJSON_AddNumberToObject(current_resolution, "height", web_cam->video[i].height);
        cJSON_AddItemToObject(camera, "currentResolution", current_resolution);

        cJSON *image_formats = cJSON_CreateArray();
        cJSON *image_format = cJSON_CreateObject();
        cJSON_AddNumberToObject(image_format, "id", 0);
        assert(snprintf(src_str, sizeof(src_str), "JPEG %" PRIu32 "x%" PRIu32, web_cam->video[i].width, web_cam->video[i].height) > 0);
        cJSON_AddStringToObject(image_format, "description", src_str);

        if (web_cam->video[i].support_control_jpeg_quality) {
            cJSON *image_format_quality = cJSON_CreateObject();

            int min_quality = 1;
            int max_quality = 100;
            int step_quality = 1;
            int default_quality = EXAMPLE_JPEG_ENC_QUALITY;
            if (web_cam->video[i].pixel_format == V4L2_PIX_FMT_JPEG) {
                struct v4l2_query_ext_ctrl qctrl = {0};

                qctrl.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
                if (ioctl(web_cam->video[i].fd, VIDIOC_QUERY_EXT_CTRL, &qctrl) == 0) {
                    min_quality = qctrl.minimum;
                    max_quality = qctrl.maximum;
                    step_quality = qctrl.step;
                    default_quality = qctrl.default_value;
                }
            }

            cJSON_AddNumberToObject(image_format_quality, "min", min_quality);
            cJSON_AddNumberToObject(image_format_quality, "max", max_quality);
            cJSON_AddNumberToObject(image_format_quality, "step", step_quality);
            cJSON_AddNumberToObject(image_format_quality, "default", default_quality);
            cJSON_AddItemToObject(image_format, "quality", image_format_quality);
        }
        cJSON_AddItemToArray(image_formats, image_format);

        cJSON_AddItemToObject(camera, "imageFormats", image_formats);
        cJSON_AddItemToArray(cameras, camera);
    }

    char *output = cJSON_Print(root);
    cJSON_Delete(root);
    return output;
}

static esp_err_t set_camera_jpeg_quality(web_cam_video_t *video, int quality)
{
    esp_err_t ret = ESP_OK;
    int quality_reset = quality;

    if (video->use_m2m_jpeg) {
        // Quality control via m2m JPEG device
        // Clamp using generic 1..100 bounds if needed
        if (quality < 1) quality_reset = 1;
        if (quality > 100) quality_reset = 100;
#if CONFIG_ESP_VIDEO_ENABLE_HW_JPEG_VIDEO_DEVICE
        if (video->m2m_fd > 0) {
            // Set V4L2 JPEG quality on the encoder
            set_codec_control(video->m2m_fd, V4L2_CID_JPEG_CLASS, V4L2_CID_JPEG_COMPRESSION_QUALITY, quality_reset);
            video->jpeg_quality = quality_reset;
            video->support_control_jpeg_quality = 1;
        } else {
            video->support_control_jpeg_quality = 0;
            ESP_LOGW(TAG, "video%d: invalid m2m fd for setting quality", video->index);
        }
#else
        video->support_control_jpeg_quality = 0;
        ESP_LOGW(TAG, "video%d: m2m JPEG not enabled in Kconfig", video->index);
#endif
    } else if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
        struct v4l2_ext_controls controls = {0};
        struct v4l2_ext_control control[1];
        struct v4l2_query_ext_ctrl qctrl = {0};

        qctrl.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
        if (ioctl(video->fd, VIDIOC_QUERY_EXT_CTRL, &qctrl) == 0) {
            if ((quality > qctrl.maximum) || (quality < qctrl.minimum) ||
                    (((quality - qctrl.minimum) % qctrl.step) != 0)) {

                if (quality > qctrl.maximum) {
                    quality_reset = qctrl.maximum;
                } else if (quality < qctrl.minimum) {
                    quality_reset = qctrl.minimum;
                } else {
                    quality_reset = qctrl.minimum + ((quality - qctrl.minimum) / qctrl.step) * qctrl.step;
                }

                ESP_LOGW(TAG, "video%d: JPEG compression quality=%d is out of sensor's range, reset to %d", video->index, quality, quality_reset);
            }

            controls.ctrl_class = V4L2_CID_JPEG_CLASS;
            controls.count = 1;
            controls.controls = control;
            control[0].id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
            control[0].value = quality_reset;
            ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_S_EXT_CTRLS, &controls), TAG, "failed to set jpeg compression quality");

            video->jpeg_quality = quality_reset;
            video->support_control_jpeg_quality = 1;
        } else {
            video->support_control_jpeg_quality = 0;
            ESP_LOGW(TAG, "video%d: JPEG compression quality control is not supported", video->index);
        }
    } else {
        ESP_RETURN_ON_ERROR(example_encoder_set_jpeg_quality(video->encoder_handle, quality_reset), TAG, "failed to set jpeg quality");
        video->jpeg_quality = quality_reset;
    }

    if (video->support_control_jpeg_quality) {
        ESP_LOGI(TAG, "video%d: set jpeg quality %d success", video->index, quality_reset);
    }

    return ret;
}

static esp_err_t camera_info_handler(httpd_req_t *req)
{
    esp_err_t ret;
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;
    char *output = get_cameras_json(web_cam);

    httpd_resp_set_type(req, "application/json");
    ret = httpd_resp_sendstr(req, output);
    free(output);

    return ret;
}

static esp_err_t camera_settings_handler(httpd_req_t *req)
{
    esp_err_t ret;
    char *content;
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    content = (char *)calloc(1, req->content_len + 1);
    ESP_RETURN_ON_FALSE(content, ESP_ERR_NO_MEM, TAG, "failed to allocate memory");

    ESP_GOTO_ON_FALSE(httpd_req_recv(req, content, req->content_len) > 0, ESP_FAIL, fail0, TAG, "failed to recv content");
    ESP_LOGD(TAG, "content: %s", content);

    cJSON *json_root = cJSON_Parse(content);
    free(content);
    content = NULL;
    ESP_GOTO_ON_FALSE(json_root, ESP_FAIL, fail0, TAG, "failed to parse JSON");

    cJSON *json_index = cJSON_GetObjectItem(json_root, "index");
    ESP_GOTO_ON_FALSE(json_index && cJSON_IsNumber(json_index), ESP_ERR_INVALID_ARG, fail1, TAG, "missing or invalid index field");
    int index = json_index->valueint;
    ESP_GOTO_ON_FALSE(index >= 0 && index < web_cam->video_count && is_valid_web_cam(&web_cam->video[index]), ESP_ERR_INVALID_ARG, fail1, TAG, "invalid index");

    cJSON *json_image_format = cJSON_GetObjectItem(json_root, "image_format");
    ESP_GOTO_ON_FALSE(json_image_format && cJSON_IsNumber(json_image_format), ESP_ERR_INVALID_ARG, fail1, TAG, "missing or invalid image_format field");
    int image_format = json_image_format->valueint;

    cJSON *json_jpeg_quality = cJSON_GetObjectItem(json_root, "jpeg_quality");
    ESP_GOTO_ON_FALSE(json_jpeg_quality && cJSON_IsNumber(json_jpeg_quality), ESP_ERR_INVALID_ARG, fail1, TAG, "missing or invalid jpeg_quality field");
    int jpeg_quality = json_jpeg_quality->valueint;

    ESP_LOGI(TAG, "JSON parse success - index:%d, image_format:%d, jpeg_quality:%d", index, image_format, jpeg_quality);
    cJSON_Delete(json_root);
    json_root = NULL;

    ESP_GOTO_ON_ERROR(set_camera_jpeg_quality(&web_cam->video[index], jpeg_quality), fail1, TAG, "failed to set camera jpeg quality");

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;

fail1:
    if (json_root) {
        cJSON_Delete(json_root);
    }
fail0:
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        httpd_resp_send_408(req);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON format");
    }
    if (content) {
        free(content);
    }
    return ret;
}

static esp_err_t static_file_handler(httpd_req_t *req)
{
    const char *uri = req->uri;

    /* Route to appropriate static file based on URI */
    if (strcmp(uri, "/") == 0) {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)index_html_gz_start, index_html_gz_end - index_html_gz_start);
    } else if (strcmp(uri, "/loading.jpg") == 0) {
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)loading_jpg_gz_start, loading_jpg_gz_end - loading_jpg_gz_start);
    } else if (strcmp(uri, "/favicon.ico") == 0) {
        httpd_resp_set_type(req, "image/x-icon");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)favicon_ico_gz_start, favicon_ico_gz_end - favicon_ico_gz_start);
    } else if (strcmp(uri, "/assets/index.js") == 0) {
        httpd_resp_set_type(req, "application/javascript");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)assets_index_js_gz_start, assets_index_js_gz_end - assets_index_js_gz_start);
    } else if (strcmp(uri, "/assets/index.css") == 0) {
        httpd_resp_set_type(req, "text/css");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)assets_index_css_gz_start, assets_index_css_gz_end - assets_index_css_gz_start);
    }

    /* If no static file matches, return 404 */
    ESP_LOGW(TAG, "File not found: %s", uri);
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

static esp_err_t image_stream_handler(httpd_req_t *req)
{
    esp_err_t ret;
    char http_string[128];
    web_cam_video_t *video = (web_cam_video_t *)req->user_ctx;

    // allow only one streaming client per video
    if (xSemaphoreTake(video->stream_lock, 0) != pdTRUE) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Stream already in use");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Stream started - video%d fd=%d", video->index, video->fd);

    ESP_RETURN_ON_FALSE(snprintf(http_string, sizeof(http_string), "%" PRIu32, video->frame_rate) > 0,
                        ESP_FAIL, TAG, "failed to format framerate buffer");

    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, STREAM_CONTENT_TYPE), TAG, "failed to set content type");
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"), TAG, "failed to set access control allow origin");
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "X-Framerate", http_string), TAG, "failed to set x framerate");

    frame_item_t item = {0};
    uint32_t sent_count = 0;
    uint32_t empty_polls = 0;
    while (1) {
        int hlen;
        struct timespec ts;
        static const TickType_t frame_delay_ticks = pdMS_TO_TICKS(67);
        static TickType_t last_tick = 0;

        // Wait for next produced frame
        if (xQueueReceive(video->frame_q, &item, pdMS_TO_TICKS(500)) != pdTRUE) {
            // no frame available, continue trying
            empty_polls++;
            if (empty_polls % 6 == 0) { // ~3s
            ESP_LOGW(TAG, "[HTTP] video%d no frames (%lu empty polls)", video->index, (unsigned long)empty_polls);
            // Fallback: if the producer hasn't published for >3s, try to restart the stream
            int64_t now = esp_timer_get_time();
            if (now - video->last_pub_us > 3000000) {
                ESP_LOGW(TAG, "[HTTP] video%d no publications for %lld us, attempting restart", video->index, (long long)(now - video->last_pub_us));
                (void)restart_video_stream(video);
                empty_polls = 0; // reset observation window
                }
            }
            continue;
        }
        empty_polls = 0;

        // Throttle to ~15 fps to avoid overwhelming client
        TickType_t now_ticks = xTaskGetTickCount();
        if (last_tick != 0) {
            TickType_t elapsed = now_ticks - last_tick;
            if (elapsed < frame_delay_ticks) {
                vTaskDelay(frame_delay_ticks - elapsed);
                now_ticks = xTaskGetTickCount();
            }
        }
        last_tick = now_ticks;

        // Send multipart headers + JPEG
        ESP_GOTO_ON_ERROR(httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)), fail0, TAG, "failed to send boundary");
        ESP_GOTO_ON_ERROR(clock_gettime(CLOCK_MONOTONIC, &ts), fail0, TAG, "failed to get time");
        ESP_GOTO_ON_FALSE((hlen = snprintf(http_string, sizeof(http_string), STREAM_PART, (unsigned int)item.size, ts.tv_sec, ts.tv_nsec)) > 0,
                          ESP_FAIL, fail0, TAG, "failed to format part buffer");
        if (httpd_resp_send_chunk(req, http_string, hlen) != ESP_OK) {
            ESP_LOGW(TAG, "[HTTP] video%d header send blocked, dropping frame", video->index);
            uint8_t slot = item.slot;
            xQueueSend(video->free_q, &slot, portMAX_DELAY);
            continue;
        }
        if (httpd_resp_send_chunk(req, (char *)video->out_bufs[item.slot], item.size) != ESP_OK) {
            ESP_LOGW(TAG, "[HTTP] video%d payload send blocked, dropping frame", video->index);
            uint8_t slot = item.slot;
            xQueueSend(video->free_q, &slot, portMAX_DELAY);
            continue;
        }

        // Return slot to free list
        uint8_t slot = item.slot;
        xQueueSend(video->free_q, &slot, portMAX_DELAY);

        sent_count++;
#if CONFIG_EXAMPLE_PERF_LOGS
        if ((sent_count % 30) == 0) {
            ESP_LOGI(TAG, "[HTTP] video%d sent=%lu (free=%lu queued=%lu)", video->index, (unsigned long)sent_count,
                     (unsigned long)uxQueueMessagesWaiting(video->free_q),
                     (unsigned long)uxQueueMessagesWaiting(video->frame_q));
        }
#endif
    }

    // not reached
    xSemaphoreGive(video->stream_lock);
    return ESP_OK;

fail0:
    // Return slot if we popped one already
    if (item.size && item.slot < video->out_buf_count) {
        uint8_t slot = item.slot;
        xQueueSend(video->free_q, &slot, 0);
    }
    xSemaphoreGive(video->stream_lock);
    return ret;
}

static esp_err_t capture_image_handler(httpd_req_t *req)
{
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    request_desc_t desc;
    ESP_RETURN_ON_ERROR(decode_request(web_cam, req, &desc), TAG, "failed to decode request");

    char type_ptr[32];
    ESP_RETURN_ON_FALSE(snprintf(type_ptr, sizeof(type_ptr), "image/jpeg;name=image%d.jpg", desc.index) > 0, ESP_FAIL, TAG, "failed to format buffer");
    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, type_ptr), TAG, "failed to set content type");

    return capture_video_image(req, &web_cam->video[desc.index], true);
}

/* removed capture_binary_handler: duplicate of capture_image_handler (always JPEG) */

static esp_err_t init_web_cam_video(web_cam_video_t *video, const web_cam_video_config_t *config, int index)
{
    int fd;
    esp_err_t ret = ESP_FAIL;
    struct v4l2_streamparm sparm;
    struct v4l2_requestbuffers req;
    struct v4l2_captureparm *cparam = &sparm.parm.capture;
    struct v4l2_fract *timeperframe = &cparam->timeperframe;

    ESP_LOGI(TAG, "[DQBUF_DEBUG] Init video%d: %s", index, config->dev_name);

    fd = open(config->dev_name, O_RDWR);
    ESP_RETURN_ON_FALSE(fd >= 0, ESP_ERR_NOT_FOUND, TAG, "Open video device %s failed", config->dev_name);

    memset(&sparm, 0, sizeof(sparm));
    sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_G_PARM, &sparm), fail0, TAG, "failed to get frame rate from %s", config->dev_name);
    video->frame_rate = timeperframe->denominator / timeperframe->numerator;
    video->expected_fps = video->frame_rate;
    ESP_LOGI(TAG, "video%d: reported fps: %lu/%lu = %lu", index,
             (unsigned long)timeperframe->denominator,
             (unsigned long)timeperframe->numerator,
             (unsigned long)video->frame_rate);

    // Force a nominal 15 fps output (driver may implement via frame skipping)
    struct v4l2_streamparm sparm_set = {0};
    sparm_set.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    sparm_set.parm.capture.capability |= V4L2_CAP_TIMEPERFRAME;
    sparm_set.parm.capture.timeperframe.numerator = 1;
    sparm_set.parm.capture.timeperframe.denominator = 15;
    if (ioctl(fd, VIDIOC_S_PARM, &sparm_set) != 0) {
        ESP_LOGW(TAG, "video%d: VIDIOC_S_PARM(15fps) failed (keeping current fps)", index);
    } else {
        memset(&sparm, 0, sizeof(sparm));
        sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd, VIDIOC_G_PARM, &sparm) == 0) {
            ESP_LOGI(TAG, "video%d: applied fps: %lu/%lu = %.2f", index,
                     (unsigned long)sparm.parm.capture.timeperframe.denominator,
                     (unsigned long)sparm.parm.capture.timeperframe.numerator,
                     sparm.parm.capture.timeperframe.numerator ?
                     (double)sparm.parm.capture.timeperframe.denominator /
                     (double)sparm.parm.capture.timeperframe.numerator : 0.0);
        }
    }

    // Force camera to output YUV422 planar for m2m compatibility
    // Get current format first to preserve width/height
    struct v4l2_format fmt_cam = {0};
    fmt_cam.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    (void)ioctl(fd, VIDIOC_G_FMT, &fmt_cam);
    if (fmt_cam.fmt.pix.width == 0 || fmt_cam.fmt.pix.height == 0) {
        // fallback defaults if driver didn't return size here
        fmt_cam.fmt.pix.width = 1280;
        fmt_cam.fmt.pix.height = 720;
    }
    struct v4l2_format sformat = {0};
    sformat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    sformat.fmt.pix.width = fmt_cam.fmt.pix.width;
    sformat.fmt.pix.height = fmt_cam.fmt.pix.height;
    sformat.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV422P;
    if (ioctl(fd, VIDIOC_S_FMT, &sformat) != 0) {
        ESP_LOGW(TAG, "video%d: VIDIOC_S_FMT(YUV422P) failed; keeping current format", index);
    }

    memset(&req, 0, sizeof(req));
    req.count  = EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_LOGI(TAG, "[BUFFER_DEBUG] Video%d: Requesting %d buffers...", index, EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER);
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_REQBUFS, &req), fail0, TAG, "failed to req buffers from %s", config->dev_name);
    ESP_LOGI(TAG, "[BUFFER_DEBUG] Video%d: Driver allocated %d buffers", index, (int)req.count);

    for (int i = 0; i < EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER; i++) {
        struct v4l2_buffer buf;

        memset(&buf, 0, sizeof(buf));
        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_QUERYBUF, &buf), fail0, TAG, "failed to query vbuf from %s", config->dev_name);

        video->buffer[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        ESP_GOTO_ON_FALSE(video->buffer[i] != MAP_FAILED, ESP_ERR_NO_MEM, fail0, TAG, "failed to mmap buffer");
        video->buffer_size = buf.length;

        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_QBUF, &buf), fail0, TAG, "failed to queue frame vbuf from %s", config->dev_name);
    }

    ESP_LOGI(TAG, "[PIPELINE_DEBUG] Video%d: All %d buffers configured and queued", index, EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER);

    struct v4l2_format format;
    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_G_FMT, &format), fail0, TAG, "Failed get fmt from %s", config->dev_name);

    video->fd = fd;
    video->dev_name = config->dev_name;
    video->width = format.fmt.pix.width;
    video->height = format.fmt.pix.height;
    video->pixel_format = format.fmt.pix.pixelformat;
    video->jpeg_quality = EXAMPLE_JPEG_ENC_QUALITY;
    video->buffer_count = req.count;

    // Do not seed exposure/gain here; let IPA + JSON drive initial 3A to avoid early overshoot.

    ESP_LOGI(TAG, "[PIPELINE_DEBUG] Video%d: Detected format: %lux%lu " V4L2_FMT_STR " (buffer_size=%lu bytes)", 
             index, (unsigned long)video->width, (unsigned long)video->height, 
             V4L2_FMT_STR_ARG(video->pixel_format), (unsigned long)video->buffer_size);

    if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
        ESP_LOGI(TAG, "[PIPELINE_DEBUG] Video%d: Native JPEG format detected", index);
        ESP_GOTO_ON_ERROR(set_camera_jpeg_quality(video, EXAMPLE_JPEG_ENC_QUALITY), fail0, TAG, "failed to set jpeg quality");
        video->support_control_jpeg_quality = 1;
    } else {
        ESP_LOGI(TAG, "[PIPELINE_DEBUG] Video%d: Non-JPEG format detected, configuring JPEG encoder...", index);
        // Try V4L2 m2m JPEG device first (hardware pipeline), fallback to example_encoder
        video->use_m2m_jpeg = 0;
#if CONFIG_ESP_VIDEO_ENABLE_HW_JPEG_VIDEO_DEVICE
        do {
            int m2m = open(ESP_VIDEO_JPEG_DEVICE_NAME, O_RDWR);
            if (m2m < 0) {
                ESP_LOGW(TAG, "video%d: JPEG m2m device open failed, fallback to SW/HW encoder API", index);
                break;
            }
            // Use blocking I/O on m2m fd to avoid pacing artifacts; stall recovery relies on publication watchdog
            struct v4l2_capability cap = {0};
            if (ioctl(m2m, VIDIOC_QUERYCAP, &cap) != 0) {
                ESP_LOGW(TAG, "video%d: m2m querycap failed, falling back", index);
                close(m2m);
                break;
            }
            // Set quality
            set_codec_control(m2m, V4L2_CID_JPEG_CLASS, V4L2_CID_JPEG_COMPRESSION_QUALITY, EXAMPLE_JPEG_ENC_QUALITY);
            // Configure OUTPUT (USERPTR) with camera format
            struct v4l2_format ofmt = {0};
            ofmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            ofmt.fmt.pix.width = video->width;
            ofmt.fmt.pix.height = video->height;
            ofmt.fmt.pix.pixelformat = video->pixel_format;
            if (ioctl(m2m, VIDIOC_S_FMT, &ofmt) != 0) {
                ESP_LOGW(TAG, "video%d: m2m OUTPUT fmt failed, falling back", index);
                close(m2m);
                break;
            }
            struct v4l2_requestbuffers oreq = {0};
            oreq.count = M2M_JPEG_CAP_COUNT; // mirror CAP depth to smooth pipeline
            oreq.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            oreq.memory = V4L2_MEMORY_USERPTR;
            if (ioctl(m2m, VIDIOC_REQBUFS, &oreq) != 0) {
                ESP_LOGW(TAG, "video%d: m2m OUTPUT reqbufs failed, falling back", index);
                close(m2m);
                break;
            }
            // Configure CAPTURE (MMAP) as JPEG
            struct v4l2_format cfmt = {0};
            cfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            cfmt.fmt.pix.width = video->width;
            cfmt.fmt.pix.height = video->height;
            cfmt.fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;
            if (ioctl(m2m, VIDIOC_S_FMT, &cfmt) != 0) {
                ESP_LOGW(TAG, "video%d: m2m CAPTURE fmt failed, fallback", index);
                close(m2m);
                break;
            }
            struct v4l2_requestbuffers creq = {0};
            creq.count = M2M_JPEG_CAP_COUNT;
            creq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            creq.memory = V4L2_MEMORY_MMAP;
            if (ioctl(m2m, VIDIOC_REQBUFS, &creq) != 0) {
                ESP_LOGW(TAG, "video%d: m2m CAPTURE reqbufs failed, fallback", index);
                close(m2m);
                break;
            }
            // Map and queue CAPTURE buffers
            for (uint8_t i = 0; i < M2M_JPEG_CAP_COUNT; i++) {
                struct v4l2_buffer b = {0};
                b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                b.memory = V4L2_MEMORY_MMAP;
                b.index = i;
                if (ioctl(m2m, VIDIOC_QUERYBUF, &b) != 0) {
            ESP_LOGW(TAG, "video%d: m2m QUERYBUF failed, falling back", index);
                    close(m2m);
                    m2m = -1; break;
                }
                video->m2m_cap_buf[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, m2m, b.m.offset);
                if (video->m2m_cap_buf[i] == MAP_FAILED) {
            ESP_LOGW(TAG, "video%d: m2m mmap failed, falling back", index);
                    close(m2m);
                    m2m = -1; break;
                }
                video->m2m_cap_len[i] = b.length;
                memset(&b, 0, sizeof(b));
                b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                b.memory = V4L2_MEMORY_MMAP;
                b.index = i;
                if (ioctl(m2m, VIDIOC_QBUF, &b) != 0) {
            ESP_LOGW(TAG, "video%d: m2m QBUF CAP failed, falling back", index);
                    close(m2m);
                    m2m = -1; break;
                }
            }
            if (m2m < 0) {
                // mapping failed earlier
            } else {
                int type = V4L2_BUF_TYPE_VIDEO_CAPTURE; ioctl(m2m, VIDIOC_STREAMON, &type);
                type = V4L2_BUF_TYPE_VIDEO_OUTPUT; ioctl(m2m, VIDIOC_STREAMON, &type);
                video->m2m_fd = m2m;
                video->m2m_cap_count = M2M_JPEG_CAP_COUNT;
                video->use_m2m_jpeg = 1;
                ESP_LOGI(TAG, "[PIPELINE_DEBUG] Video%d: JPEG m2m enabled", index);
            }
        } while (0);
#endif // CONFIG_ESP_VIDEO_ENABLE_HW_JPEG_VIDEO_DEVICE

        if (!video->use_m2m_jpeg) {
            // Fallback to example_encoder (HW JPEG engine or SW, depending on Kconfig)
            example_encoder_config_t encoder_config = {0};
            encoder_config.width = video->width;
            encoder_config.height = video->height;
            encoder_config.pixel_format = video->pixel_format;
            encoder_config.quality = EXAMPLE_JPEG_ENC_QUALITY;
            ESP_GOTO_ON_ERROR(example_encoder_init(&encoder_config, &video->encoder_handle), fail0, TAG, "failed to init encoder");
            ESP_LOGI(TAG, "[PIPELINE_DEBUG] Video%d: Encoder API configured", index);
        }

        video->support_control_jpeg_quality = 1;
    }

    if (!video->use_m2m_jpeg) {
        video->sem = xSemaphoreCreateBinary();
        ESP_GOTO_ON_FALSE(video->sem, ESP_ERR_NO_MEM, fail2, TAG, "failed to create semaphore");
        xSemaphoreGive(video->sem);
    }

    // Create producer resources (queues, buffers, task, stream lock)
    const uint8_t out_count = CONFIG_EXAMPLE_ENCODED_QUEUE_DEPTH; // queue depth / number of encoded buffers
    video->out_buf_count = out_count;
    video->frame_q = xQueueCreate(out_count, sizeof(frame_item_t));
    ESP_GOTO_ON_FALSE(video->frame_q, ESP_ERR_NO_MEM, fail2, TAG, "failed to create frame_q");
    video->free_q = xQueueCreate(out_count, sizeof(uint8_t));
    ESP_GOTO_ON_FALSE(video->free_q, ESP_ERR_NO_MEM, fail3, TAG, "failed to create free_q");
    video->stream_lock = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(video->stream_lock, ESP_ERR_NO_MEM, fail4, TAG, "failed to create stream_lock");
    xSemaphoreGive(video->stream_lock);

    video->out_bufs = (uint8_t **)calloc(out_count, sizeof(uint8_t *));
    ESP_GOTO_ON_FALSE(video->out_bufs, ESP_ERR_NO_MEM, fail5, TAG, "failed to alloc out_bufs array");

    if (video->use_m2m_jpeg || video->pixel_format == V4L2_PIX_FMT_JPEG) {
        // For m2m path, size output buffers from CAP buffer lengths
        if (video->use_m2m_jpeg) {
            size_t max_cap = 0;
            for (uint8_t i = 0; i < video->m2m_cap_count; i++) {
                if (video->m2m_cap_len[i] > max_cap) max_cap = video->m2m_cap_len[i];
            }
            video->out_buf_size = (uint32_t)max_cap;
        } else {
            video->out_buf_size = video->buffer_size;
        }
        for (uint8_t i = 0; i < out_count; i++) {
            video->out_bufs[i] = (uint8_t *)malloc(video->out_buf_size);
            ESP_GOTO_ON_FALSE(video->out_bufs[i], ESP_ERR_NO_MEM, fail6, TAG, "failed to alloc jpeg out buf");
        }
    } else {
        // allocate multiple encoder output buffers
        uint32_t size0 = 0;
        for (uint8_t i = 0; i < out_count; i++) {
            uint8_t *ptr = NULL; uint32_t sz = 0;
            ESP_GOTO_ON_ERROR(example_encoder_alloc_output_buffer(video->encoder_handle, &ptr, &sz), fail6, TAG, "failed to alloc jpeg output buf");
            if (i == 0) {
                size0 = sz;
            }
            video->out_bufs[i] = ptr;
        }
        video->out_buf_size = size0;
    }

    // initialize free slots
    for (uint8_t i = 0; i < out_count; i++) {
        xQueueSend(video->free_q, &i, portMAX_DELAY);
    }

    // start producer task
    if (xTaskCreate(producer_task, "vid_producer", 8192, video, 7, &video->producer_task) != pdPASS) {
        ESP_LOGE(TAG, "failed to create producer task");
        ret = ESP_FAIL;
        goto fail6;
    }

    ESP_LOGI(TAG, "[PIPELINE_DEBUG] Video%d: Initialization completed successfully!", index);
    return ESP_OK;

fail6:
    if (video->out_bufs) {
        if (video->use_m2m_jpeg || video->pixel_format == V4L2_PIX_FMT_JPEG) {
            for (uint8_t i = 0; i < out_count; i++) {
                if (video->out_bufs[i]) free(video->out_bufs[i]);
            }
        } else {
            for (uint8_t i = 0; i < out_count; i++) {
                if (video->out_bufs[i]) example_encoder_free_output_buffer(video->encoder_handle, video->out_bufs[i]);
            }
        }
        free(video->out_bufs);
        video->out_bufs = NULL;
    }
fail5:
    if (video->stream_lock) { vSemaphoreDelete(video->stream_lock); video->stream_lock = NULL; }
fail4:
    if (video->free_q) { vQueueDelete(video->free_q); video->free_q = NULL; }
fail3:
    if (video->frame_q) { vQueueDelete(video->frame_q); video->frame_q = NULL; }
fail2:
    if (!video->use_m2m_jpeg && video->encoder_handle) {
        example_encoder_deinit(video->encoder_handle);
        video->encoder_handle = NULL;
    }
fail0:
    close(fd);
    video->fd = -1;
    return ret;
}

static esp_err_t deinit_web_cam_video(web_cam_video_t *video)
{
    if (video->producer_task) {
        vTaskDelete(video->producer_task);
        video->producer_task = NULL;
    }

    if (video->sem) {
        vSemaphoreDelete(video->sem);
        video->sem = NULL;
    }

    // free producer resources
    if (video->use_m2m_jpeg && video->m2m_fd > 0) {
        int type = V4L2_BUF_TYPE_VIDEO_OUTPUT; ioctl(video->m2m_fd, VIDIOC_STREAMOFF, &type);
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE; ioctl(video->m2m_fd, VIDIOC_STREAMOFF, &type);
        for (uint8_t i = 0; i < video->m2m_cap_count; i++) {
            if (video->m2m_cap_buf[i]) munmap(video->m2m_cap_buf[i], video->m2m_cap_len[i]);
            video->m2m_cap_buf[i] = NULL;
            video->m2m_cap_len[i] = 0;
        }
        close(video->m2m_fd);
        video->m2m_fd = -1;
        video->use_m2m_jpeg = 0;
    }
    if (video->out_bufs) {
        if (video->use_m2m_jpeg || video->pixel_format == V4L2_PIX_FMT_JPEG) {
            for (uint8_t i = 0; i < video->out_buf_count; i++) {
                if (video->out_bufs[i]) free(video->out_bufs[i]);
            }
        } else {
            for (uint8_t i = 0; i < video->out_buf_count; i++) {
                if (video->out_bufs[i]) example_encoder_free_output_buffer(video->encoder_handle, video->out_bufs[i]);
            }
        }
        free(video->out_bufs);
        video->out_bufs = NULL;
    }
    if (video->frame_q) { vQueueDelete(video->frame_q); video->frame_q = NULL; }
    if (video->free_q)  { vQueueDelete(video->free_q);  video->free_q = NULL; }
    if (video->stream_lock) { vSemaphoreDelete(video->stream_lock); video->stream_lock = NULL; }

    if (!video->use_m2m_jpeg && video->encoder_handle) {
        example_encoder_deinit(video->encoder_handle);
        video->encoder_handle = NULL;
    }

    close(video->fd);
    return ESP_OK;
}

static esp_err_t new_web_cam(const web_cam_video_config_t *config, int config_count, web_cam_t **ret_wc)
{
    int i;
    web_cam_t *wc;
    esp_err_t ret = ESP_FAIL;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    wc = calloc(1, sizeof(web_cam_t) + config_count * sizeof(web_cam_video_t));
    ESP_RETURN_ON_FALSE(wc, ESP_ERR_NO_MEM, TAG, "failed to alloc web cam");
    wc->video_count = config_count;

    for (i = 0; i < config_count; i++) {
        wc->video[i].index = i;
        wc->video[i].fd = -1;

        ret = init_web_cam_video(&wc->video[i], &config[i], i);
        if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "failed to find web_cam %d", i);
            continue;
        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to initialize web_cam %d", i);
            goto fail0;
        }

        ESP_LOGI(TAG, "video%d: width=%" PRIu32 " height=%" PRIu32 " format=" V4L2_FMT_STR, i, wc->video[i].width,
                 wc->video[i].height, V4L2_FMT_STR_ARG(wc->video[i].pixel_format));
    }

    for (i = 0; i < config_count; i++) {
        if (is_valid_web_cam(&wc->video[i])) {
            ESP_LOGI(TAG, "[PIPELINE_DEBUG] Starting streaming for video%d (fd=%d)...", i, wc->video[i].fd);
            ESP_GOTO_ON_ERROR(ioctl(wc->video[i].fd, VIDIOC_STREAMON, &type), fail1, TAG, "failed to start stream");
            ESP_LOGI(TAG, "[PIPELINE_DEBUG] Video%d: Streaming started successfully!", i);
        } else {
            ESP_LOGW(TAG, "[PIPELINE_DEBUG] Video%d: Skipping - invalid camera", i);
        }
    }

    *ret_wc = wc;

    ESP_LOGI(TAG, "[PIPELINE_DEBUG] Webcam configured successfully - %d active cameras", config_count);
    return ESP_OK;

fail1:
    ESP_LOGE(TAG, "[PIPELINE_DEBUG] STREAMON failed for video%d! Stopping previous streams...", i);
    for (int j = i - 1; j >= 0; j--) {
        if (is_valid_web_cam(&wc->video[j])) {
            ESP_LOGI(TAG, "[PIPELINE_DEBUG] Stopping stream video%d...", j);
            ioctl(wc->video[j].fd, VIDIOC_STREAMOFF, &type);
        }
    }
    i = config_count; // deinit all web_cam
fail0:
    ESP_LOGE(TAG, "[PIPELINE_DEBUG] Initialization failed! Cleaning up resources...");
    for (int j = i - 1; j >= 0; j--) {
        if (is_valid_web_cam(&wc->video[j])) {
            ESP_LOGI(TAG, "[PIPELINE_DEBUG] Deinitializing video%d...", j);
            deinit_web_cam_video(&wc->video[j]);
        }
    }
    free(wc);
    return ret;
}

static void free_web_cam(web_cam_t *web_cam)
{
    for (int i = 0; i < web_cam->video_count; i++) {
        if (is_valid_web_cam(&web_cam->video[i])) {
            deinit_web_cam_video(&web_cam->video[i]);
        }
    }
    free(web_cam);
}

static esp_err_t http_server_init(web_cam_t *web_cam)
{
    httpd_handle_t stream_httpd;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    /* Unified static file handler for all static resources */
    httpd_uri_t static_file_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = static_file_handler,
        .user_ctx = (void *)web_cam
    };

    /* API handlers */
    httpd_uri_t capture_image_uri = {
        .uri = "/api/capture_image",
        .method = HTTP_GET,
        .handler = capture_image_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t camera_info_uri = {
        .uri = "/api/get_camera_info",
        .method = HTTP_GET,
        .handler = camera_info_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t camera_settings_uri = {
        .uri = "/api/set_camera_config",
        .method = HTTP_POST,
        .handler = camera_settings_handler,
        .user_ctx = (void *)web_cam
    };

    config.stack_size = 1024 * 8;
    ESP_LOGI(TAG, "Starting stream server on port: '%d'", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        /* Register API handlers (more specific URIs) */
        httpd_register_uri_handler(stream_httpd, &capture_image_uri);
        httpd_register_uri_handler(stream_httpd, &camera_info_uri);
        httpd_register_uri_handler(stream_httpd, &camera_settings_uri);

        /* Register wildcard static file handler to catch all other requests */
        httpd_register_uri_handler(stream_httpd, &static_file_uri);
    }

    for (int i = 0; i < web_cam->video_count; i++) {
        if (!is_valid_web_cam(&web_cam->video[i])) {
            continue;
        }

        httpd_uri_t stream_0_uri = {
            .uri = "/stream",
            .method = HTTP_GET,
            .handler = image_stream_handler,
            .user_ctx = (void *) &web_cam->video[i]
        };

        config.stack_size = 1024 * 8;
        config.server_port += 1;
        config.ctrl_port += 1;
        if (httpd_start(&stream_httpd, &config) == ESP_OK) {
            httpd_register_uri_handler(stream_httpd, &stream_0_uri);
        }
    }

    return ESP_OK;
}

static esp_err_t start_cam_web_server(const web_cam_video_config_t *config, int config_count)
{
    esp_err_t ret;
    web_cam_t *web_cam;

    ESP_RETURN_ON_ERROR(new_web_cam(config, config_count, &web_cam), TAG, "Failed to new web cam");
    ESP_GOTO_ON_ERROR(http_server_init(web_cam), fail0, TAG, "Failed to init http server");

    return ESP_OK;

fail0:
    free_web_cam(web_cam);
    return ret;
}

static void initialise_mdns(void)
{
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(EXAMPLE_MDNS_HOST_NAME));
    ESP_ERROR_CHECK(mdns_instance_name_set(EXAMPLE_MDNS_INSTANCE));

    mdns_txt_item_t serviceTxtData[] = {
        {"board", CONFIG_IDF_TARGET},
        {"path", "/"}
    };

    ESP_ERROR_CHECK(mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, serviceTxtData,
                                     sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
}

static void example_app_task(void *arg)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    initialise_mdns();
    netbiosns_init();
    netbiosns_set_name(EXAMPLE_MDNS_HOST_NAME);

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    ESP_ERROR_CHECK(example_video_init());

    // Start shell task for camera control
    xTaskCreate(shell_task, "shell_task", 4096, NULL, 5, NULL);

    web_cam_video_config_t config[] = {
#if EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
        {
            .dev_name = ESP_VIDEO_MIPI_CSI_DEVICE_NAME,
        },
#endif /* EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR */
#if EXAMPLE_ENABLE_DVP_CAM_SENSOR
        {
            .dev_name = ESP_VIDEO_DVP_DEVICE_NAME,
        },
#endif /* EXAMPLE_ENABLE_DVP_CAM_SENSOR */
#if EXAMPLE_ENABLE_SPI_CAM_SENSOR
        {
            .dev_name = ESP_VIDEO_SPI_DEVICE_NAME,
        }
#endif /* EXAMPLE_ENABLE_SPI_CAM_SENSOR */
#if EXAMPLE_ENABLE_USB_UVC_CAM_SENSOR
        {
            .dev_name = ESP_VIDEO_USB_UVC_DEVICE_NAME(0),
        }
#endif /* EXAMPLE_ENABLE_USB_UVC_CAM_SENSOR */
    };
    int config_count = sizeof(config) / sizeof(config[0]);

    assert(config_count > 0);
    ESP_ERROR_CHECK(start_cam_web_server(config, config_count));

    ESP_LOGI(TAG, "Camera web server starts");

    vTaskDelete(NULL);
}

void app_main(void)
{
    // Run the example in a dedicated task with a larger stack to avoid main task overflow
    const uint32_t stack_size = 12288; // bytes
    xTaskCreate(example_app_task, "svs_example", stack_size, NULL, 5, NULL);
}
static esp_err_t restart_video_stream(web_cam_video_t *video)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_buffer buf;
    ESP_LOGW(TAG, "[RECOVER] Restarting stream video%d", video->index);

    // Stop streaming
    int r = ioctl(video->fd, VIDIOC_STREAMOFF, &type);
    if (r != 0) {
        ESP_LOGE(TAG, "[RECOVER] STREAMOFF failed errno=%d", errno);
    }

    // Requeue all mmapped buffers
    for (uint32_t i = 0; i < video->buffer_count; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        
        buf.index  = i;
        if (ioctl(video->fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "[RECOVER] QBUF failed on buffer %lu", (unsigned long)i);
        }
    }

    // Start streaming again
    if (ioctl(video->fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "[RECOVER] STREAMON failed errno=%d", errno);
        return ESP_FAIL;
    }

    ESP_LOGW(TAG, "[RECOVER] Stream restarted video%d", video->index);
    return ESP_OK;
}

static void producer_task(void *arg)
{
    web_cam_video_t *video = (web_cam_video_t *)arg;
    struct v4l2_buffer buf;
    video->last_pub_us = esp_timer_get_time();
    video->meas_frames_window = 0;
    video->meas_window_start_us = video->last_pub_us;
    int64_t last_restart_us = 0;

    for (;;) {
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // Time-based stall watchdog: if no publications for >3s, force restart
        {
            int64_t now_watch = esp_timer_get_time();
            if (now_watch - video->last_pub_us > 3000000) {
                if (now_watch - last_restart_us > 2000000) { // avoid thrash
                    ESP_LOGW(TAG, "[PROD] video%d watchdog: >3s without publications, restarting", video->index);
                    if (restart_video_stream(video) == ESP_OK) {
                        video->stats_recoveries++;
                    } else {
                        video->stats_restart_fail++;
                    }
                    last_restart_us = now_watch;
                    continue;
                }
            }
        }

        // Blocking DQBUF (camera); stall recovery handled by publication watchdog above

        int64_t t0 = esp_timer_get_time();
        int dq = ioctl(video->fd, VIDIOC_DQBUF, &buf);
        int64_t t1 = esp_timer_get_time();
        if (dq != 0) {
            video->stats_dq_errors++;
            ESP_LOGE(TAG, "[PROD] video%d DQBUF erro errno=%d (%s)", video->index, errno, strerror(errno));
            if (restart_video_stream(video) == ESP_OK) {
                video->stats_recoveries++;
            } else {
                video->stats_restart_fail++;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        ESP_LOGD(TAG, "[PROD] video%d DQBUF idx=%d bytes=%lu dt=%lldus", video->index, (int)buf.index, (unsigned long)buf.bytesused, (long long)(t1 - t0));

        if (video->dq_fps_samples < 30) {
            if (video->dq_last_us != 0) {
                int64_t dt = t1 - video->dq_last_us;
                if (dt > 0) {
                    float dq_fps = 1000000.0f / (float)dt;
                    ESP_LOGI(TAG, "[PROD] video%d sensor frame %lu -> %.2f fps (raw)",
                             video->index,
                             (unsigned long)(video->dq_fps_samples + 1),
                             dq_fps);
                }
            }
            video->dq_last_us = t1;
            video->dq_fps_samples++;
        }

        if (!(buf.flags & V4L2_BUF_FLAG_DONE)) {
            ioctl(video->fd, VIDIOC_QBUF, &buf);
            continue;
        }

        // Acquire an output slot (or drop oldest)
        uint8_t slot;
        frame_item_t drop;
        if (xQueueReceive(video->free_q, &slot, 0) != pdTRUE) {
            if (xQueueReceive(video->frame_q, &drop, 0) == pdTRUE) {
                slot = drop.slot; // reuse the oldest slot
            } else {
                // No slot available: drop this frame
                ioctl(video->fd, VIDIOC_QBUF, &buf);
                continue;
            }
        }

        uint32_t out_size = 0;
        if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
            uint32_t len = buf.bytesused;
            if (len > video->out_buf_size) len = video->out_buf_size;
            memcpy(video->out_bufs[slot], video->buffer[buf.index], len);
            out_size = len;
        } else if (video->use_m2m_jpeg) {
            // m2m JPEG: Q camera buffer pointer to m2m OUTPUT, DQ m2m CAPTURE
            struct v4l2_buffer mout = {0};
            mout.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            mout.memory = V4L2_MEMORY_USERPTR;
            mout.index = 0; // index is not significant for USERPTR path
            mout.m.userptr = (unsigned long)video->buffer[buf.index];
            mout.length = buf.bytesused;
            mout.bytesused = buf.bytesused;
            if (ioctl(video->m2m_fd, VIDIOC_QBUF, &mout) != 0) {
                // fail safe: drop
                xQueueSend(video->free_q, &slot, 0);
                ioctl(video->fd, VIDIOC_QBUF, &buf);
                continue;
            }
            struct v4l2_buffer mcap = {0};
            mcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            mcap.memory = V4L2_MEMORY_MMAP;
            if (ioctl(video->m2m_fd, VIDIOC_DQBUF, &mcap) != 0) {
                // Blocking m2m CAP DQ failed: clean OUTPUT if any and drop this camera frame
                struct v4l2_buffer mout_clean = {0};
                mout_clean.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
                mout_clean.memory = V4L2_MEMORY_USERPTR;
                (void)ioctl(video->m2m_fd, VIDIOC_DQBUF, &mout_clean);
                xQueueSend(video->free_q, &slot, 0);
                ioctl(video->fd, VIDIOC_QBUF, &buf);
                continue;
            }
            uint32_t len = mcap.bytesused;
            if (len > video->out_buf_size) len = video->out_buf_size;
            memcpy(video->out_bufs[slot], video->m2m_cap_buf[mcap.index], len);
            out_size = len;
            // return m2m CAP buffer
            struct v4l2_buffer mcap_ret = {0};
            mcap_ret.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            mcap_ret.memory = V4L2_MEMORY_MMAP;
            mcap_ret.index = mcap.index;
            ioctl(video->m2m_fd, VIDIOC_QBUF, &mcap_ret);
            // clean m2m OUTPUT userptr
            struct v4l2_buffer mout_clean2 = {0};
            mout_clean2.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            mout_clean2.memory = V4L2_MEMORY_USERPTR;
            ioctl(video->m2m_fd, VIDIOC_DQBUF, &mout_clean2);
        } else {
            // Encode RAW -> JPEG
            int64_t e0 = esp_timer_get_time();
            if (xSemaphoreTake(video->sem, pdMS_TO_TICKS(500)) != pdTRUE) {
                // failed to get encoder, drop
                xQueueSend(video->free_q, &slot, 0);
                ioctl(video->fd, VIDIOC_QBUF, &buf);
                continue;
            }
            esp_err_t enc_ret = example_encoder_process(video->encoder_handle, video->buffer[buf.index], video->buffer_size,
                                        video->out_bufs[slot], video->out_buf_size, &out_size);
            int64_t e1 = esp_timer_get_time();
            if (enc_ret != ESP_OK) {
                xSemaphoreGive(video->sem);
                xQueueSend(video->free_q, &slot, 0);
                ioctl(video->fd, VIDIOC_QBUF, &buf);
                ESP_LOGE(TAG, "[PROD] video%d encode failed", video->index);
                continue;
            }
            xSemaphoreGive(video->sem);
            ESP_LOGD(TAG, "[PROD] video%d encode OK size=%lu dt=%lldus", video->index, (unsigned long)out_size, (long long)(e1 - e0));
        }

        // Return the camera buffer ASAP
        ioctl(video->fd, VIDIOC_QBUF, &buf);

        // Publish the frame (or drop if queue full)
        frame_item_t item = { .slot = slot, .size = out_size };

        // Drop oldest frame if queue is full to avoid stalling HTTP
        if (xQueueSend(video->frame_q, &item, 0) != pdTRUE) {
            frame_item_t oldest;
            if (xQueueReceive(video->frame_q, &oldest, 0) == pdTRUE) {
                // return slot of the dropped frame
                xQueueSend(video->free_q, &oldest.slot, 0);
                video->stats_drops++;
                ESP_LOGW(TAG, "[PROD] video%d drop oldest (free=%lu queued=%lu)",
                         video->index,
                         (unsigned long)uxQueueMessagesWaiting(video->free_q),
                         (unsigned long)uxQueueMessagesWaiting(video->frame_q));
            }
            // retry enqueue (should succeed now)
            if (xQueueSend(video->frame_q, &item, 0) != pdTRUE) {
                // give up: return slot to free list
                xQueueSend(video->free_q, &slot, 0);
                continue;
            }
        }

        {
            video->stats_published++;
            if (video->stats_published <= 30) {
                int64_t now_us = esp_timer_get_time();
                int64_t delta_us = now_us - video->last_pub_us;
                if (delta_us > 0) {
                    float inst_fps = 1000000.0f / (float)delta_us;
                    ESP_LOGI(TAG, "[PROD] video%d early frame %lu -> %.2f fps", video->index,
                             (unsigned long)video->stats_published, inst_fps);
                }
                video->last_pub_us = now_us;
            } else if (video->stats_published == 31) {
                // reset last_pub_us to maintain existing watchdog timing
                video->last_pub_us = esp_timer_get_time();
            } else {
                video->last_pub_us = esp_timer_get_time();
            }
            video->meas_frames_window++;
            int64_t noww = video->last_pub_us;
            int64_t win_dt = noww - video->meas_window_start_us;
            if (win_dt >= 1000000) { // ~1s window
#if CONFIG_EXAMPLE_PERF_LOGS
                uint32_t meas_fps = (uint32_t)(video->meas_frames_window * 1000000ULL / (uint64_t)win_dt);
                ESP_LOGD(TAG, "[PROD] video%d measured FPS=%lu, expected=%lu",
                         video->index, (unsigned long)meas_fps, (unsigned long)video->expected_fps);
#endif
                video->meas_frames_window = 0;
                video->meas_window_start_us = noww;
            }
            if ((video->stats_published % 60) == 0) {
#if CONFIG_EXAMPLE_PERF_LOGS
                UBaseType_t hw = uxTaskGetStackHighWaterMark(NULL);
                ESP_LOGI(TAG, "[PROD] video%d pub=%lu drop=%lu dqerr=%lu recov=%lu selto=%lu free=%lu queued=%lu stackfree=%lu",
                         video->index,
                         (unsigned long)video->stats_published,
                         (unsigned long)video->stats_drops,
                         (unsigned long)video->stats_dq_errors,
                         (unsigned long)video->stats_recoveries,
                         (unsigned long)video->stats_select_timeouts,
                         (unsigned long)uxQueueMessagesWaiting(video->free_q),
                         (unsigned long)uxQueueMessagesWaiting(video->frame_q),
                         (unsigned long)hw);
#endif
            }
        }
    }
}

// Minimal helper to set V4L2 ext controls (used for JPEG m2m quality)
#if CONFIG_ESP_VIDEO_ENABLE_HW_JPEG_VIDEO_DEVICE
static esp_err_t set_codec_control(int fd, uint32_t ctrl_class, uint32_t id, int32_t value)
{
    struct v4l2_ext_controls controls = {0};
    struct v4l2_ext_control control[1] = {0};
    controls.ctrl_class = ctrl_class;
    controls.count = 1;
    controls.controls = control;
    control[0].id = id;
    control[0].value = value;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to set control: %" PRIu32, id);
        return ESP_FAIL;
    }
    return ESP_OK;
}
#endif
