/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "example_video_common.h"
#include "usb/usb_host.h"
#if CONFIG_EXAMPLE_VIDEO_BUFFER_TYPE_USER
#include "esp_heap_caps.h"

#define MEMORY_TYPE V4L2_MEMORY_USERPTR
#define MEMORY_ALIGN 64
#else
#define MEMORY_TYPE V4L2_MEMORY_MMAP
#endif

#define BUFFER_COUNT 2
#define CAPTURE_SECONDS 10

static const char *TAG = "example";

static esp_err_t camera_capture_stream(void)
{
    int fd;
    esp_err_t ret;
    int fmt_index = 0;
    uint32_t frame_size;
    uint32_t frame_count;
    struct v4l2_buffer buf;
    uint8_t *buffer[BUFFER_COUNT];
#if CONFIG_EXAMPLE_VIDEO_BUFFER_TYPE_USER
    uint32_t buffer_size[BUFFER_COUNT];
#endif
    struct v4l2_format init_format;
    struct v4l2_requestbuffers req;
    struct v4l2_capability capability;
#if CONFIG_EXAMPLE_ENABLE_CAM_SENSOR_PIC_VFLIP || CONFIG_EXAMPLE_ENABLE_CAM_SENSOR_PIC_HFLIP
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
#endif
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    vTaskDelay(3000 / portTICK_PERIOD_MS); // Give user some time to plug in the camera
    fd = open(ESP_VIDEO_USB_UVC_DEVICE_NAME, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "failed to open device");
        return ESP_FAIL;
    }

    if (ioctl(fd, VIDIOC_QUERYCAP, &capability)) {
        ESP_LOGE(TAG, "failed to get capability");
        ret = ESP_FAIL;
        goto exit_0;
    }

    ESP_LOGI(TAG, "version: %d.%d.%d", (uint16_t)(capability.version >> 16),
             (uint8_t)(capability.version >> 8),
             (uint8_t)capability.version);
    ESP_LOGI(TAG, "driver:  %s", capability.driver);
    ESP_LOGI(TAG, "card:    %s", capability.card);
    ESP_LOGI(TAG, "bus:     %s", capability.bus_info);
    ESP_LOGI(TAG, "capabilities:");
    if (capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
        ESP_LOGI(TAG, "\tVIDEO_CAPTURE");
    }
    if (capability.capabilities & V4L2_CAP_READWRITE) {
        ESP_LOGI(TAG, "\tREADWRITE");
    }
    if (capability.capabilities & V4L2_CAP_ASYNCIO) {
        ESP_LOGI(TAG, "\tASYNCIO");
    }
    if (capability.capabilities & V4L2_CAP_STREAMING) {
        ESP_LOGI(TAG, "\tSTREAMING");
    }
    if (capability.capabilities & V4L2_CAP_META_OUTPUT) {
        ESP_LOGI(TAG, "\tMETA_OUTPUT");
    }
    if (capability.capabilities & V4L2_CAP_TIMEPERFRAME) {
        ESP_LOGI(TAG, "\tTIMEPERFRAME");
    }
    if (capability.capabilities & V4L2_CAP_DEVICE_CAPS) {
        ESP_LOGI(TAG, "device capabilities:");
        if (capability.device_caps & V4L2_CAP_VIDEO_CAPTURE) {
            ESP_LOGI(TAG, "\tVIDEO_CAPTURE");
        }
        if (capability.device_caps & V4L2_CAP_READWRITE) {
            ESP_LOGI(TAG, "\tREADWRITE");
        }
        if (capability.device_caps & V4L2_CAP_ASYNCIO) {
            ESP_LOGI(TAG, "\tASYNCIO");
        }
        if (capability.device_caps & V4L2_CAP_STREAMING) {
            ESP_LOGI(TAG, "\tSTREAMING");
        }
        if (capability.device_caps & V4L2_CAP_META_OUTPUT) {
            ESP_LOGI(TAG, "\tMETA_OUTPUT");
        }
        if (capability.device_caps & V4L2_CAP_TIMEPERFRAME) {
            ESP_LOGI(TAG, "\tTIMEPERFRAME");
        }
    }

    memset(&init_format, 0, sizeof(struct v4l2_format));
    init_format.type = type;
    if (ioctl(fd, VIDIOC_G_FMT, &init_format) != 0) {
        ESP_LOGE(TAG, "failed to get format");
        ret = ESP_FAIL;
        goto exit_0;
    }

#if CONFIG_EXAMPLE_ENABLE_CAM_SENSOR_PIC_VFLIP
    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_VFLIP;
    control[0].value    = 1;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to mirror the frame horizontally and skip this step");
    }
#endif

#if CONFIG_EXAMPLE_ENABLE_CAM_SENSOR_PIC_HFLIP
    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_HFLIP;
    control[0].value    = 1;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to mirror the frame horizontally and skip this step");
    }
#endif

    while (1) {
        struct v4l2_fmtdesc fmtdesc = {
            .index = fmt_index++,
            .type = type,
        };

        if (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != 0) {
            break;
        }

        struct v4l2_format format = {
            .type = type,
            .fmt.pix.width = 160,
            .fmt.pix.height = 120,
            .fmt.pix.pixelformat = fmtdesc.pixelformat,
        };

        if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) {
            if (errno == ESRCH) {
                continue;
            } else {
                ESP_LOGE(TAG, "failed to set format");
                ret = ESP_FAIL;
                goto exit_0;
            }
        }

        ESP_LOGI(TAG, "Capture %s format frames for %d seconds:", (char *)fmtdesc.description, CAPTURE_SECONDS);

        memset(&req, 0, sizeof(req));
        req.count  = BUFFER_COUNT;
        req.type   = type;
        req.memory = MEMORY_TYPE;
        if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
            ESP_LOGE(TAG, "failed to require buffer");
            ret = ESP_FAIL;
            goto exit_0;
        }

        for (int i = 0; i < BUFFER_COUNT; i++) {
            struct v4l2_buffer buf;

            memset(&buf, 0, sizeof(buf));
            buf.type        = type;
            buf.memory      = MEMORY_TYPE;
            buf.index       = i;
            if (ioctl(fd, VIDIOC_QUERYBUF, &buf) != 0) {
                ESP_LOGE(TAG, "failed to query buffer");
                ret = ESP_FAIL;
                goto exit_0;
            }

#if CONFIG_EXAMPLE_VIDEO_BUFFER_TYPE_USER
            buffer[i] = heap_caps_aligned_alloc(MEMORY_ALIGN, buf.length, MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
#else
            buffer[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                        MAP_SHARED, fd, buf.m.offset);
#endif
            if (!buffer[i]) {
                ESP_LOGE(TAG, "failed to map buffer");
                ret = ESP_FAIL;
                goto exit_0;
            }
#if CONFIG_EXAMPLE_VIDEO_BUFFER_TYPE_USER
            else {
                buf.m.userptr = (unsigned long)buffer[i];
                buffer_size[i] = buf.length;
            }
#endif

            if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
                ESP_LOGE(TAG, "failed to queue video frame");
                ret = ESP_FAIL;
                goto exit_0;
            }
        }

        if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
            ESP_LOGE(TAG, "failed to start stream");
            ret = ESP_FAIL;
            goto exit_0;
        }

        frame_count = 0;
        frame_size = 0;
        int64_t start_time_us = esp_timer_get_time();
        while (esp_timer_get_time() - start_time_us < (CAPTURE_SECONDS * 1000 * 1000)) {
            memset(&buf, 0, sizeof(buf));
            buf.type   = type;
            buf.memory = MEMORY_TYPE;
            if (ioctl(fd, VIDIOC_DQBUF, &buf) != 0) {
                ESP_LOGE(TAG, "failed to receive video frame");
                ret = ESP_FAIL;
                goto exit_0;
            }

            /**
             * If no error, the flags has V4L2_BUF_FLAG_DONE. If error, the flags has V4L2_BUF_FLAG_ERROR.
             * We need to skip these frames, but we also need queue the buffer.
             */
            if (buf.flags & V4L2_BUF_FLAG_DONE) {
                frame_size += buf.bytesused;
                frame_count++;
            }

#if CONFIG_EXAMPLE_VIDEO_BUFFER_TYPE_USER
            buf.m.userptr = (unsigned long)buffer[buf.index];
            buf.length = buffer_size[buf.index];
#endif
            if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
                ESP_LOGE(TAG, "failed to queue video frame");
                ret = ESP_FAIL;
                goto exit_0;
            }
        }

        if (ioctl(fd, VIDIOC_STREAMOFF, &type) != 0) {
            ESP_LOGE(TAG, "failed to stop stream");
            ret = ESP_FAIL;
            goto exit_0;
        }

#if CONFIG_EXAMPLE_VIDEO_BUFFER_TYPE_USER
        for (int i = 0; i < BUFFER_COUNT; i++) {
            heap_caps_free(buffer[i]);
        }
#endif

        if (frame_count > 0) {
            ESP_LOGI(TAG, "\twidth:  %" PRIu32, format.fmt.pix.width);
            ESP_LOGI(TAG, "\theight: %" PRIu32, format.fmt.pix.height);
            ESP_LOGI(TAG, "\tsize:   %" PRIu32, frame_size / frame_count);
            ESP_LOGI(TAG, "\tFPS:    %" PRIu32, frame_count / CAPTURE_SECONDS);
        } else {
            ESP_LOGW(TAG, "No frame captured");
        }
    }

    ret = ESP_OK;

exit_0:
    close(fd);
    return ret;
}

esp_err_t esp_video_install_usb_uvc_driver(size_t task_stack, unsigned task_priority, int task_affinity);
static void usb_lib_task(void *arg)
{
    // Install USB Host driver. Should only be called once in entire application
    ESP_LOGI(TAG, "Installing USB Host");
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_ERROR_CHECK(esp_video_install_usb_uvc_driver(4096, 10, 0));

    ESP_LOGI(TAG, "USB Host and UVC driver installed");
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
        }
    }
}

void app_main(void)
{
    // Create a task that will handle USB library events
    BaseType_t task_created = xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, 11, NULL, tskNO_AFFINITY);
    assert(task_created == pdTRUE);

    esp_err_t ret = ESP_OK;

    // ret = example_video_init();
    // ESP_GOTO_ON_ERROR(ret, clean1, TAG, "Camera init failed");

    ret = camera_capture_stream();
    ESP_GOTO_ON_ERROR(ret, clean0, TAG, "Camera capture stream failed");

clean0:
//     ESP_ERROR_CHECK(example_video_deinit());
// clean1:
    return;
}
