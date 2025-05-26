/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#ifdef CONFIG_HEAP_TRACING
#include "esp_heap_trace.h"
#endif

#include "memory_checks.h"
#include "unity_test_utils_memory.h"
#include "unity.h"

#include "example_video_common.h"

#define TEST_MEMORY_LEAK_THRESHOLD (-300)

#define VIDEO_BUFFER_NUM 2

#define TEST_APP_VIDEO_DEVICE EXAMPLE_CAM_DEV_PATH

void setUp(void);

static bool s_ut_inited;
static size_t before_free_8bit;
static size_t before_free_32bit;

static void ut_init(void)
{
    if (!s_ut_inited) {
        TEST_ESP_OK(example_video_init());
        setUp();
        s_ut_inited = true;
    }
}

TEST_CASE("V4L2 Command", "[video]")
{
    int fd;
    int ret;
    uint16_t width;
    uint16_t height;
    struct v4l2_format format;
    struct v4l2_capability cap;

    ut_init();

    fd = open(TEST_APP_VIDEO_DEVICE, O_RDWR);
    TEST_ASSERT_GREATER_OR_EQUAL(0, fd);

    memset(&cap, 0, sizeof(cap));
    ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    TEST_ESP_OK(ret);
    TEST_ASSERT_EQUAL_INT(V4L2_CAP_VIDEO_CAPTURE, cap.capabilities & V4L2_CAP_VIDEO_CAPTURE);

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(fd, VIDIOC_G_FMT, &format);
    TEST_ESP_OK(ret);

    width = format.fmt.pix.width;
    height = format.fmt.pix.height;

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
    ret = ioctl(fd, VIDIOC_S_FMT, &format);
    TEST_ESP_OK(ret);

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width - 1;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
    ret = ioctl(fd, VIDIOC_S_FMT, &format);
    TEST_ASSERT_EQUAL_INT(-1, ret);

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height - 1;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
    ret = ioctl(fd, VIDIOC_S_FMT, &format);
    TEST_ASSERT_EQUAL_INT(-1, ret);

    close(fd);
}

TEST_CASE("V4L2 M2M device", "[video]")
{
    int fd;
    int ret;
    int val;
    uint16_t width = 320;
    uint16_t height = 240;
    struct v4l2_buffer buf;
    struct v4l2_format format;
    struct v4l2_capability cap;
    struct v4l2_requestbuffers req;
    uint8_t *out_buf[VIDEO_BUFFER_NUM];
    uint8_t *cap_buf[VIDEO_BUFFER_NUM];

    ut_init();

    fd = open(ESP_VIDEO_JPEG_DEVICE_NAME, O_RDWR);
    TEST_ASSERT_GREATER_OR_EQUAL(0, fd);

    memset(&cap, 0, sizeof(cap));
    ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    TEST_ESP_OK(ret);
    TEST_ASSERT_EQUAL_INT(V4L2_CAP_VIDEO_M2M, cap.capabilities & V4L2_CAP_VIDEO_M2M);

    /* Initialize output buffer */

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
    ret = ioctl(fd, VIDIOC_S_FMT, &format);
    TEST_ESP_OK(ret);

    memset(&req, 0, sizeof(req));
    req.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    req.memory = V4L2_MEMORY_MMAP;
    req.count  = VIDEO_BUFFER_NUM;
    ret = ioctl(fd, VIDIOC_REQBUFS, &req);
    TEST_ESP_OK(ret);

    for (int i = 0; i < VIDEO_BUFFER_NUM; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type        = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;
        ret = ioctl(fd, VIDIOC_QUERYBUF, &buf);
        TEST_ESP_OK(ret);

        TEST_ASSERT_EQUAL_INT(width * height * 2, buf.length);

        out_buf[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd, buf.m.offset);
        TEST_ASSERT_NOT_NULL(out_buf[i]);

        ret = ioctl(fd, VIDIOC_QBUF, &buf);
        TEST_ESP_OK(ret);
    }

    /* Initialize capture buffer */

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;
    ret = ioctl(fd, VIDIOC_S_FMT, &format);
    TEST_ESP_OK(ret);

    memset(&req, 0, sizeof(req));
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    req.count  = VIDEO_BUFFER_NUM;
    ret = ioctl(fd, VIDIOC_REQBUFS, &req);
    TEST_ESP_OK(ret);

    for (int i = 0; i < VIDEO_BUFFER_NUM; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        ret = ioctl(fd, VIDIOC_QUERYBUF, &buf);
        TEST_ESP_OK(ret);

        cap_buf[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd, buf.m.offset);
        TEST_ASSERT_NOT_NULL(cap_buf[i]);

        ret = ioctl(fd, VIDIOC_QBUF, &buf);
        TEST_ESP_OK(ret);
    }

    val = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ret = ioctl(fd, VIDIOC_STREAMON, &val);
    TEST_ESP_OK(ret);

    val = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(fd, VIDIOC_STREAMON, &val);
    TEST_ESP_OK(ret);

    for (int i = 0; i < 100; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        ret = ioctl(fd, VIDIOC_DQBUF, &buf);
        TEST_ESP_OK(ret);

        TEST_ASSERT_EQUAL_HEX8(0xff, cap_buf[buf.index][0]);
        TEST_ASSERT_EQUAL_HEX8(0xd8, cap_buf[buf.index][1]);

        ret = ioctl(fd, VIDIOC_QBUF, &buf);
        TEST_ESP_OK(ret);

        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory = V4L2_MEMORY_MMAP;
        ret = ioctl(fd, VIDIOC_DQBUF, &buf);
        TEST_ESP_OK(ret);

        ret = ioctl(fd, VIDIOC_QBUF, &buf);
        TEST_ESP_OK(ret);
    }

    val = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ret = ioctl(fd, VIDIOC_STREAMOFF, &val);
    TEST_ESP_OK(ret);

    val = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(fd, VIDIOC_STREAMOFF, &val);
    TEST_ESP_OK(ret);

    ret = close(fd);
    TEST_ESP_OK(ret);
}

TEST_CASE("V4L2 set/get selection", "[video]")
{
    int fd;
    struct v4l2_selection in_selection;
    struct v4l2_selection out_selection;

    ut_init();

    fd = open(ESP_VIDEO_ISP1_DEVICE_NAME, O_RDWR);
    TEST_ASSERT_GREATER_OR_EQUAL(0, fd);

    memset(&in_selection, 0, sizeof(in_selection));
    in_selection.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    TEST_ASSERT_EQUAL_INT(-1, ioctl(fd, VIDIOC_G_SELECTION, &in_selection));

    memset(&in_selection, 0, sizeof(in_selection));
    in_selection.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    TEST_ASSERT_EQUAL_INT(-1, ioctl(fd, VIDIOC_S_SELECTION, &in_selection));

    memset(&in_selection, 0, sizeof(in_selection));
    in_selection.type = V4L2_BUF_TYPE_META_CAPTURE;
    TEST_ESP_OK(ioctl(fd, VIDIOC_G_SELECTION, &in_selection));

    memset(&in_selection, 0, sizeof(in_selection));
    in_selection.type = V4L2_BUF_TYPE_META_CAPTURE;
    in_selection.r.left = 0;
    in_selection.r.width = 1080;
    in_selection.r.top = 0;
    in_selection.r.height = 720;
    TEST_ESP_OK(ioctl(fd, VIDIOC_S_SELECTION, &in_selection));

    memset(&out_selection, 0, sizeof(out_selection));
    out_selection.type = V4L2_BUF_TYPE_META_CAPTURE;
    TEST_ESP_OK(ioctl(fd, VIDIOC_G_SELECTION, &out_selection));

    TEST_ESP_OK(memcmp(&out_selection, &in_selection, sizeof(out_selection)));

    TEST_ESP_OK(close(fd));
}

static void check_leak(size_t before_free, size_t after_free, const char *type)
{
    ssize_t delta = after_free - before_free;
    printf("MALLOC_CAP_%s: Before %u bytes free, After %u bytes free (delta %d)\n", type, before_free, after_free, delta);
    TEST_ASSERT_MESSAGE(delta >= TEST_MEMORY_LEAK_THRESHOLD, "memory leak");
}

void setUp(void)
{
    before_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    before_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
}

void tearDown(void)
{
    /* some FreeRTOS stuff is cleaned up by idle task */
    vTaskDelay(5);

    /* clean up some of the newlib's lazy allocations */
    esp_reent_cleanup();

    size_t after_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t after_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
    check_leak(before_free_8bit, after_free_8bit, "8BIT");
    check_leak(before_free_32bit, after_free_32bit, "32BIT");
}

void app_main(void)
{
    /**
     * \ \     /_ _| __ \  ____|  _ \
     *  \ \   /   |  |   | __|   |   |
     *   \ \ /    |  |   | |     |   |
     *    \_/   ___|____/ _____|\___/
    */

    printf("\r\n");
    printf("\\ \\     /_ _| __ \\  ____|  _ \\  \r\n");
    printf(" \\ \\   /   |  |   | __|   |   |\r\n");
    printf("  \\ \\ /    |  |   | |     |   | \r\n");
    printf("   \\_/   ___|____/ _____|\\___/  \r\n");

    unity_run_menu();
}
