/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "esp_sccb_intf.h"
#include "esp_cam_motor.h"
#include "esp_cam_motor_detect.h"

#include "dw9807.h"

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#define delay_ms(ms)  vTaskDelay((ms > portTICK_PERIOD_MS ? ms/ portTICK_PERIOD_MS : 1))

#define DW9807_DEFAULT_FOCUS_POS (CONFIG_DW9807_DEFAULT_POSITION)
#define DW9807_MAX_FOCUS_POS     1023
#define DW9807_MIN_FOCUS_POS     0

static const char *TAG = "dw9807";

/* Simple 1-format description: direct mode control */
static const esp_cam_motor_format_t dw9807_format_info[] = {
    {
        .name = "Direct_mode",
        .mode = ESP_CAM_MOTOR_DIRECT_MODE,
        .step_period = {
            .period_in_us = 1000,   // typical control delay
            .codes_per_step = 1,
        },
        .init_position = DW9807_DEFAULT_FOCUS_POS,
        .regs = NULL,
        .regs_size = 0,
        .reserved = NULL,
    },
};

/* Low-level register helpers */
static inline esp_err_t dw9807_reg_write8(esp_sccb_io_handle_t sccb, uint8_t reg, uint8_t val)
{
    esp_err_t ret = esp_sccb_transmit_reg_a8v8(sccb, reg, val);
    ESP_LOGD(TAG, "I2C wr8: reg=0x%02X val=0x%02X ret=%d", (int)reg, (int)val, (int)ret);
    return ret;
}

static inline esp_err_t dw9807_reg_read8(esp_sccb_io_handle_t sccb, uint8_t reg, uint8_t *val)
{
    esp_err_t ret = esp_sccb_transmit_receive_reg_a8v8(sccb, reg, val);
    ESP_LOGD(TAG, "I2C rd8: reg=0x%02X -> 0x%02X ret=%d", (int)reg, (int)(val ? *val : 0), (int)ret);
    return ret;
}

/* Set 10-bit position value */
static esp_err_t dw9807_set_pos_code(esp_cam_motor_device_t *dev, int pos)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    int req = pos;
    if (pos > DW9807_MAX_FOCUS_POS) {
        pos = DW9807_MAX_FOCUS_POS;
    } else if (pos < DW9807_MIN_FOCUS_POS) {
        pos = DW9807_MIN_FOCUS_POS;
    }

    uint8_t msb = (pos >> 8) & 0x03;  // top 2 bits
    uint8_t lsb = pos & 0xFF;         // lower 8 bits

    ESP_LOGI(TAG, "Set focus: req=%d clamp=%d", req, pos);

    // Write MSB then LSB as per datasheet/reference driver
    esp_err_t ret = dw9807_reg_write8(dev->sccb_handle, DW9807_MSB_ADDR, msb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write MSB failed ret=%d", (int)ret);
        return ret;
    }

    ret = dw9807_reg_write8(dev->sccb_handle, DW9807_LSB_ADDR, lsb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write LSB failed ret=%d", (int)ret);
        return ret;
    }

    dev->current_position = pos;
    dev->moving_start_time = esp_timer_get_time();
    return ESP_OK;
}

static esp_err_t dw9807_query_para_desc(esp_cam_motor_device_t *dev, esp_cam_motor_param_desc_t *qdesc)
{
    esp_err_t ret = ESP_OK;
    switch (qdesc->id) {
    case ESP_CAM_MOTOR_POSITION_CODE:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = DW9807_MIN_FOCUS_POS;
        qdesc->number.maximum = DW9807_MAX_FOCUS_POS;
        qdesc->number.step = 1;
        qdesc->default_value = DW9807_DEFAULT_FOCUS_POS;
        break;
    case ESP_CAM_MOTOR_MOVING_START_TIME:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_U8;
        qdesc->u8.size = sizeof(int64_t);
        break;
    default:
        ESP_LOGD(TAG, "id=%" PRIx32 " is not supported", qdesc->id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    return ret;
}

static esp_err_t dw9807_get_para_value(esp_cam_motor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;
    switch (id) {
    case ESP_CAM_MOTOR_POSITION_CODE:
        ESP_RETURN_ON_FALSE(arg && size >= sizeof(uint16_t), ESP_ERR_INVALID_ARG, TAG, "Para size err");
        *(uint16_t *)arg = dev->current_position;
        break;
    case ESP_CAM_MOTOR_MOVING_START_TIME:
        ESP_RETURN_ON_FALSE(arg && size == sizeof(int64_t), ESP_ERR_INVALID_ARG, TAG, "Para size err");
        *(int64_t *)arg = dev->moving_start_time;
        break;
    default:
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }
    return ret;
}

static esp_err_t dw9807_set_para_value(esp_cam_motor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;
    switch (id) {
    case ESP_CAM_MOTOR_POSITION_CODE: {
        ESP_RETURN_ON_FALSE(arg && size >= sizeof(int), ESP_ERR_INVALID_ARG, TAG, "Para size err");
        int *value = (int *)arg;
        ESP_LOGI(TAG, "set_para POSITION_CODE=%d", *value);
        ret = dw9807_set_pos_code(dev, *value);
        break;
    }
    default:
        ESP_LOGE(TAG, "set id=%" PRIx32 " is not supported", id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    return ret;
}

static esp_err_t dw9807_query_support_formats(esp_cam_motor_device_t *dev, esp_cam_motor_fmt_array_t *formats)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);
    formats->count = 1;
    formats->fmt_array = &dw9807_format_info[0];
    return ESP_OK;
}

static esp_err_t dw9807_power_ctrl_reg(esp_cam_motor_device_t *dev, bool on)
{
    // DW9807 CTL register: 0x00 = active, 0x01 = power down
    return dw9807_reg_write8(dev->sccb_handle, DW9807_CTL_ADDR, on ? 0x00 : 0x01);
}

static esp_err_t dw9807_hw_power_on(esp_cam_motor_device_t *dev, bool en)
{
    if (dev->pwdn_pin < 0) {
        return ESP_OK;
    }
    gpio_config_t conf = { 0 };
    conf.pin_bit_mask = 1LL << dev->pwdn_pin;
    conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&conf);
    gpio_set_level(dev->pwdn_pin, en ? 1 : 0);
    delay_ms(DW9807_WAIT_STABLE_TIME);
    return ESP_OK;
}

static esp_err_t dw9807_set_format(esp_cam_motor_device_t *dev, const esp_cam_motor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    if (format == NULL) {
        format = &dw9807_format_info[CONFIG_DW9807_FORMAT_INDEX_DEFAULT];
    }

    ESP_LOGI(TAG, "Motor fmt: %s mode=%d codes_per_step=%u period_us=%u init_pos=%d",
             format->name ? format->name : "(null)", (int)format->mode,
             (unsigned)format->step_period.codes_per_step,
             (unsigned)format->step_period.period_in_us,
             (int)format->init_position);

    // Power on via control register (and optional external pin)
    ESP_RETURN_ON_ERROR(dw9807_power_ctrl_reg(dev, true), TAG, "Power on failed");

    // Move to init position
    ESP_RETURN_ON_ERROR(dw9807_set_pos_code(dev, format->init_position), TAG, "Init position failed");

    dev->cur_format = format;
    dev->current_position = format->init_position;
    return ESP_OK;
}

static esp_err_t dw9807_get_format(esp_cam_motor_device_t *dev, esp_cam_motor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);
    if (dev->cur_format == NULL) {
        return ESP_FAIL;
    }
    memcpy(format, dev->cur_format, sizeof(esp_cam_motor_format_t));
    return ESP_OK;
}

static esp_err_t dw9807_soft_standby(esp_cam_motor_device_t *dev, bool en)
{
    // For simplicity, just toggle CTL register; optionally could ramp to idle position
    ESP_LOGI(TAG, "SW standby %s", en ? "EN" : "DIS");
    ESP_RETURN_ON_ERROR(dw9807_power_ctrl_reg(dev, !en), TAG, "CTL write failed");
    delay_ms(DW9807_WAIT_STABLE_TIME);
    return ESP_OK;
}

static esp_err_t dw9807_priv_ioctl(esp_cam_motor_device_t *dev, uint32_t cmd, void *arg)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    esp_err_t ret = ESP_FAIL;
    switch (cmd) {
    case ESP_CAM_MOTOR_IOC_HW_POWER_ON:
        ESP_LOGI(TAG, "IOCTL: HW_POWER_ON %d", *(int *)arg);
        ret = dw9807_hw_power_on(dev, *(int *)arg);
        break;
    case ESP_CAM_MOTOR_IOC_SW_STANDBY:
        ESP_LOGI(TAG, "IOCTL: SW_STANDBY %d", *(int *)arg);
        ret = dw9807_soft_standby(dev, *(int *)arg);
        break;
    default:
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    return ret;
}

static esp_err_t dw9807_delete(esp_cam_motor_device_t *dev)
{
    ESP_LOGD(TAG, "del dw9807 (%p)", dev);
    if (dev) {
        free(dev);
        dev = NULL;
    }
    return ESP_OK;
}

static const esp_cam_motor_ops_t dw9807_ops = {
    .query_para_desc = dw9807_query_para_desc,
    .get_para_value = dw9807_get_para_value,
    .set_para_value = dw9807_set_para_value,
    .query_support_formats = dw9807_query_support_formats,
    .set_format = dw9807_set_format,
    .get_format = dw9807_get_format,
    .priv_ioctl = dw9807_priv_ioctl,
    .del = dw9807_delete
};

static esp_err_t dw9807_detect_check(esp_cam_motor_device_t *dev)
{
    // Basic presence check: try reading STATUS register
    uint8_t status = 0;
    esp_err_t ret = dw9807_reg_read8(dev->sccb_handle, DW9807_STATUS_ADDR, &status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Status read failed ret=%d", (int)ret);
        return ret;
    }
    ESP_LOGD(TAG, "Status=0x%02X", (int)status);
    return ESP_OK;
}

esp_cam_motor_device_t *dw9807_detect(esp_cam_motor_config_t *config)
{
    if (config == NULL) {
        return NULL;
    }
    esp_cam_motor_device_t *dev = calloc(1, sizeof(esp_cam_motor_device_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "No memory for motor");
        return NULL;
    }

    dev->name = (char *)TAG;
    dev->sccb_handle = config->sccb_handle;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->signal_pin = -1;
    dev->ops = &dw9807_ops;
    dev->cur_format = &dw9807_format_info[CONFIG_DW9807_FORMAT_INDEX_DEFAULT];

    if (dw9807_hw_power_on(dev, true) != ESP_OK) {
        ESP_LOGE(TAG, "HW power on failed");
        goto err_free;
    }

    if (dw9807_detect_check(dev) != ESP_OK) {
        ESP_LOGE(TAG, "Detect check failed");
        goto err_poweroff;
    }

    if (dw9807_set_format(dev, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "Set default format failed");
        goto err_poweroff;
    }

    ESP_LOGI(TAG, "Detected Cam motor");
    return dev;

err_poweroff:
    dw9807_hw_power_on(dev, false);
err_free:
    free(dev);
    return NULL;
}

#if CONFIG_CAM_MOTOR_DW9807_AUTO_DETECT
ESP_CAM_MOTOR_DETECT_FN(dw9807_detect, NULL, DW9807_SCCB_ADDR)
{
    return dw9807_detect(config);
}
#endif

