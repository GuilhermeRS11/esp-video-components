/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "imx708_settings.h"
#include "imx708.h"

#define IMX708_IO_MUX_LOCK(mux)
#define IMX708_IO_MUX_UNLOCK(mux)
#define IMX708_ENABLE_OUT_CLOCK(pin,clk)
#define IMX708_DISABLE_OUT_CLOCK(pin)

#define IMX708_PID         0x0708
#define IMX708_SENSOR_NAME "IMX708"
#define IMX708_AE_TARGET_DEFAULT (0x50)

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define delay_ms(ms)  vTaskDelay((ms > portTICK_PERIOD_MS ? ms/ portTICK_PERIOD_MS : 1))
#define IMX708_SUPPORT_NUM CONFIG_CAMERA_IMX708_MAX_SUPPORT

static const char *TAG = "imx708";

static const esp_cam_sensor_isp_info_t imx708_isp_info[] = {
    // Full resolution 4608x2592 15fps
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 182400000,
            .vts = 2649,
            .hts = 15648,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    },
    // 2x2 binned 2304x1296 30fps
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 182400000,
            .vts = 2154,
            .hts = 12740,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    },
    // 720p 1280x720 60fps
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 182400000,
            .vts = 1500,
            .hts = 12740,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    },
    // 2x2 binned RAW8 2304x1296 45fps
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 182400000,
            .vts = 1436,
            .hts = 12740,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    },
    // 720p RAW8 1280x720 90fps
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 182400000,
            .vts = 1000,
            .hts = 12740,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    }
};

static const esp_cam_sensor_format_t imx708_format_info[] = {
    {
        .name = "MIPI_2lane_24Minput_RAW10_4608x2592_15fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 4608,
        .height = 2592,
        .regs = imx708_4608x2592_regs,
        .regs_size = ARRAY_SIZE(imx708_4608x2592_regs),
        .fps = 15,
        .isp_info = &imx708_isp_info[0],
        .mipi_info = {
            .mipi_clk = 450000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_2lane_24Minput_RAW10_2304x1296_30fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 2304,
        .height = 1296,
        .regs = imx708_2304x1296_regs,
        .regs_size = ARRAY_SIZE(imx708_2304x1296_regs),
        .fps = 30,
        .isp_info = &imx708_isp_info[1],
        .mipi_info = {
            .mipi_clk = 450000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_2lane_24Minput_RAW10_1280x720_60fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 1280,
        .height = 720,
        .regs = imx708_1280x720_regs,
        .regs_size = ARRAY_SIZE(imx708_1280x720_regs),
        .fps = 60,
        .isp_info = &imx708_isp_info[2],
        .mipi_info = {
            .mipi_clk = 450000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_2lane_24Minput_RAW8_2304x1296_45fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW8,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 2304,
        .height = 1296,
        .regs = imx708_2304x1296_regs,
        .regs_size = ARRAY_SIZE(imx708_2304x1296_regs),
        .fps = 45,
        .isp_info = &imx708_isp_info[3],
        .mipi_info = {
            .mipi_clk = 450000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_2lane_24Minput_RAW8_1280x720_90fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW8,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 1280,
        .height = 720,
        .regs = imx708_1280x720_regs,
        .regs_size = ARRAY_SIZE(imx708_1280x720_regs),
        .fps = 90,
        .isp_info = &imx708_isp_info[4],
        .mipi_info = {
            .mipi_clk = 450000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    }
};

static esp_err_t imx708_read(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t *read_buf)
{
    return esp_sccb_transmit_receive_reg_a16v8(sccb_handle, reg, read_buf);
}

static esp_err_t imx708_write(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t data)
{
    return esp_sccb_transmit_reg_a16v8(sccb_handle, reg, data);
}

/* write a array of registers */
static esp_err_t imx708_write_array(esp_sccb_io_handle_t sccb_handle, const imx708_reginfo_t *regarray)
{
    int i = 0;
    esp_err_t ret = ESP_OK;
    while ((ret == ESP_OK) && regarray[i].reg != IMX708_REG_END) {
        if (regarray[i].reg != IMX708_REG_DELAY) {
            ret = imx708_write(sccb_handle, regarray[i].reg, regarray[i].val);
        } else {
            delay_ms(regarray[i].val);
        }
        i++;
    }
    return ret;
}

static esp_err_t imx708_set_reg_bits(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t offset, uint8_t length, uint8_t value)
{
    esp_err_t ret = ESP_OK;
    uint8_t reg_data = 0;

    ret = imx708_read(sccb_handle, reg, &reg_data);
    if (ret != ESP_OK) {
        return ret;
    }
    uint8_t mask = ((1 << length) - 1) << offset;
    value = (reg_data & ~mask) | ((value << offset) & mask);
    ret = imx708_write(sccb_handle, reg, value);
    return ret;
}

static esp_err_t imx708_set_test_pattern(esp_cam_sensor_device_t *dev, int enable)
{
    return imx708_set_reg_bits(dev->sccb_handle, IMX708_REG_TEST_PATTERN, 0, 1, enable ? 0x01 : 0x00);
}

static esp_err_t imx708_hw_reset(esp_cam_sensor_device_t *dev)
{
    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(1);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(1);
    }
    return ESP_OK;
}

static esp_err_t imx708_soft_reset(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = imx708_set_reg_bits(dev->sccb_handle, IMX708_REG_MODE_SELECT, 0, 1, IMX708_MODE_STANDBY);
    delay_ms(1);
    return ret;
}

static esp_err_t imx708_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t pid_h, pid_l;

    ret = imx708_read(dev->sccb_handle, IMX708_REG_CHIP_ID, &pid_h);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "read pid_h failed");

    ret = imx708_read(dev->sccb_handle, IMX708_REG_CHIP_ID + 1, &pid_l);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "read pid_l failed");

    id->pid = (pid_h << 8) | pid_l;
    return ret;
}

static esp_err_t imx708_set_stream(esp_cam_sensor_device_t *dev, int enable)
{
    esp_err_t ret = ESP_FAIL;

    ret = imx708_write(dev->sccb_handle, IMX708_REG_MODE_SELECT, enable ? IMX708_MODE_STREAMING : IMX708_MODE_STANDBY);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write stream mode failed");

    if (ret == ESP_OK) {
        dev->stream_status = enable;
    }
    ESP_LOGD(TAG, "Stream=%d", enable);
    return ret;
}

static esp_err_t imx708_set_mirror(esp_cam_sensor_device_t *dev, int enable)
{
    return imx708_set_reg_bits(dev->sccb_handle, IMX708_REG_ORIENTATION, 0, 1, enable ? 0x01 : 0x00);
}

static esp_err_t imx708_set_vflip(esp_cam_sensor_device_t *dev, int enable)
{
    return imx708_set_reg_bits(dev->sccb_handle, IMX708_REG_ORIENTATION, 1, 1, enable ? 0x01 : 0x00);
}

static esp_err_t imx708_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
{
    esp_err_t ret = ESP_OK;

    switch (qdesc->id) {
    case ESP_CAM_SENSOR_GAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX708_ANA_GAIN_MIN;
        qdesc->number.maximum = IMX708_ANA_GAIN_MAX;
        qdesc->number.step = IMX708_ANA_GAIN_STEP;
        qdesc->default_value = IMX708_ANA_GAIN_DEFAULT;
        break;
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX708_EXPOSURE_MIN;
        qdesc->number.maximum = IMX708_EXPOSURE_MAX;
        qdesc->number.step = IMX708_EXPOSURE_STEP;
        qdesc->default_value = IMX708_EXPOSURE_DEFAULT;
        break;
    case ESP_CAM_SENSOR_VFLIP:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = 0;
        qdesc->number.maximum = 1;
        qdesc->number.step = 1;
        qdesc->default_value = 0;
        break;
    default:
        ESP_LOGE(TAG, "id=%" PRIx32 " is not supported", qdesc->id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    return ret;
}

static esp_err_t imx708_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;

    switch (id) {
    default: {
        ESP_LOGE(TAG, "get id=%" PRIx32 " is not supported", id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    }

    return ret;
}

static esp_err_t imx708_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;

    switch (id) {
    case ESP_CAM_SENSOR_VFLIP: {
        if (size != sizeof(int)) {
            return ESP_ERR_INVALID_SIZE;
        }
        int *value = (int *)arg;
        ret = imx708_set_vflip(dev, *value);
        break;
    }
    case ESP_CAM_SENSOR_GAIN: {
        if (size != sizeof(int)) {
            return ESP_ERR_INVALID_SIZE;
        }
        int *value = (int *)arg;
        uint8_t gain_h = (*value >> 8) & 0xFF;
        uint8_t gain_l = *value & 0xFF;
        ret = imx708_write(dev->sccb_handle, IMX708_REG_ANALOG_GAIN, gain_l);
        if (ret == ESP_OK) {
            ret = imx708_write(dev->sccb_handle, IMX708_REG_ANALOG_GAIN + 1, gain_h);
        }
        break;
    }
    case ESP_CAM_SENSOR_EXPOSURE_VAL: {
        if (size != sizeof(int)) {
            return ESP_ERR_INVALID_SIZE;
        }
        int *value = (int *)arg;
        uint8_t exp_h = (*value >> 8) & 0xFF;
        uint8_t exp_l = *value & 0xFF;
        ret = imx708_write(dev->sccb_handle, IMX708_REG_EXPOSURE, exp_l);
        if (ret == ESP_OK) {
            ret = imx708_write(dev->sccb_handle, IMX708_REG_EXPOSURE + 1, exp_h);
        }
        break;
    }
    default: {
        ESP_LOGE(TAG, "set id=%" PRIx32 " is not supported", id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    }

    return ret;
}

static esp_err_t imx708_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);

    formats->count = ARRAY_SIZE(imx708_format_info);
    formats->format_array = &imx708_format_info[0];
    return ESP_OK;
}

static esp_err_t imx708_query_support_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *sensor_cap)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, sensor_cap);

    sensor_cap->fmt_raw = 1;
    return ESP_OK;
}

static esp_err_t imx708_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);

    esp_err_t ret = ESP_OK;
    /* Depending on the interface type, an available configuration is chosen */
    dev->cur_format = format;

    if (format->port == ESP_CAM_SENSOR_MIPI_CSI) {
        ESP_LOGD(TAG, "MIPI CSI format selected: %s", format->name);
        
        // Initialize sensor with common settings
        ret = imx708_write_array(dev->sccb_handle, imx708_init_reg_tbl);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "write imx708_init_reg_tbl failed");
            return ret;
        }

        // Apply format-specific settings
        ret = imx708_write_array(dev->sccb_handle, (imx708_reginfo_t *)format->regs);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "write format registers failed");
            return ret;
        }

        // Set link frequency based on configuration
#if CONFIG_CAMERA_IMX708_MIPI_LINK_FREQ_447MHZ
        ret = imx708_write_array(dev->sccb_handle, imx708_link_freq_447mhz);
#elif CONFIG_CAMERA_IMX708_MIPI_LINK_FREQ_453MHZ
        ret = imx708_write_array(dev->sccb_handle, imx708_link_freq_453mhz);
#else // Default to 450MHz
        ret = imx708_write_array(dev->sccb_handle, imx708_link_freq_450mhz);
#endif
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "write link frequency registers failed");
            return ret;
        }

        // Configure QBC adjustment if enabled
#if CONFIG_CAMERA_IMX708_QBC_ADJUST > 0
        uint8_t qbc_intensity = CONFIG_CAMERA_IMX708_QBC_ADJUST;
        ret = imx708_write(dev->sccb_handle, IMX708_LPF_INTENSITY_EN, IMX708_LPF_INTENSITY_ENABLED);
        if (ret == ESP_OK) {
            ret = imx708_write(dev->sccb_handle, IMX708_LPF_INTENSITY, qbc_intensity);
        }
#else
        ret = imx708_write(dev->sccb_handle, IMX708_LPF_INTENSITY_EN, IMX708_LPF_INTENSITY_DISABLED);
#endif

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "QBC configuration failed");
            return ret;
        }
    }

    return ret;
}

static esp_err_t imx708_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);

    esp_err_t ret = ESP_FAIL;

    if (dev->cur_format != NULL) {
        memcpy(format, dev->cur_format, sizeof(esp_cam_sensor_format_t));
        ret = ESP_OK;
    }
    return ret;
}

static esp_err_t imx708_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
{
    esp_err_t ret = ESP_OK;
    uint8_t regval;
    esp_cam_sensor_reg_val_t *reg_val;

    switch (cmd) {
    case ESP_CAM_SENSOR_IOC_HW_RESET:
        ret = imx708_hw_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_SW_RESET:
        ret = imx708_soft_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_S_REG:
        reg_val = (esp_cam_sensor_reg_val_t *)arg;
        ret = imx708_write(dev->sccb_handle, reg_val->regaddr, reg_val->value);
        break;
    case ESP_CAM_SENSOR_IOC_G_REG:
        reg_val = (esp_cam_sensor_reg_val_t *)arg;
        ret = imx708_read(dev->sccb_handle, reg_val->regaddr, &regval);
        reg_val->value = regval;
        break;
    case ESP_CAM_SENSOR_IOC_S_STREAM:
        ret = imx708_set_stream(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_S_TEST_PATTERN:
        ret = imx708_set_test_pattern(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
        ret = imx708_get_sensor_id(dev, arg);
        break;
    default:
        ESP_LOGE(TAG, "cmd=%" PRIx32 " is not supported", cmd);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }

    return ret;
}

static esp_err_t imx708_power_on(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        IMX708_ENABLE_OUT_CLOCK(dev->xclk_pin, dev->xclk_freq_hz);
    }

    if (dev->pwdn_pin >= 0) {
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->pwdn_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        // Active low PWDN, so set to high to power on
        gpio_set_level(dev->pwdn_pin, 0);
        delay_ms(10);
    }

    if (dev->reset_pin >= 0) {
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->reset_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        gpio_set_level(dev->reset_pin, 0);
        delay_ms(1);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(1);
    }

    return ret;
}

static esp_err_t imx708_power_off(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        IMX708_DISABLE_OUT_CLOCK(dev->xclk_pin);
    }

    if (dev->pwdn_pin >= 0) {
        gpio_set_level(dev->pwdn_pin, 1);
    }

    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
    }

    return ret;
}

static esp_err_t imx708_delete(esp_cam_sensor_device_t *dev)
{
    ESP_LOGD(TAG, "del imx708 (%p)", dev);
    if (dev) {
        free(dev);
    }
    return ESP_OK;
}

static const esp_cam_sensor_ops_t imx708_ops = {
    .query_para_desc = imx708_query_para_desc,
    .get_para_value = imx708_get_para_value,
    .set_para_value = imx708_set_para_value,
    .query_support_formats = imx708_query_support_formats,
    .query_support_capability = imx708_query_support_capability,
    .set_format = imx708_set_format,
    .get_format = imx708_get_format,
    .priv_ioctl = imx708_priv_ioctl,
    .del = imx708_delete
};

esp_cam_sensor_device_t *imx708_detect(esp_cam_sensor_config_t *config)
{
    esp_cam_sensor_device_t *dev = NULL;
    if (config == NULL) {
        return NULL;
    }

    dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "No memory for camera");
        return NULL;
    }

    dev->name = IMX708_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &imx708_ops;
    dev->cur_format = &imx708_format_info[0];

    // Power on and detect the sensor
    if (imx708_power_on(dev) != ESP_OK) {
        ESP_LOGE(TAG, "Camera power on failed");
        goto err_free_handler;
    }

    esp_cam_sensor_id_t sensor_id;
    if (imx708_get_sensor_id(dev, &sensor_id) != ESP_OK) {
        ESP_LOGE(TAG, "Get sensor ID failed");
        goto err_free_handler;
    } else if (sensor_id.pid != IMX708_PID) {
        ESP_LOGE(TAG, "Camera sensor is not IMX708, PID=0x%x", sensor_id.pid);
        goto err_free_handler;
    }
    ESP_LOGI(TAG, "Detected IMX708 camera sensor PID=0x%x", sensor_id.pid);

    return dev;

err_free_handler:
    imx708_power_off(dev);
    free(dev);
    return NULL;
}

#if CONFIG_CAMERA_IMX708_AUTO_DETECT_MIPI_INTERFACE_SENSOR
ESP_CAM_SENSOR_DETECT_FN(imx708_detect, ESP_CAM_SENSOR_MIPI_CSI, IMX708_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return imx708_detect(config);
}
#endif
