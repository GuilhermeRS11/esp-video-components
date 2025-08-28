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
#include "esp_sccb_intf.h"
#include "imx708_regs.h"
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
    },
    // Custom 800x640 15fps
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 182400000,
            .vts = 2048,
            .hts = 15648,
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
    },
    {
        .name = "MIPI_2lane_24Minput_RAW10_800x640_15fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 800,
        .height = 640,
        .regs = imx708_800x640_regs,
        .regs_size = ARRAY_SIZE(imx708_800x640_regs),
        .fps = 15,
        .isp_info = &imx708_isp_info[5],
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
    ESP_LOGD(TAG, "IMX708 read: Attempting to read register 0x%04X from I2C addr 0x%02X", reg, IMX708_SCCB_ADDR);
    esp_err_t ret = esp_sccb_transmit_receive_reg_a16v8(sccb_handle, reg, read_buf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMX708 read: Failed to read register 0x%04X, error: %s", reg, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "IMX708 read: Successfully read register 0x%04X = 0x%02X", reg, *read_buf);
    }
    return ret;
}

static esp_err_t imx708_write(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t data)
{
    ESP_LOGD(TAG, "IMX708 write: Attempting to write 0x%02X to register 0x%04X at I2C addr 0x%02X", data, reg, IMX708_SCCB_ADDR);
    esp_err_t ret = esp_sccb_transmit_reg_a16v8(sccb_handle, reg, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMX708 write: Failed to write 0x%02X to register 0x%04X, error: %s", data, reg, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "IMX708 write: Successfully wrote 0x%02X to register 0x%04X", data, reg);
    }
    return ret;
}

/* write a array of registers */
static esp_err_t imx708_write_array(esp_sccb_io_handle_t sccb_handle, const imx708_reginfo_t *regarray)
{
    int i = 0;
    esp_err_t ret = ESP_OK;
    while ((ret == ESP_OK) && regarray[i].reg != 0xffff) {
        if (regarray[i].reg != 0xeeee) {
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
    return imx708_set_reg_bits(dev->sccb_handle, 0x0600, 0, 1, enable ? 0x01 : 0x00);
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
    esp_err_t ret = imx708_set_reg_bits(dev->sccb_handle, 0x0100, 0, 1, 0);
    delay_ms(1);
    return ret;
}

static esp_err_t imx708_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t pid_h, pid_l;

    ESP_LOGI(TAG, "IMX708 get_sensor_id: Reading chip ID from registers 0x0016 and 0x0017");
    ESP_LOGI(TAG, "IMX708 get_sensor_id: Using I2C address 0x%02X", IMX708_SCCB_ADDR);

    // Try common alternative I2C addresses for IMX708 if the default fails
    uint8_t addresses[] = {IMX708_SCCB_ADDR, 0x10, 0x36, 0x20, 0x34};
    int num_addresses = sizeof(addresses) / sizeof(addresses[0]);
    
    for (int addr_idx = 0; addr_idx < num_addresses; addr_idx++) {
        if (addr_idx > 0) {
            ESP_LOGI(TAG, "IMX708 get_sensor_id: Trying alternative I2C address 0x%02X", addresses[addr_idx]);
        }
        
        ret = imx708_read(dev->sccb_handle, 0x0016, &pid_h);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "IMX708 get_sensor_id: Read pid_h = 0x%02X at address 0x%02X", pid_h, addresses[addr_idx]);

            ret = imx708_read(dev->sccb_handle, 0x0017, &pid_l);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "IMX708 get_sensor_id: Read pid_l = 0x%02X at address 0x%02X", pid_l, addresses[addr_idx]);

                id->pid = (pid_h << 8) | pid_l;
                ESP_LOGI(TAG, "IMX708 get_sensor_id: Calculated chip ID = 0x%04X (expected: 0x%04X) at address 0x%02X", 
                        id->pid, IMX708_PID, addresses[addr_idx]);
                
                if (id->pid == IMX708_PID) {
                    ESP_LOGI(TAG, "IMX708 get_sensor_id: Successfully found IMX708 at I2C address 0x%02X", addresses[addr_idx]);
                    return ESP_OK;
                }
            } else {
                ESP_LOGE(TAG, "IMX708 get_sensor_id: Failed to read pid_l from register 0x0017 at address 0x%02X, error: %s", 
                        addresses[addr_idx], esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(TAG, "IMX708 get_sensor_id: Failed to read pid_h from register 0x0016 at address 0x%02X, error: %s", 
                    addresses[addr_idx], esp_err_to_name(ret));
        }
        
        if (addr_idx < num_addresses - 1) {
            ESP_LOGI(TAG, "IMX708 get_sensor_id: Address 0x%02X failed, trying next address...", addresses[addr_idx]);
        }
    }
    
    ESP_LOGE(TAG, "IMX708 get_sensor_id: IMX708 not found at any tested I2C address");
    return ESP_FAIL;
}

static esp_err_t imx708_set_stream(esp_cam_sensor_device_t *dev, int enable)
{
    esp_err_t ret = ESP_FAIL;

    ret = imx708_write(dev->sccb_handle, 0x0100, enable ? 0x01 : 0x00);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write stream mode failed");

    if (ret == ESP_OK) {
        dev->stream_status = enable;
    }
    ESP_LOGD(TAG, "Stream=%d", enable);
    return ret;
}

static esp_err_t imx708_set_mirror(esp_cam_sensor_device_t *dev, int enable)
{
    return imx708_set_reg_bits(dev->sccb_handle, 0x0101, 0, 1, enable ? 0x01 : 0x00);
}

static esp_err_t imx708_set_vflip(esp_cam_sensor_device_t *dev, int enable)
{
    return imx708_set_reg_bits(dev->sccb_handle, 0x0101, 1, 1, enable ? 0x01 : 0x00);
}

static esp_err_t imx708_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
{
    esp_err_t ret = ESP_OK;

    switch (qdesc->id) {
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

    esp_err_t ret = ESP_OK;
    
    // If format is NULL, use the default format based on Kconfig
    if (format == NULL) {
        ESP_LOGW(TAG, "imx708_set_format: format parameter is NULL, using default format");
#if defined(CONFIG_CAMERA_IMX708_MIPI_RAW10_800X640_15FPS)
        format = &imx708_format_info[5];  // 800x640 15fps
        ESP_LOGI(TAG, "imx708_set_format: Selected 800x640 format based on Kconfig");
#elif defined(CONFIG_CAMERA_IMX708_MIPI_RAW10_4608X2592_15FPS)
        format = &imx708_format_info[0];  // 4608x2592 15fps
        ESP_LOGI(TAG, "imx708_set_format: Selected 4608x2592 format based on Kconfig");
#elif defined(CONFIG_CAMERA_IMX708_MIPI_RAW10_2304X1296_30FPS)
        format = &imx708_format_info[1];  // 2304x1296 30fps
        ESP_LOGI(TAG, "imx708_set_format: Selected 2304x1296 format based on Kconfig");
#elif defined(CONFIG_CAMERA_IMX708_MIPI_RAW10_1280X720_60FPS)
        format = &imx708_format_info[2];  // 1280x720 60fps
        ESP_LOGI(TAG, "imx708_set_format: Selected 1280x720 format based on Kconfig");
#elif defined(CONFIG_CAMERA_IMX708_MIPI_RAW8_2304X1296_45FPS)
        format = &imx708_format_info[3];  // 2304x1296 45fps RAW8
        ESP_LOGI(TAG, "imx708_set_format: Selected 2304x1296 RAW8 format based on Kconfig");
#elif defined(CONFIG_CAMERA_IMX708_MIPI_RAW8_1280X720_90FPS)
        format = &imx708_format_info[4];  // 1280x720 90fps RAW8
        ESP_LOGI(TAG, "imx708_set_format: Selected 1280x720 RAW8 format based on Kconfig");
#else
        format = &imx708_format_info[2];  // Default to 720p if no config is set
        ESP_LOGI(TAG, "imx708_set_format: Using default 720p format (no Kconfig match)");
#endif
        ESP_LOGI(TAG, "imx708_set_format: Using default format: %s", format->name);
    } else {
        ESP_LOGI(TAG, "imx708_set_format: Setting format: %s", format->name);
    }

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
        ret = imx708_write(dev->sccb_handle, 0xC428, 0x00);
        if (ret == ESP_OK) {
            ret = imx708_write(dev->sccb_handle, 0xC429, qbc_intensity);
        }
#else
        ret = imx708_write(dev->sccb_handle, 0xC428, 0x01);
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

    ESP_LOGI(TAG, "IMX708 power_on: Starting power on sequence");
    ESP_LOGI(TAG, "IMX708 power_on: xclk_pin=%d, pwdn_pin=%d, reset_pin=%d", 
             dev->xclk_pin, dev->pwdn_pin, dev->reset_pin);

    if (dev->xclk_pin >= 0) {
        ESP_LOGI(TAG, "IMX708 power_on: Enabling external clock on pin %d", dev->xclk_pin);
        IMX708_ENABLE_OUT_CLOCK(dev->xclk_pin, dev->xclk_freq_hz);
    } else {
        ESP_LOGI(TAG, "IMX708 power_on: No external clock pin configured");
    }

    if (dev->pwdn_pin >= 0) {
        ESP_LOGI(TAG, "IMX708 power_on: Configuring power down pin %d", dev->pwdn_pin);
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->pwdn_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        // Active low PWDN, so set to high to power on
        gpio_set_level(dev->pwdn_pin, 0);
        ESP_LOGI(TAG, "IMX708 power_on: Set power down pin to LOW (power on)");
        delay_ms(10);
    } else {
        ESP_LOGI(TAG, "IMX708 power_on: No power down pin configured");
    }

    if (dev->reset_pin >= 0) {
        ESP_LOGI(TAG, "IMX708 power_on: Configuring reset pin %d", dev->reset_pin);
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->reset_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        gpio_set_level(dev->reset_pin, 0);
        delay_ms(1);
        gpio_set_level(dev->reset_pin, 1);
        ESP_LOGI(TAG, "IMX708 power_on: Reset sequence completed");
        delay_ms(1);
    } else {
        ESP_LOGI(TAG, "IMX708 power_on: No reset pin configured");
    }

    ESP_LOGI(TAG, "IMX708 power_on: Power on sequence completed");
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
        ESP_LOGE(TAG, "IMX708 detect: config is NULL");
        return NULL;
    }

    ESP_LOGI(TAG, "IMX708 detect: Starting detection process");
    ESP_LOGI(TAG, "IMX708 detect: SCCB addr=0x%02x, sensor_port=%d", 
             IMX708_SCCB_ADDR, config->sensor_port);

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

    // Set default format based on Kconfig
#if defined(CONFIG_CAMERA_IMX708_MIPI_RAW10_800X640_15FPS)
    dev->cur_format = &imx708_format_info[5];  // 800x640 15fps
#elif defined(CONFIG_CAMERA_IMX708_MIPI_RAW10_4608X2592_15FPS)
    dev->cur_format = &imx708_format_info[0];  // 4608x2592 15fps
#elif defined(CONFIG_CAMERA_IMX708_MIPI_RAW10_2304X1296_30FPS)
    dev->cur_format = &imx708_format_info[1];  // 2304x1296 30fps
#elif defined(CONFIG_CAMERA_IMX708_MIPI_RAW10_1280X720_60FPS)
    dev->cur_format = &imx708_format_info[2];  // 1280x720 60fps
#elif defined(CONFIG_CAMERA_IMX708_MIPI_RAW8_2304X1296_45FPS)
    dev->cur_format = &imx708_format_info[3];  // 2304x1296 45fps RAW8
#elif defined(CONFIG_CAMERA_IMX708_MIPI_RAW8_1280X720_90FPS)
    dev->cur_format = &imx708_format_info[4];  // 1280x720 90fps RAW8
#else
    dev->cur_format = &imx708_format_info[2];  // Default to 720p if no config is set
#endif

    ESP_LOGI(TAG, "IMX708 detect: Device structure initialized, calling power on");
    // Power on and detect the sensor
    if (imx708_power_on(dev) != ESP_OK) {
        ESP_LOGE(TAG, "Camera power on failed");
        goto err_free_handler;
    }

    ESP_LOGI(TAG, "IMX708 detect: Power on successful, reading sensor ID");
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
    ESP_LOGI(TAG, "IMX708 auto-detect: Called for MIPI CSI interface");
    ESP_LOGI(TAG, "IMX708 auto-detect: Config pointer: %p", config);
    if (config) {
        ESP_LOGI(TAG, "IMX708 auto-detect: Setting sensor_port to ESP_CAM_SENSOR_MIPI_CSI");
        ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    }
    esp_cam_sensor_device_t *result = imx708_detect(config);
    if (result) {
        ESP_LOGI(TAG, "IMX708 auto-detect: Detection successful!");
    } else {
        ESP_LOGE(TAG, "IMX708 auto-detect: Detection failed!");
    }
    return result;
}
#endif
