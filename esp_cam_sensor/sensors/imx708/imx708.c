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
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_cam_sensor_detect.h"
#include "esp_sccb_intf.h"
#include "imx708_regs.h"
#include "imx708_settings.h"
#include "imx708.h"

/*
 * IMX708 camera sensor gain control structure.
 * IMX708 uses separate analog and digital gain controls for fine-tuned exposure control.
 */
typedef struct {
    uint16_t analog_gain;   // Analog gain value (register format)
    uint16_t digital_gain;  // Digital gain value (register format)
} imx708_gain_t;

/*
 * IMX708 camera sensor parameter tracking structure.
 * Stores current state of all dynamically controllable parameters.
 */
typedef struct {
    uint32_t exposure_val;     // Current exposure value (lines)
    uint32_t exposure_max;     // Maximum allowed exposure
    uint32_t gain_val;         // Current total gain value
    uint32_t analog_gain;      // Current analog gain
    uint32_t digital_gain;     // Current digital gain
    // If true, ignore exposure updates (fixed shutter); adjust only gain
    bool  fixed_exposure_en;
    
    // Image quality controls
    int8_t brightness;         // Brightness adjustment (-10 to +10)
    int8_t contrast;          // Contrast adjustment (-10 to +10)
    int8_t saturation;        // Saturation adjustment (-10 to +10)
    
    // Color balance controls
    uint16_t red_gain;        // Red channel gain (100-2000, 400=default)
    uint16_t blue_gain;       // Blue channel gain (100-2000, 280=default)
    
    // Orientation controls  
    uint8_t vflip_en : 1;     // Vertical flip enable
    uint8_t hmirror_en : 1;   // Horizontal mirror enable
    
    // Auto controls
    uint8_t ae_enable : 1;    // Auto exposure enable
    uint8_t awb_enable : 1;   // Auto white balance enable
    uint8_t agc_enable : 1;   // Auto gain control enable
} imx708_para_t;

/*
 * IMX708 camera sensor device context structure.
 * Contains all dynamic parameters and state information.
 */
struct imx708_cam {
    imx708_para_t imx708_para;
    // Simple rate limiter to avoid back-to-back parameter writes mid-stream
    TickType_t last_param_update_tick;
    TickType_t min_update_interval_ticks;
    // Coalescing: store last requested values when updates are rate-limited
    bool pending_exposure_valid;
    uint32_t pending_exposure_lines;
    bool pending_gain_valid;
    uint32_t pending_total_gain;
    // Warm-up guard after stream-on to avoid early unstable writes
    TickType_t warmup_end_tick;
    uint32_t warmup_frames;
    // Pending color balance updates
    bool pending_red_valid;
    uint16_t pending_red_gain;
    bool pending_blue_valid;
    uint16_t pending_blue_gain;
    // Staged values applied under current Group Hold (for consolidated logging on release)
    
    
    
};

#define IMX708_IO_MUX_LOCK(mux)
#define IMX708_IO_MUX_UNLOCK(mux)
#define IMX708_ENABLE_OUT_CLOCK(pin,clk)
#define IMX708_DISABLE_OUT_CLOCK(pin)

#define IMX708_PID         0x0708
#define IMX708_SENSOR_NAME "IMX708"
#define IMX708_AE_TARGET_DEFAULT (0x50)

// IMX708 Exposure Control Constants
// Using constants from imx708_regs.h: IMX708_EXPOSURE_MIN, IMX708_EXPOSURE_STEP, etc.
#define IMX708_EXPOSURE_MAX_OFFSET  0x0006    // Offset from VTS for max exposure
// Additional safety margin for exposure (percentage of VTS)
// Safer default headroom: cap exposure to 80% of VTS
#define IMX708_EXPOSURE_MAX_RATIO_PCT 80
// Ensure a minimum vertical blanking margin (in lines) to avoid starving blanking
#define IMX708_MIN_BLANKING_LINES     64
// Extra fixed safety margin (in microseconds) to absorb scheduling jitter near frame boundary
#define IMX708_EXPOSURE_FIXED_MARGIN_US 2000

// Stability thresholds to avoid tiny updates that cause flicker
#define IMX708_MIN_EXPOSURE_DELTA_LINES  16    // ignore exposure changes smaller than 16 lines
#define IMX708_MIN_TOTAL_GAIN_DELTA      5     // total_gain is x100; ignore < 0.05x changes

// IMX708 common registers
#define IMX708_REG_MODE_SELECT   0x0100
#define IMX708_REG_GROUP_HOLD    0x0104
// Frame length lines (VTS)
#define IMX708_REG_FRAME_LENGTH  0x0340   // 0x0340[15:8], 0x0341[7:0]

// IMX708 Gain Control Constants (align with imx708_regs.h)
#define IMX708_ANALOG_GAIN_MIN      IMX708_ANA_GAIN_MIN     // 112 (0x0070) ~ 1.0x
#define IMX708_ANALOG_GAIN_MAX      IMX708_ANA_GAIN_MAX     // 960 (0x03C0)
#define IMX708_DIGITAL_GAIN_MIN     IMX708_DGTL_GAIN_MIN    // 0x0100 = 1.0x
#define IMX708_DIGITAL_GAIN_MAX     IMX708_DGTL_GAIN_MAX    // 0xFFFF
#define IMX708_TOTAL_GAIN_MIN       100       // 1.0x scaled by 100
#define IMX708_TOTAL_GAIN_MAX       25600     // 256x scaled by 100
// Approximate max analog gain factor (x100). Empirically ~8x for code 960.
#define IMX708_ANALOG_MAX_X100      800

// IMX708 Image Quality Control Ranges
#define IMX708_BRIGHTNESS_MIN       -10       // Increased range for calibration
#define IMX708_BRIGHTNESS_MAX       10        // Increased range for calibration  
#define IMX708_CONTRAST_MIN         -10       // Increased range for calibration
#define IMX708_CONTRAST_MAX         10        // Increased range for calibration
#define IMX708_SATURATION_MIN       -10       // Increased range for calibration
#define IMX708_SATURATION_MAX       10        // Increased range for calibration

// IMX708 Color Balance Control Ranges (for Red/Blue gain adjustment)
#define IMX708_COLOR_BALANCE_RED_MIN    100   // Minimum red gain (0x0100 = 1.0x)
#define IMX708_COLOR_BALANCE_RED_MAX    2000  // Maximum red gain (0x0800 = 8.0x)
#define IMX708_COLOR_BALANCE_RED_DEFAULT 400  // Default red gain (0x0400 = 4.0x)
#define IMX708_COLOR_BALANCE_BLUE_MIN   100   // Minimum blue gain (0x0100 = 1.0x)  
#define IMX708_COLOR_BALANCE_BLUE_MAX   2000  // Maximum blue gain (0x0800 = 8.0x)
#define IMX708_COLOR_BALANCE_BLUE_DEFAULT 280 // Default blue gain (0x0280 = 2.5x)

// Freeze analog gain and drive total gain via digital gain only (for stability tests)
// Set to 1 to freeze analog gain at default and avoid AG updates while streaming.
#ifndef IMX708_FREEZE_ANALOG_GAIN
#define IMX708_FREEZE_ANALOG_GAIN 1
#endif

// Test switch: freeze exposure and total gain updates (ignore runtime changes)
// Set to 1 to disable any exposure or total gain writes after initialization
#ifndef IMX708_FREEZE_EXPOSURE
#define IMX708_FREEZE_EXPOSURE 0
#endif

#ifndef IMX708_FREEZE_TOTAL_GAIN
#define IMX708_FREEZE_TOTAL_GAIN 0
#endif

// Custom Color Balance Control IDs (using user class)
#define IMX708_COLOR_BALANCE_RED    ESP_CAM_SENSOR_CLASS_ID(ESP_CAM_SENSOR_CID_CLASS_USER, 0x0100)
#define IMX708_COLOR_BALANCE_BLUE   ESP_CAM_SENSOR_CLASS_ID(ESP_CAM_SENSOR_CID_CLASS_USER, 0x0101)

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define delay_ms(ms)  vTaskDelay((ms > portTICK_PERIOD_MS ? ms/ portTICK_PERIOD_MS : 1))
#define IMX708_SUPPORT_NUM CONFIG_CAMERA_IMX708_MAX_SUPPORT

// IMX708 IPA JSON Configuration
#if defined(CONFIG_CAMERA_IMX708_DEFAULT_IPA_JSON_CONFIGURATION_FILE)
#define IMX708_IPA_JSON_CONFIG_PATH "esp_cam_sensor/sensors/imx708/cfg/imx708_default.json"
#elif defined(CONFIG_CAMERA_IMX708_CUSTOMIZED_IPA_JSON_CONFIGURATION_FILE)
#define IMX708_IPA_JSON_CONFIG_PATH CONFIG_CAMERA_IMX708_CUSTOMIZED_IPA_JSON_CONFIGURATION_FILE_PATH
#else
#define IMX708_IPA_JSON_CONFIG_PATH NULL
#endif

static const char *TAG = "imx708";

// Forward declarations for internal setters used before definition
static esp_err_t imx708_set_total_gain(esp_cam_sensor_device_t *dev, uint32_t total_gain);

// Exposure clamp from Kconfig (ms -> us)
#define IMX708_CFG_MIN_EXPOSURE_US   ((uint32_t)CONFIG_CAMERA_IMX708_EXPOSURE_MIN_MS * 1000U)
#define IMX708_CFG_MAX_EXPOSURE_US   ((uint32_t)CONFIG_CAMERA_IMX708_EXPOSURE_MAX_MS * 1000U)

#if !CONFIG_CAMERA_IMX708_NO_IPA_JSON_CONFIGURATION_FILE
static const esp_cam_sensor_isp_info_t imx708_isp_info[] = {
    // Full resolution 4608x2592 15fps
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 182400000,
            .vts = 2649,
            .hts = 15648,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,  // Correto: RGGB
        }
    },
    // 2x2 binned 2304x1296 30fps
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 182400000,
            .vts = 2154,
            .hts = 12740,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,  // Correto: RGGB
        }
    },
    // 720p 1280x720 60fps
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 182400000,
            .vts = 1500,
            .hts = 12740,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,  // Correto: RGGB
        }
    },
    // 2x2 binned RAW8 2304x1296 45fps
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 182400000,
            .vts = 1436,
            .hts = 12740,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,  // Correto: RGGB
        }
    },
    // 720p RAW8 1280x720 90fps
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 182400000,
            .vts = 1000,
            .hts = 12740,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,  // Correto: RGGB
        }
    },
    // Custom 800x640 15fps - Centered crop
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 595200000,
            .vts = 3296,
            .hts = 12032,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,  // Correto: RGGB
        }
    }
};
#endif

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
        .isp_info = 
#if CONFIG_CAMERA_IMX708_NO_IPA_JSON_CONFIGURATION_FILE
            NULL,
#else
            &imx708_isp_info[0],
#endif
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
        .isp_info = 
#if CONFIG_CAMERA_IMX708_NO_IPA_JSON_CONFIGURATION_FILE
            NULL,
#else
            &imx708_isp_info[1],
#endif
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
        .isp_info = 
#if CONFIG_CAMERA_IMX708_NO_IPA_JSON_CONFIGURATION_FILE
            NULL,
#else
            &imx708_isp_info[2],
#endif
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
        .isp_info = 
#if CONFIG_CAMERA_IMX708_NO_IPA_JSON_CONFIGURATION_FILE
            NULL,
#else
            &imx708_isp_info[3],
#endif
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
        .isp_info = 
#if CONFIG_CAMERA_IMX708_NO_IPA_JSON_CONFIGURATION_FILE
            NULL,
#else
            &imx708_isp_info[4],
#endif
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
        .isp_info = 
#if CONFIG_CAMERA_IMX708_NO_IPA_JSON_CONFIGURATION_FILE
            NULL,
#else
            &imx708_isp_info[5],
#endif
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

// Track nested group-hold sections to avoid mid-frame releases when composing controls
static int s_group_hold_depth = 0;

static inline esp_err_t imx708_group_hold(esp_cam_sensor_device_t *dev, bool enable)
{
    // 0x0104: Grouped parameter hold (1=hold, 0=release)
    if (enable) {
        // Entering hold: only write when transitioning from 0 -> 1
        if (s_group_hold_depth == 0) {
            esp_err_t ret = imx708_write(dev->sccb_handle, IMX708_REG_GROUP_HOLD, 0x01);
            if (ret != ESP_OK) {
                return ret;
            }
        }
        s_group_hold_depth++;
        return ESP_OK;
    } else {
        // Leaving hold: protect underflow
        if (s_group_hold_depth > 0) {
            s_group_hold_depth--;
        }
        // Only release when depth reaches 0
        if (s_group_hold_depth == 0) {
            return imx708_write(dev->sccb_handle, IMX708_REG_GROUP_HOLD, 0x00);
        }
        return ESP_OK;
    }
}

static inline uint32_t imx708_min_u32(uint32_t a, uint32_t b)
{
    return a < b ? a : b;
}

// Read current frame length (VTS) from sensor registers
static esp_err_t imx708_get_frame_length_lines(esp_cam_sensor_device_t *dev, uint32_t *vts_out)
{
    uint8_t msb = 0, lsb = 0;
    esp_err_t ret = imx708_read(dev->sccb_handle, IMX708_REG_FRAME_LENGTH, &msb);
    ret |= imx708_read(dev->sccb_handle, IMX708_REG_FRAME_LENGTH + 1, &lsb);
    if (ret == ESP_OK && vts_out) {
        *vts_out = ((uint32_t)msb << 8) | lsb;
    }
    return ret;
}

// Force VTS to nominal value from current format (fixed-fps guard)
static esp_err_t imx708_enforce_nominal_vts(esp_cam_sensor_device_t *dev)
{
    if (!dev || !dev->cur_format || !dev->cur_format->isp_info) {
        // No nominal VTS available (e.g. NO_IPA build) – skip enforcement
        return ESP_OK;
    }
    // Do not enforce nominal VTS while streaming to avoid mid-frame disturbances
    if (dev->stream_status) {
        return ESP_OK;
    }
    const uint32_t nominal_vts = dev->cur_format->isp_info->isp_v1_info.vts;
    uint32_t cur_vts = 0;
    if (imx708_get_frame_length_lines(dev, &cur_vts) != ESP_OK) {
        return ESP_FAIL;
    }
    if (cur_vts != nominal_vts) {
        ESP_LOGW(TAG, "VTS drift detected: cur=%lu, nominal=%lu. Restoring.", (unsigned long)cur_vts, (unsigned long)nominal_vts);
        // Apply under group hold for clean boundary
        imx708_group_hold(dev, true);
        esp_err_t ret = imx708_write(dev->sccb_handle, IMX708_REG_FRAME_LENGTH,     (nominal_vts >> 8) & 0xFF);
        ret |=        imx708_write(dev->sccb_handle, IMX708_REG_FRAME_LENGTH + 1,  (nominal_vts     ) & 0xFF);
        imx708_group_hold(dev, false);
        return ret;
    }
    return ESP_OK;
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
        // After stream-on, apply warm-up guard to avoid early unstable writes
        if (enable && dev->cur_format) {
            struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
            if (cam_imx708) {
                uint32_t fps_local = dev->cur_format->fps ? dev->cur_format->fps : 15;
                uint32_t frame_time_ms = (fps_local > 0) ? (1000 / fps_local) : 66;
                cam_imx708->warmup_frames = 3;
                cam_imx708->warmup_end_tick = xTaskGetTickCount() + pdMS_TO_TICKS(frame_time_ms * cam_imx708->warmup_frames);
            }
        }
    }
    ESP_LOGD(TAG, "Stream=%d", enable);
    return ret;
}

static esp_err_t imx708_set_hmirror(esp_cam_sensor_device_t *dev, int enable)
{
    ESP_LOGD(TAG, "Setting horizontal mirror to %s", enable ? "enabled" : "disabled");
    return imx708_set_reg_bits(dev->sccb_handle, 0x0101, 0, 1, enable ? 0x01 : 0x00);
}

static esp_err_t imx708_set_vflip(esp_cam_sensor_device_t *dev, int enable)
{
    return imx708_set_reg_bits(dev->sccb_handle, 0x0101, 1, 1, enable ? 0x01 : 0x00);
}

/*
 * IMX708 Exposure Control Functions
 * The IMX708 uses a 16-bit exposure register (0x0202-0x0203) to control integration time.
 * Exposure is specified in number of scan lines.
 */

/**
 * @brief Set IMX708 exposure value in scan lines
 * @param dev Camera sensor device handle
 * @param exposure_val Exposure value in scan lines (4 to VTS-6)
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t imx708_set_exposure_val(esp_cam_sensor_device_t *dev, uint32_t exposure_val)
{
    esp_err_t ret = ESP_OK;
    struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;

    if (!cam_imx708) {
        ESP_LOGE(TAG, "Camera context not initialized");
        return ESP_ERR_INVALID_STATE;
    }

#if IMX708_FREEZE_EXPOSURE
    // Exposure updates are frozen for diagnostics; ignore runtime changes
    return ESP_OK;
#endif
    
    // Enforce nominal VTS (fixed fps) where available and get current VTS
    imx708_enforce_nominal_vts(dev);
    // Calculate maximum exposure based on VTS with extra ratio margin
    uint32_t vts = 0;
    if (dev->cur_format && dev->cur_format->isp_info) {
        vts = dev->cur_format->isp_info->isp_v1_info.vts;
    } else {
        (void)imx708_get_frame_length_lines(dev, &vts);
    }
    // Get current FPS from format (fallback to 15 if not available)
    uint32_t fps = (dev->cur_format && dev->cur_format->fps) ? dev->cur_format->fps : 15;
    uint32_t max_by_offset = (vts > IMX708_EXPOSURE_MAX_OFFSET) ? (vts - IMX708_EXPOSURE_MAX_OFFSET) : vts;
    uint32_t max_by_ratio  = (vts * IMX708_EXPOSURE_MAX_RATIO_PCT) / 100;
    uint32_t max_by_margin = (vts > IMX708_MIN_BLANKING_LINES) ? (vts - IMX708_MIN_BLANKING_LINES) : vts;
    // Convert fixed microsecond margin to line budget and reduce max further
    uint32_t fixed_lines_margin = (uint32_t)(((uint64_t)IMX708_EXPOSURE_FIXED_MARGIN_US * fps * vts + 999999ULL) / 1000000ULL);
    uint32_t max_by_fixed = (vts > fixed_lines_margin) ? (vts - fixed_lines_margin) : vts;
    uint32_t max_exposure  = imx708_min_u32(imx708_min_u32(max_by_offset, max_by_ratio), imx708_min_u32(max_by_margin, max_by_fixed));
    
    // Validate exposure range
    if (exposure_val < IMX708_EXPOSURE_MIN) {
        ESP_LOGW(TAG, "Exposure %lu below minimum %d, clamping", exposure_val, IMX708_EXPOSURE_MIN);
        exposure_val = IMX708_EXPOSURE_MIN;
    } else if (exposure_val > max_exposure) {
        ESP_LOGW(TAG, "Exposure %lu above maximum %lu, clamping", exposure_val, max_exposure);
        exposure_val = max_exposure;
    }
    
    // If a pending value exists and we're allowed to update now, replace request
    if (cam_imx708->pending_exposure_valid) {
        exposure_val = cam_imx708->pending_exposure_lines;
        cam_imx708->pending_exposure_valid = false;
    }

    // If unchanged or too small change, skip writes to avoid flicker
    if (cam_imx708->imx708_para.exposure_val == exposure_val ||
        (exposure_val > cam_imx708->imx708_para.exposure_val ? (exposure_val - cam_imx708->imx708_para.exposure_val) : (cam_imx708->imx708_para.exposure_val - exposure_val)) < IMX708_MIN_EXPOSURE_DELTA_LINES) {
        return ESP_OK;
    }

    // Rate-limit top-level updates to avoid hitting unsafe timing windows when streaming
    if (s_group_hold_depth == 0 && cam_imx708->min_update_interval_ticks > 0) {
        TickType_t now = xTaskGetTickCount();
        // Warm-up guard after stream on
        if (now < cam_imx708->warmup_end_tick) {
            cam_imx708->pending_exposure_lines = exposure_val;
            cam_imx708->pending_exposure_valid = true;
            return ESP_OK;
        }
        if ((now - cam_imx708->last_param_update_tick) < cam_imx708->min_update_interval_ticks) {
            // Defer: store pending and return OK
            cam_imx708->pending_exposure_lines = exposure_val;
            cam_imx708->pending_exposure_valid = true;
            return ESP_OK;
        }
    }

    ESP_LOGD(TAG, "Setting exposure to %lu lines (VTS=%lu, max=%lu)", exposure_val, vts, max_exposure);

    // Write exposure (and any pending color balance) under group hold to avoid mid-frame glitches
imx708_group_hold(dev, true);
ret  = imx708_write(dev->sccb_handle, IMX708_REG_EXPOSURE, (exposure_val >> 8) & 0xFF);
ret |= imx708_write(dev->sccb_handle, IMX708_REG_EXPOSURE + 1, exposure_val & 0xFF);
// Apply pending color balance if any
struct imx708_cam *cctx = (struct imx708_cam *)dev->priv;
if (ret == ESP_OK && cctx) {
    if (cctx->pending_red_valid) {
        ret |= imx708_write(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_RED, (cctx->pending_red_gain >> 8) & 0xFF);
        ret |= imx708_write(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_RED + 1, cctx->pending_red_gain & 0xFF);
        cctx->imx708_para.red_gain = cctx->pending_red_gain;
        cctx->pending_red_valid = false;
    }
    if (cctx->pending_blue_valid) {
        ret |= imx708_write(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_BLUE, (cctx->pending_blue_gain >> 8) & 0xFF);
        ret |= imx708_write(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_BLUE + 1, cctx->pending_blue_gain & 0xFF);
        cctx->imx708_para.blue_gain = cctx->pending_blue_gain;
        cctx->pending_blue_valid = false;
    }
}
imx708_group_hold(dev, false);

    if (ret == ESP_OK) {
        cam_imx708->imx708_para.exposure_val = exposure_val;
        cam_imx708->imx708_para.exposure_max = max_exposure;
        if (s_group_hold_depth == 0) {
            cam_imx708->last_param_update_tick = xTaskGetTickCount();
        }
        // Read-back and log actual exposure in microseconds for visibility
        uint8_t r_msb = 0, r_lsb = 0;
        if (imx708_read(dev->sccb_handle, IMX708_REG_EXPOSURE, &r_msb) == ESP_OK &&
            imx708_read(dev->sccb_handle, IMX708_REG_EXPOSURE + 1, &r_lsb) == ESP_OK) {
            uint32_t lines = ((uint32_t)r_msb << 8) | r_lsb;
            uint32_t us  = (uint32_t)((uint64_t)lines * 1000000ULL / (fps * (uint64_t)vts));
            ESP_LOGI(TAG, "Exposure applied: %lu lines (~%lu us), VTS=%lu", (unsigned long)lines, (unsigned long)us, (unsigned long)vts);
        } else {
            ESP_LOGD(TAG, "Exposure set successfully to %lu lines", exposure_val);
        }
    } else {
        ESP_LOGE(TAG, "Failed to write exposure registers");
    }
    
    return ret;
}

/**
 * @brief Set IMX708 exposure time in microseconds
 * @param dev Camera sensor device handle  
 * @param exposure_us Exposure time in microseconds
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t imx708_set_exposure_us(esp_cam_sensor_device_t *dev, uint32_t exposure_us)
{
    // Convert microseconds to scan lines based on current format timing
    const esp_cam_sensor_format_t *format = dev->cur_format;
    uint32_t fps = format->fps;
    uint32_t vts = 0;
    if (format->isp_info) {
        vts = format->isp_info->isp_v1_info.vts;
    } else {
        // Fallback: read current VTS from sensor
        if (imx708_get_frame_length_lines(dev, &vts) != ESP_OK || vts == 0) {
            ESP_LOGE(TAG, "Failed to get VTS for exposure conversion");
            return ESP_FAIL;
        }
    }

    // Clamp exposure by configured min/max (safety against over/under exposure)
    if (exposure_us < IMX708_CFG_MIN_EXPOSURE_US) {
        exposure_us = IMX708_CFG_MIN_EXPOSURE_US;
    }
    if (exposure_us > IMX708_CFG_MAX_EXPOSURE_US) {
        exposure_us = IMX708_CFG_MAX_EXPOSURE_US;
    }

    // Calculate exposure in lines: exposure_us * fps * vts / 1,000,000
    uint64_t exposure_lines_req = ((uint64_t)exposure_us * fps * vts) / 1000000ULL;
    uint32_t max_lines = vts - IMX708_EXPOSURE_MAX_OFFSET;
    uint32_t ratio_lines = (vts * IMX708_EXPOSURE_MAX_RATIO_PCT) / 100;
    if (ratio_lines < max_lines) max_lines = ratio_lines;

    // Default: no compensation (no-op)

    // If requested exposure exceeds per-frame budget, clamp only (let IPA adjust gain next frames)
    if (exposure_lines_req > max_lines) {
        ESP_LOGW(TAG, "Requested exposure %lu us (lines=%llu) exceeds budget (max_lines=%lu). Clamping only",
                 (unsigned long)exposure_us, (unsigned long long)exposure_lines_req, (unsigned long)max_lines);
        exposure_lines_req = max_lines;
        // No immediate gain compensation here to avoid brightness jumps; IPA will correct on next cycle
    }

    ESP_LOGD(TAG, "Converting %lu us to %llu lines (fps=%lu, vts=%lu)",
             (unsigned long)exposure_us, (unsigned long long)exposure_lines_req, (unsigned long)fps, (unsigned long)vts);

    return imx708_set_exposure_val(dev, (uint32_t)exposure_lines_req);
}

/*
 * IMX708 Gain Control Functions  
 * The IMX708 has separate analog and digital gain controls for fine-tuned exposure adjustment.
 */

/**
 * @brief Set IMX708 analog gain
 * @param dev Camera sensor device handle
 * @param gain Analog gain value (0x00-0xFF, 0x00=1x, 0xFF=~16x)
 * @return ESP_OK on success, ESP_FAIL on error  
 */
static esp_err_t imx708_set_analog_gain(esp_cam_sensor_device_t *dev, uint16_t gain)
{
    esp_err_t ret = ESP_OK;
    struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
    
    if (!cam_imx708) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clamp gain to valid range
    if (gain < IMX708_ANALOG_GAIN_MIN) {
        gain = IMX708_ANALOG_GAIN_MIN;
    } else if (gain > IMX708_ANALOG_GAIN_MAX) {
        ESP_LOGW(TAG, "Analog gain 0x%X above maximum 0x%X, clamping", gain, IMX708_ANALOG_GAIN_MAX);
        gain = IMX708_ANALOG_GAIN_MAX;
    }
    
    ESP_LOGD(TAG, "Setting analog gain to 0x%02X", gain);
    
    // Write analog gain register (16-bit: 0x0204 high, 0x0205 low) under group hold
    uint8_t gain_h = (gain >> 8) & 0xFF;
    uint8_t gain_l = (gain & 0xFF);
    imx708_group_hold(dev, true);
    ret  = imx708_write(dev->sccb_handle, IMX708_REG_ANALOG_GAIN,     gain_h);
    ret |= imx708_write(dev->sccb_handle, IMX708_REG_ANALOG_GAIN + 1, gain_l);
    imx708_group_hold(dev, false);
    
    if (ret == ESP_OK) {
        cam_imx708->imx708_para.analog_gain = gain;
        ESP_LOGD(TAG, "Analog gain set successfully to 0x%02X (H=0x%02X L=0x%02X)", gain, gain_h, gain_l);
    } else {
        ESP_LOGE(TAG, "Failed to write analog gain register");
    }
    
    return ret;
}

/**
 * @brief Set IMX708 digital gain
 * @param dev Camera sensor device handle
 * @param gain Digital gain value (0x0100-0x0FFF, 0x0100=1x, 0x0FFF=~16x)
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t imx708_set_digital_gain(esp_cam_sensor_device_t *dev, uint16_t gain)
{
    esp_err_t ret = ESP_OK;
    struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
    
    if (!cam_imx708) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clamp gain to valid range (and cap max to reduce flicker bursts)
    if (gain < IMX708_DIGITAL_GAIN_MIN) {
        ESP_LOGW(TAG, "Digital gain 0x%X below minimum 0x%X, clamping", gain, IMX708_DIGITAL_GAIN_MIN);
        gain = IMX708_DIGITAL_GAIN_MIN;
    }
    
    ESP_LOGD(TAG, "Setting digital gain to 0x%03X", gain);
    
    // Write digital gain register (16-bit big-endian format) under group hold
    imx708_group_hold(dev, true);
    ret = imx708_write(dev->sccb_handle, IMX708_REG_DIGITAL_GAIN, (gain >> 8) & 0xFF);
    ret |= imx708_write(dev->sccb_handle, IMX708_REG_DIGITAL_GAIN + 1, gain & 0xFF);
    imx708_group_hold(dev, false);
    
    if (ret == ESP_OK) {
        cam_imx708->imx708_para.digital_gain = gain;
        ESP_LOGD(TAG, "Digital gain set successfully to 0x%03X", gain);
    } else {
        ESP_LOGE(TAG, "Failed to write digital gain registers");
    }
    
    return ret;
}

/**
 * @brief Set IMX708 total gain (combines analog and digital)
 * @param dev Camera sensor device handle
 * @param total_gain Total gain value (100-25600, where 100=1.0x, 200=2.0x, etc.)
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t imx708_set_total_gain(esp_cam_sensor_device_t *dev, uint32_t total_gain)
{
    esp_err_t ret = ESP_OK;
    struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;

    if (!cam_imx708) {
        return ESP_ERR_INVALID_STATE;
    }

#if IMX708_FREEZE_TOTAL_GAIN
    // Total gain updates are frozen for diagnostics; ignore runtime changes
    return ESP_OK;
#endif

    // If a pending value exists and we're allowed to update now, replace request
    if (cam_imx708->pending_gain_valid) {
        total_gain = cam_imx708->pending_total_gain;
        cam_imx708->pending_gain_valid = false;
    }

    // Avoid frequent VTS writes here to reduce mid-stream disturbance; exposure path enforces when needed

    // Do not apply pending compensation here (disabled to avoid flicker)
    
    // Clamp total gain to valid range
    if (total_gain < IMX708_TOTAL_GAIN_MIN) {
        ESP_LOGW(TAG, "Total gain %lu below minimum %d, clamping", total_gain, IMX708_TOTAL_GAIN_MIN);
        total_gain = IMX708_TOTAL_GAIN_MIN;
    } else if (total_gain > IMX708_TOTAL_GAIN_MAX) {
        ESP_LOGW(TAG, "Total gain %lu above maximum %d, clamping", total_gain, IMX708_TOTAL_GAIN_MAX);
        total_gain = IMX708_TOTAL_GAIN_MAX;
    }

    // Ignore tiny total gain changes to avoid oscillation
    if ((total_gain > cam_imx708->imx708_para.gain_val ? (total_gain - cam_imx708->imx708_para.gain_val) : (cam_imx708->imx708_para.gain_val - total_gain)) < IMX708_MIN_TOTAL_GAIN_DELTA) {
        return ESP_OK;
    }

    // Rate-limit top-level updates
    if (s_group_hold_depth == 0 && cam_imx708->min_update_interval_ticks > 0) {
        TickType_t now = xTaskGetTickCount();
        // Warm-up guard after stream on
        if (now < cam_imx708->warmup_end_tick) {
            cam_imx708->pending_total_gain = total_gain;
            cam_imx708->pending_gain_valid = true;
            return ESP_OK;
        }
        if ((now - cam_imx708->last_param_update_tick) < cam_imx708->min_update_interval_ticks) {
            cam_imx708->pending_total_gain = total_gain;
            cam_imx708->pending_gain_valid = true;
            return ESP_OK; // defer but remember
        }
    }

    ESP_LOGD(TAG, "Setting total gain to %lu (%.2fx)", total_gain, total_gain / 100.0);
    
    // Strategy: Use analog gain first (better noise performance), then digital gain
    uint16_t analog_gain = 0;
    uint16_t digital_gain = IMX708_DIGITAL_GAIN_MIN; // Start at 1x digital
    
    #if IMX708_FREEZE_ANALOG_GAIN
        // Keep analog gain fixed at current (or default if unset), drive total gain via digital gain only
        analog_gain = cam_imx708->imx708_para.analog_gain ? (uint16_t)cam_imx708->imx708_para.analog_gain : (uint16_t)IMX708_ANA_GAIN_DEFAULT;
        // With analog fixed ~1.0x default, digital gain code is proportional to total_gain
        uint32_t dgain_code = (total_gain * 256) / 100; // 256 per 1x
        if (dgain_code < IMX708_DIGITAL_GAIN_MIN) dgain_code = IMX708_DIGITAL_GAIN_MIN;
        if (dgain_code > IMX708_DIGITAL_GAIN_MAX) dgain_code = IMX708_DIGITAL_GAIN_MAX;
        digital_gain = (uint16_t)dgain_code;
    #else
        if (total_gain <= IMX708_ANALOG_MAX_X100) {
            // Map [1.0x..AG_max] linearly into [ANA_MIN..ANA_MAX]
            uint32_t span = IMX708_ANALOG_GAIN_MAX - IMX708_ANALOG_GAIN_MIN;
            uint32_t num  = (total_gain > 100) ? (total_gain - 100) : 0;
            uint32_t den  = (IMX708_ANALOG_MAX_X100 > 100) ? (IMX708_ANALOG_MAX_X100 - 100) : 1;
            analog_gain = (uint16_t)(IMX708_ANALOG_GAIN_MIN + (span * num) / den);
            digital_gain = IMX708_DIGITAL_GAIN_MIN;
        } else {
            // Use max analog, put remainder into digital gain (8.8 fixed: 0x0100=1.0x)
            analog_gain = IMX708_ANALOG_GAIN_MAX;
            uint32_t remaining_x100 = (total_gain * 100) / IMX708_ANALOG_MAX_X100; // factor over 1x in x100
            if (remaining_x100 < 100) remaining_x100 = 100;
            uint32_t dgain_code = (remaining_x100 * 256) / 100; // 256 per 1x
            if (dgain_code < IMX708_DIGITAL_GAIN_MIN) dgain_code = IMX708_DIGITAL_GAIN_MIN;
            if (dgain_code > IMX708_DIGITAL_GAIN_MAX) dgain_code = IMX708_DIGITAL_GAIN_MAX;
            digital_gain = (uint16_t)dgain_code;
        }
    #endif
    
    ESP_LOGD(TAG, "Calculated gains: analog=0x%02X, digital=0x%03X", analog_gain, digital_gain);

    // If unchanged, skip writes
    if (cam_imx708->imx708_para.analog_gain == analog_gain && cam_imx708->imx708_para.digital_gain == digital_gain) {
        cam_imx708->imx708_para.gain_val = total_gain;
        return ESP_OK;
    }

    // Atomically program both gains (and any pending color balance) under group hold
    imx708_group_hold(dev, true);
    #if IMX708_FREEZE_ANALOG_GAIN
        // Do not touch analog gain when freezing
        ret  = imx708_set_digital_gain(dev, digital_gain);
    #else
        ret  = imx708_set_analog_gain(dev, analog_gain);
        ret |= imx708_set_digital_gain(dev, digital_gain);
    #endif
    // Apply pending color balance if any
    struct imx708_cam *cctx2 = (struct imx708_cam *)dev->priv;
    if (ret == ESP_OK && cctx2) {
        if (cctx2->pending_red_valid) {
            ret |= imx708_write(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_RED, (cctx2->pending_red_gain >> 8) & 0xFF);
            ret |= imx708_write(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_RED + 1, cctx2->pending_red_gain & 0xFF);
            cctx2->imx708_para.red_gain = cctx2->pending_red_gain;
            cctx2->pending_red_valid = false;
            
            
        }
        if (cctx2->pending_blue_valid) {
            ret |= imx708_write(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_BLUE, (cctx2->pending_blue_gain >> 8) & 0xFF);
            ret |= imx708_write(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_BLUE + 1, cctx2->pending_blue_gain & 0xFF);
            cctx2->imx708_para.blue_gain = cctx2->pending_blue_gain;
            cctx2->pending_blue_valid = false;
            
            
        }
    }
    imx708_group_hold(dev, false);
    
    if (ret == ESP_OK) {
        cam_imx708->imx708_para.gain_val = total_gain;
        ESP_LOGD(TAG, "Total gain set successfully to %lu", total_gain);
        if (s_group_hold_depth == 0) {
            cam_imx708->last_param_update_tick = xTaskGetTickCount();
        }
        
        
    }
    
    return ret;
}

/*
 * IMX708 Color Balance Control Functions
 * These functions provide real-time color balance adjustment by controlling
 * the Red and Blue channel gains to correct color temperature and tint.
 */

/**
 * @brief Set IMX708 Red channel gain for color balance
 * @param dev Camera sensor device handle
 * @param gain Red gain value (100-2000, where 400=4.0x gain)
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t imx708_set_red_gain(esp_cam_sensor_device_t *dev, uint16_t gain)
{
    esp_err_t ret = ESP_OK;
    struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
    
    if (!cam_imx708) {
        ESP_LOGE(TAG, "set_red_gain: Camera context is NULL");
        return ESP_FAIL;
    }
    
    // Clamp gain to valid range
    if (gain < IMX708_COLOR_BALANCE_RED_MIN) {
        gain = IMX708_COLOR_BALANCE_RED_MIN;
    } else if (gain > IMX708_COLOR_BALANCE_RED_MAX) {
        gain = IMX708_COLOR_BALANCE_RED_MAX;
    }
    
    ESP_LOGD(TAG, "imx708_set_red_gain: Request red gain %u (0x%03X)", gain, gain);
    // If we are in warm-up or cooldown, defer and coalesce under next group hold
    if (s_group_hold_depth == 0 && cam_imx708->min_update_interval_ticks > 0) {
        TickType_t now = xTaskGetTickCount();
        if (now < cam_imx708->warmup_end_tick ||
            (now - cam_imx708->last_param_update_tick) < cam_imx708->min_update_interval_ticks) {
            cam_imx708->pending_red_gain = gain;
            cam_imx708->pending_red_valid = true;
            return ESP_OK;
        }
    }
    // Otherwise write under group hold immediately
    imx708_group_hold(dev, true);
    ret = imx708_write(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_RED, (gain >> 8) & 0xFF);
    ret |= imx708_write(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_RED + 1, gain & 0xFF);
    imx708_group_hold(dev, false);
    
    if (ret == ESP_OK) {
        cam_imx708->imx708_para.red_gain = gain;
        ESP_LOGD(TAG, "Red gain set successfully to %u", gain);
        
        // Read back to verify the value was written correctly
        uint8_t reg_high, reg_low;
        esp_err_t read_ret = imx708_read(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_RED, &reg_high);
        read_ret |= imx708_read(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_RED + 1, &reg_low);
        
        if (read_ret == ESP_OK) {
            uint16_t readback = (reg_high << 8) | reg_low;
            ESP_LOGD(TAG, "Red gain readback: 0x%04X (expected: 0x%04X)", readback, gain);
            if (readback != gain) {
                ESP_LOGW(TAG, "Red gain readback mismatch! Expected 0x%04X, got 0x%04X", gain, readback);
            }
        } else {
            ESP_LOGW(TAG, "Failed to read back red gain register");
        }
    } else {
        ESP_LOGE(TAG, "Failed to set red gain to %u", gain);
    }
    
    return ret;
}

/**
 * @brief Set IMX708 Blue channel gain for color balance
 * @param dev Camera sensor device handle
 * @param gain Blue gain value (100-2000, where 280=2.8x gain)
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t imx708_set_blue_gain(esp_cam_sensor_device_t *dev, uint16_t gain)
{
    esp_err_t ret = ESP_OK;
    struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
    
    if (!cam_imx708) {
        ESP_LOGE(TAG, "set_blue_gain: Camera context is NULL");
        return ESP_FAIL;
    }
    
    // Clamp gain to valid range
    if (gain < IMX708_COLOR_BALANCE_BLUE_MIN) {
        gain = IMX708_COLOR_BALANCE_BLUE_MIN;
    } else if (gain > IMX708_COLOR_BALANCE_BLUE_MAX) {
        gain = IMX708_COLOR_BALANCE_BLUE_MAX;
    }
    
    ESP_LOGD(TAG, "imx708_set_blue_gain: Request blue gain %u (0x%03X)", gain, gain);
    // If we are in warm-up or cooldown, defer and coalesce under next group hold
    if (s_group_hold_depth == 0 && cam_imx708->min_update_interval_ticks > 0) {
        TickType_t now = xTaskGetTickCount();
        if (now < cam_imx708->warmup_end_tick ||
            (now - cam_imx708->last_param_update_tick) < cam_imx708->min_update_interval_ticks) {
            cam_imx708->pending_blue_gain = gain;
            cam_imx708->pending_blue_valid = true;
            return ESP_OK;
        }
    }
    // Otherwise write under group hold immediately
    imx708_group_hold(dev, true);
    ret = imx708_write(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_BLUE, (gain >> 8) & 0xFF);
    ret |= imx708_write(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_BLUE + 1, gain & 0xFF);
    imx708_group_hold(dev, false);
    
    if (ret == ESP_OK) {
        cam_imx708->imx708_para.blue_gain = gain;
        ESP_LOGD(TAG, "Blue gain set successfully to %u", gain);
        
        // Read back to verify the value was written correctly
        uint8_t reg_high, reg_low;
        esp_err_t read_ret = imx708_read(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_BLUE, &reg_high);
        read_ret |= imx708_read(dev->sccb_handle, IMX708_REG_COLOUR_BALANCE_BLUE + 1, &reg_low);
        
        if (read_ret == ESP_OK) {
            uint16_t readback = (reg_high << 8) | reg_low;
            ESP_LOGD(TAG, "Blue gain readback: 0x%04X (expected: 0x%04X)", readback, gain);
            if (readback != gain) {
                ESP_LOGW(TAG, "Blue gain readback mismatch! Expected 0x%04X, got 0x%04X", gain, readback);
            }
        } else {
            ESP_LOGW(TAG, "Failed to read back blue gain register");
        }
    } else {
        ESP_LOGE(TAG, "Failed to set blue gain to %u", gain);
    }
    
    return ret;
}

/*
 * IMX708 Image Quality Control Functions
 * These functions provide brightness, contrast, and saturation adjustments
 * by manipulating color balance and gain settings.
 */

/**
 * @brief Set IMX708 brightness adjustment
 * @param dev Camera sensor device handle
 * @param level Brightness level (-2 to +2, 0=default)
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t imx708_set_brightness(esp_cam_sensor_device_t *dev, int level)
{
    struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
    
    if (!cam_imx708) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clamp level to valid range
    if (level < IMX708_BRIGHTNESS_MIN) {
        ESP_LOGW(TAG, "Brightness %d below minimum %d, clamping", level, IMX708_BRIGHTNESS_MIN);
        level = IMX708_BRIGHTNESS_MIN;
    } else if (level > IMX708_BRIGHTNESS_MAX) {
        ESP_LOGW(TAG, "Brightness %d above maximum %d, clamping", level, IMX708_BRIGHTNESS_MAX);
        level = IMX708_BRIGHTNESS_MAX;
    }
    
    ESP_LOGD(TAG, "Setting brightness to level %d", level);
    
    // Delegate to ISP; avoid touching sensor gains here
    cam_imx708->imx708_para.brightness = level;
    ESP_LOGD(TAG, "Brightness (ISP) set to level %d (sensor gains unchanged)", level);
    return ESP_OK;
}

/**
 * @brief Set IMX708 contrast adjustment  
 * @param dev Camera sensor device handle
 * @param level Contrast level (-2 to +2, 0=default)
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t imx708_set_contrast(esp_cam_sensor_device_t *dev, int level)
{
    struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
    
    if (!cam_imx708) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clamp level to valid range
    if (level < IMX708_CONTRAST_MIN) {
        ESP_LOGW(TAG, "Contrast %d below minimum %d, clamping", level, IMX708_CONTRAST_MIN);
        level = IMX708_CONTRAST_MIN;
    } else if (level > IMX708_CONTRAST_MAX) {
        ESP_LOGW(TAG, "Contrast %d above maximum %d, clamping", level, IMX708_CONTRAST_MAX);
        level = IMX708_CONTRAST_MAX;
    }
    
    ESP_LOGD(TAG, "Setting contrast to level %d", level);
    
    cam_imx708->imx708_para.contrast = level;
    ESP_LOGD(TAG, "Contrast (ISP) set to level %d (sensor gains unchanged)", level);
    return ESP_OK;
}

/**
 * @brief Set IMX708 saturation adjustment
 * @param dev Camera sensor device handle  
 * @param level Saturation level (-2 to +2, 0=default)
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t imx708_set_saturation(esp_cam_sensor_device_t *dev, int level)
{
    struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
    
    if (!cam_imx708) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clamp level to valid range
    if (level < IMX708_SATURATION_MIN) {
        ESP_LOGW(TAG, "Saturation %d below minimum %d, clamping", level, IMX708_SATURATION_MIN);
        level = IMX708_SATURATION_MIN;
    } else if (level > IMX708_SATURATION_MAX) {
        ESP_LOGW(TAG, "Saturation %d above maximum %d, clamping", level, IMX708_SATURATION_MAX);
        level = IMX708_SATURATION_MAX;
    }
    
    ESP_LOGD(TAG, "Setting saturation to level %d", level);
    
    // Delegate to ISP; do not alter color balance here
    cam_imx708->imx708_para.saturation = level;
    ESP_LOGD(TAG, "Saturation (ISP) set to level %d (sensor gains unchanged)", level);
    return ESP_OK;
}

static esp_err_t imx708_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
{
    esp_err_t ret = ESP_OK;

    switch (qdesc->id) {
    // Orientation Controls
    case ESP_CAM_SENSOR_VFLIP:
    case ESP_CAM_SENSOR_HMIRROR:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = 0;
        qdesc->number.maximum = 1;
        qdesc->number.step = 1;
        qdesc->default_value = 0;
        break;
        
    // Exposure Controls
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX708_EXPOSURE_MIN;
        {
            uint32_t vts = 0;
            if (dev->cur_format && dev->cur_format->isp_info) {
                vts = dev->cur_format->isp_info->isp_v1_info.vts;
            } else {
                (void)imx708_get_frame_length_lines(dev, &vts);
            }
            if (vts > IMX708_EXPOSURE_MAX_OFFSET) {
                qdesc->number.maximum = vts - IMX708_EXPOSURE_MAX_OFFSET;
            } else {
                qdesc->number.maximum = IMX708_EXPOSURE_MIN;
            }
        }
        qdesc->number.step = IMX708_EXPOSURE_STEP;
        qdesc->default_value = IMX708_EXPOSURE_DEFAULT;
        break;
        
    case ESP_CAM_SENSOR_EXPOSURE_US:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        // Convert exposure limits from lines to microseconds
        {
            uint32_t fps = dev->cur_format->fps;
            uint32_t vts = 0;
            if (dev->cur_format && dev->cur_format->isp_info) {
                vts = dev->cur_format->isp_info->isp_v1_info.vts;
            } else {
                (void)imx708_get_frame_length_lines(dev, &vts);
            }
            if (vts == 0) vts = 1; // avoid div by 0
            uint32_t min_us_vts = (IMX708_EXPOSURE_MIN * 1000000UL) / (fps * vts);
            uint32_t max_us_vts = ((vts - IMX708_EXPOSURE_MAX_OFFSET) * 1000000UL) / (fps * vts);
            // Intersect with configured clamps
            uint32_t min_us = (IMX708_CFG_MIN_EXPOSURE_US > min_us_vts) ? IMX708_CFG_MIN_EXPOSURE_US : min_us_vts;
            uint32_t max_us = (IMX708_CFG_MAX_EXPOSURE_US < max_us_vts) ? IMX708_CFG_MAX_EXPOSURE_US : max_us_vts;
            if (max_us < min_us) {
                // Fallback to vts-derived if misconfigured
                min_us = min_us_vts;
                max_us = max_us_vts;
            }
            qdesc->number.minimum = min_us;
            qdesc->number.maximum = max_us;
                        // Use step of 65 to match V4L2 validation requirements
            qdesc->number.step = 65;
            // Adjust default to be a multiple of 65
            uint32_t calculated_default = (IMX708_EXPOSURE_DEFAULT * 1000000UL) / (fps * vts);
            if (calculated_default < qdesc->number.minimum) calculated_default = qdesc->number.minimum;
            if (calculated_default > qdesc->number.maximum) calculated_default = qdesc->number.maximum;
            qdesc->default_value = ((calculated_default / 65) * 65); // Round to nearest multiple of 65
        }
        break;
        
    // Gain Controls
    case ESP_CAM_SENSOR_GAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX708_TOTAL_GAIN_MIN;
        qdesc->number.maximum = IMX708_TOTAL_GAIN_MAX;
        qdesc->number.step = 1;
        qdesc->default_value = IMX708_TOTAL_GAIN_MIN;
        break;
        
    case ESP_CAM_SENSOR_ANGAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX708_ANA_GAIN_MIN;
        qdesc->number.maximum = IMX708_ANA_GAIN_MAX;
        qdesc->number.step = IMX708_ANA_GAIN_STEP;
        qdesc->default_value = IMX708_ANA_GAIN_DEFAULT;
        break;
        
    case ESP_CAM_SENSOR_DGAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX708_DGTL_GAIN_MIN;
        qdesc->number.maximum = IMX708_DGTL_GAIN_MAX;
        qdesc->number.step = IMX708_DGTL_GAIN_STEP;
        qdesc->default_value = IMX708_DGTL_GAIN_DEFAULT;
        break;
        
    // Image Quality Controls
    case ESP_CAM_SENSOR_BRIGHTNESS:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX708_BRIGHTNESS_MIN;
        qdesc->number.maximum = IMX708_BRIGHTNESS_MAX;
        qdesc->number.step = 1;
        qdesc->default_value = 0;
        break;
        
    case ESP_CAM_SENSOR_CONTRAST:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX708_CONTRAST_MIN;
        qdesc->number.maximum = IMX708_CONTRAST_MAX;
        qdesc->number.step = 1;
        qdesc->default_value = 0;
        break;
        
    case ESP_CAM_SENSOR_SATURATION:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX708_SATURATION_MIN;
        qdesc->number.maximum = IMX708_SATURATION_MAX;
        qdesc->number.step = 1;
        qdesc->default_value = 0;
        break;

    case ESP_CAM_SENSOR_DATA_SEQ:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = ESP_CAM_SENSOR_DATA_SEQ_NONE;
        qdesc->number.maximum = ESP_CAM_SENSOR_DATA_SEQ_NONE;
        qdesc->number.step = 1;
        qdesc->default_value = ESP_CAM_SENSOR_DATA_SEQ_NONE;
        qdesc->flags = ESP_CAM_SENSOR_PARAM_FLAG_READ_ONLY;
        break;

    // Color Balance Controls
    case IMX708_COLOR_BALANCE_RED:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX708_COLOR_BALANCE_RED_MIN;
        qdesc->number.maximum = IMX708_COLOR_BALANCE_RED_MAX;
        qdesc->number.step = 1;
        qdesc->default_value = IMX708_COLOR_BALANCE_RED_DEFAULT;
        break;
        
    case IMX708_COLOR_BALANCE_BLUE:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = IMX708_COLOR_BALANCE_BLUE_MIN;
        qdesc->number.maximum = IMX708_COLOR_BALANCE_BLUE_MAX;
        qdesc->number.step = 1;
        qdesc->default_value = IMX708_COLOR_BALANCE_BLUE_DEFAULT;
        break;
        
    // Group Control for Exposure + Gain
    case ESP_CAM_SENSOR_GROUP_EXP_GAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_U8;
        qdesc->u8.size = sizeof(esp_cam_sensor_gh_exp_gain_t);
        break;
        
    // 3A Lock Control
    case ESP_CAM_SENSOR_3A_LOCK:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = 0;
        qdesc->number.maximum = 7; // AWB (1) + AE (2) + AF (4) combinations
        qdesc->number.step = 1;
        qdesc->default_value = 0;
        break;
        
    // AE Level Control  
    case ESP_CAM_SENSOR_AE_LEVEL:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = -2;
        qdesc->number.maximum = 2;
        qdesc->number.step = 1;
        qdesc->default_value = 0;
        break;
        
    // Sharpness Control (0x30018 equivalent)
    case ESP_CAM_SENSOR_SHARPNESS:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = -2;
        qdesc->number.maximum = 2;
        qdesc->number.step = 1;
        qdesc->default_value = 0;
        break;
        
    default:
        ESP_LOGE(TAG, "query_para_desc: Parameter ID 0x%" PRIx32 " is not supported", qdesc->id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "query_para_desc: ID 0x%" PRIx32 " type=%lu", qdesc->id, qdesc->type);
    }
    
    return ret;
}

static esp_err_t imx708_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;
    struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
    
    if (!cam_imx708) {
        ESP_LOGE(TAG, "get_para_value: Camera context not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGD(TAG, "get_para_value: Getting parameter ID 0x%" PRIx32 ", size=%zu", id, size);

    switch (id) {
    // Orientation Controls
    case ESP_CAM_SENSOR_VFLIP:
        ESP_RETURN_ON_FALSE(size == sizeof(int32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for VFLIP");
        *(int32_t *)arg = cam_imx708->imx708_para.vflip_en;
        break;
        
    case ESP_CAM_SENSOR_HMIRROR:
        ESP_RETURN_ON_FALSE(size == sizeof(int32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for HMIRROR");
        *(int32_t *)arg = cam_imx708->imx708_para.hmirror_en;
        break;
        
    // Exposure Controls
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        ESP_RETURN_ON_FALSE(size == sizeof(uint32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for EXPOSURE_VAL");
        *(uint32_t *)arg = cam_imx708->imx708_para.exposure_val;
        break;
        
    case ESP_CAM_SENSOR_EXPOSURE_US:
        ESP_RETURN_ON_FALSE(size == sizeof(uint32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for EXPOSURE_US");
        {
            // Convert current exposure from lines to microseconds
            uint32_t fps = dev->cur_format->fps;
            uint32_t vts = 0;
            if (dev->cur_format && dev->cur_format->isp_info) {
                vts = dev->cur_format->isp_info->isp_v1_info.vts;
            } else {
                (void)imx708_get_frame_length_lines(dev, &vts);
            }
            if (vts == 0) vts = 1;
            *(uint32_t *)arg = (cam_imx708->imx708_para.exposure_val * 1000000UL) / (fps * vts);
        }
        break;
        
    // Gain Controls
    case ESP_CAM_SENSOR_GAIN:
        ESP_RETURN_ON_FALSE(size == sizeof(uint32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for GAIN");
        *(uint32_t *)arg = cam_imx708->imx708_para.gain_val;
        break;
        
    case ESP_CAM_SENSOR_ANGAIN:
        ESP_RETURN_ON_FALSE(size == sizeof(uint32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for ANGAIN");
        *(uint32_t *)arg = cam_imx708->imx708_para.analog_gain;
        break;
        
    case ESP_CAM_SENSOR_DGAIN:
        ESP_RETURN_ON_FALSE(size == sizeof(uint32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for DGAIN");
        *(uint32_t *)arg = cam_imx708->imx708_para.digital_gain;
        break;

    case ESP_CAM_SENSOR_DATA_SEQ:
        ESP_RETURN_ON_FALSE(size == sizeof(uint32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for DATA_SEQ");
        *(uint32_t *)arg = ESP_CAM_SENSOR_DATA_SEQ_NONE;
        break;

    // Image Quality Controls
    case ESP_CAM_SENSOR_BRIGHTNESS:
        ESP_RETURN_ON_FALSE(size == sizeof(int32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for BRIGHTNESS");
        *(int32_t *)arg = cam_imx708->imx708_para.brightness;
        break;
        
    case ESP_CAM_SENSOR_CONTRAST:
        ESP_RETURN_ON_FALSE(size == sizeof(int32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for CONTRAST");
        *(int32_t *)arg = cam_imx708->imx708_para.contrast;
        break;
        
    case ESP_CAM_SENSOR_SATURATION:
        ESP_RETURN_ON_FALSE(size == sizeof(int32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for SATURATION");
        *(int32_t *)arg = cam_imx708->imx708_para.saturation;
        break;
        
    // Color Balance Controls
    case IMX708_COLOR_BALANCE_RED:
        ESP_RETURN_ON_FALSE(size == sizeof(int32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for COLOR_BALANCE_RED");
        *(int32_t *)arg = cam_imx708->imx708_para.red_gain;
        break;
        
    case IMX708_COLOR_BALANCE_BLUE:
        ESP_RETURN_ON_FALSE(size == sizeof(int32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for COLOR_BALANCE_BLUE");
        *(int32_t *)arg = cam_imx708->imx708_para.blue_gain;
        break;
        
    // Auto Control Status  
    case ESP_CAM_SENSOR_AE_CONTROL:
        ESP_RETURN_ON_FALSE(size == sizeof(int32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for AE_CONTROL");
        *(int32_t *)arg = cam_imx708->imx708_para.ae_enable;
        break;
        
    case ESP_CAM_SENSOR_AWB:
        ESP_RETURN_ON_FALSE(size == sizeof(int32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for AWB");
        *(int32_t *)arg = cam_imx708->imx708_para.awb_enable;
        break;
        
    case ESP_CAM_SENSOR_AGC:
        ESP_RETURN_ON_FALSE(size == sizeof(int32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for AGC");
        *(int32_t *)arg = cam_imx708->imx708_para.agc_enable;
        break;
        
    // 3A Lock Control
    case ESP_CAM_SENSOR_3A_LOCK:
        ESP_RETURN_ON_FALSE(size == sizeof(int32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for 3A_LOCK");
        *(int32_t *)arg = 0; // Default: no locks active
        break;
        
    // AE Level Control
    case ESP_CAM_SENSOR_AE_LEVEL:
        ESP_RETURN_ON_FALSE(size == sizeof(int32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for AE_LEVEL");
        *(int32_t *)arg = 0; // Default: normal AE level
        break;
        
    // Sharpness Control
    case ESP_CAM_SENSOR_SHARPNESS:
        ESP_RETURN_ON_FALSE(size == sizeof(int32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for SHARPNESS");
        *(int32_t *)arg = 0; // Default: normal sharpness
        break;
        
    default:
        ESP_LOGE(TAG, "get_para_value: Parameter ID 0x%" PRIx32 " is not supported", id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "get_para_value: ID 0x%" PRIx32 " retrieved successfully", id);
    }

    return ret;
}

static esp_err_t imx708_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;
    
    ESP_LOGD(TAG, "set_para_value: Setting parameter ID 0x%" PRIx32 ", size=%zu", id, size);

    switch (id) {
    // Orientation Controls
    case ESP_CAM_SENSOR_VFLIP:
        ESP_RETURN_ON_FALSE(size == sizeof(int), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for VFLIP");
        {
            int *value = (int *)arg;
            ret = imx708_set_vflip(dev, *value);
            ESP_LOGD(TAG, "set_para_value: VFLIP set to %d", *value);
        }
        break;
        
    case ESP_CAM_SENSOR_HMIRROR:
        ESP_RETURN_ON_FALSE(size == sizeof(int), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for HMIRROR");
        {
            int *value = (int *)arg;
            ret = imx708_set_hmirror(dev, *value);
            ESP_LOGD(TAG, "set_para_value: HMIRROR set to %d", *value);
        }
        break;
        
    // Exposure Controls
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        ESP_RETURN_ON_FALSE(size == sizeof(uint32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for EXPOSURE_VAL");
        {
            uint32_t *value = (uint32_t *)arg;
            ret = imx708_set_exposure_val(dev, *value);
            ESP_LOGD(TAG, "set_para_value: EXPOSURE_VAL set to %lu lines", *value);
        }
        break;
        
    case ESP_CAM_SENSOR_EXPOSURE_US:
        ESP_RETURN_ON_FALSE(size == sizeof(uint32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for EXPOSURE_US");
        {
            uint32_t *value = (uint32_t *)arg;
            ret = imx708_set_exposure_us(dev, *value);
            ESP_LOGD(TAG, "set_para_value: EXPOSURE_US set to %lu microseconds", *value);
        }
        break;
        
    // Gain Controls
    case ESP_CAM_SENSOR_GAIN:
        ESP_RETURN_ON_FALSE(size == sizeof(uint32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for GAIN");
        {
            uint32_t *value = (uint32_t *)arg;
            ret = imx708_set_total_gain(dev, *value);
            ESP_LOGD(TAG, "set_para_value: Total GAIN set to %lu", *value);
        }
        break;
        
    case ESP_CAM_SENSOR_ANGAIN:
        ESP_RETURN_ON_FALSE(size == sizeof(uint16_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for ANGAIN");
        {
            uint16_t *value = (uint16_t *)arg;
            ret = imx708_set_analog_gain(dev, *value);
            ESP_LOGD(TAG, "set_para_value: Analog GAIN set to 0x%02X", *value);
        }
        break;
        
    case ESP_CAM_SENSOR_DGAIN:
        ESP_RETURN_ON_FALSE(size == sizeof(uint16_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for DGAIN");
        {
            uint16_t *value = (uint16_t *)arg;
            ret = imx708_set_digital_gain(dev, *value);
            ESP_LOGD(TAG, "set_para_value: Digital GAIN set to 0x%03X", *value);
        }
        break;

    case ESP_CAM_SENSOR_DATA_SEQ:
        ESP_RETURN_ON_FALSE(size == sizeof(uint32_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for DATA_SEQ");
        if (*(uint32_t *)arg != ESP_CAM_SENSOR_DATA_SEQ_NONE) {
            ret = ESP_ERR_INVALID_ARG;
        }
        break;

    // Image Quality Controls
    case ESP_CAM_SENSOR_BRIGHTNESS:
        ESP_RETURN_ON_FALSE(size == sizeof(int), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for BRIGHTNESS");
        {
            int *value = (int *)arg;
            ret = imx708_set_brightness(dev, *value);
            ESP_LOGD(TAG, "set_para_value: BRIGHTNESS set to level %d", *value);
        }
        break;
        
    case ESP_CAM_SENSOR_CONTRAST:
        ESP_RETURN_ON_FALSE(size == sizeof(int), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for CONTRAST");
        {
            int *value = (int *)arg;
            ret = imx708_set_contrast(dev, *value);
            ESP_LOGD(TAG, "set_para_value: CONTRAST set to level %d", *value);
        }
        break;
        
    case ESP_CAM_SENSOR_SATURATION:
        ESP_RETURN_ON_FALSE(size == sizeof(int), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for SATURATION");
        {
            int *value = (int *)arg;
            ret = imx708_set_saturation(dev, *value);
            ESP_LOGD(TAG, "set_para_value: SATURATION set to level %d", *value);
        }
        break;
        
    // Color Balance Controls
    case IMX708_COLOR_BALANCE_RED:
        ESP_RETURN_ON_FALSE(size == sizeof(int), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for COLOR_BALANCE_RED");
        {
            int *value = (int *)arg;
            ESP_LOGI(TAG, "set_para_value: COLOR_BALANCE_RED called with value %d", *value);
            ret = imx708_set_red_gain(dev, *value);
            ESP_LOGI(TAG, "set_para_value: COLOR_BALANCE_RED set to %d, result=%d", *value, ret);
        }
        break;
        
    case IMX708_COLOR_BALANCE_BLUE:
        ESP_RETURN_ON_FALSE(size == sizeof(int), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for COLOR_BALANCE_BLUE");
        {
            int *value = (int *)arg;
            ESP_LOGI(TAG, "set_para_value: COLOR_BALANCE_BLUE called with value %d", *value);
            ret = imx708_set_blue_gain(dev, *value);
            ESP_LOGI(TAG, "set_para_value: COLOR_BALANCE_BLUE set to %d, result=%d", *value, ret);
        }
        break;
        
    // Group Control for synchronized Exposure + Gain updates
    case ESP_CAM_SENSOR_GROUP_EXP_GAIN:
        ESP_RETURN_ON_FALSE(size == sizeof(esp_cam_sensor_gh_exp_gain_t), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for GROUP_EXP_GAIN");
        {
            esp_cam_sensor_gh_exp_gain_t *value = (esp_cam_sensor_gh_exp_gain_t *)arg;
            ESP_LOGD(TAG, "set_para_value: GROUP_EXP_GAIN exposure_us=%lu, gain_index=%lu", 
                     value->exposure_us, value->gain_index);

            // Apply exposure and gain atomically via group hold to ensure clean transitions
            imx708_group_hold(dev, true);
            ret = imx708_set_exposure_us(dev, value->exposure_us);
            ret |= imx708_set_total_gain(dev, value->gain_index);
            imx708_group_hold(dev, false);
        }
        break;
        
    // Auto Control Enable/Disable (for future implementation)
    case ESP_CAM_SENSOR_AE_CONTROL:
        ESP_RETURN_ON_FALSE(size == sizeof(int), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for AE_CONTROL");
        {
            int *value = (int *)arg;
            struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
            if (cam_imx708) {
                cam_imx708->imx708_para.ae_enable = *value ? 1 : 0;
                ESP_LOGD(TAG, "set_para_value: AE_CONTROL set to %d", *value);
                // Note: Actual AE algorithm implementation would go here
            } else {
                ret = ESP_ERR_INVALID_STATE;
            }
        }
        break;
        
    case ESP_CAM_SENSOR_AWB:
        ESP_RETURN_ON_FALSE(size == sizeof(int), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for AWB");
        {
            int *value = (int *)arg;
            struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
            if (cam_imx708) {
                cam_imx708->imx708_para.awb_enable = *value ? 1 : 0;
                ESP_LOGD(TAG, "set_para_value: AWB set to %d", *value);
                // Note: Actual AWB algorithm implementation would go here
            } else {
                ret = ESP_ERR_INVALID_STATE;
            }
        }
        break;
        
    case ESP_CAM_SENSOR_AGC:
        ESP_RETURN_ON_FALSE(size == sizeof(int), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for AGC");
        {
            int *value = (int *)arg;
            struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
            if (cam_imx708) {
                cam_imx708->imx708_para.agc_enable = *value ? 1 : 0;
                ESP_LOGD(TAG, "set_para_value: AGC set to %d", *value);
                // Note: Actual AGC algorithm implementation would go here
            } else {
                ret = ESP_ERR_INVALID_STATE;
            }
        }
        break;
        
    // 3A Lock Control
    case ESP_CAM_SENSOR_3A_LOCK:
        ESP_RETURN_ON_FALSE(size == sizeof(int), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for 3A_LOCK");
        {
            int *value = (int *)arg;
            struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
            if (cam_imx708) {
                int mask = *value; // AWB(1) + AE(2) + AF(4)
                bool ae_locked = (mask & 0x2) != 0;
                cam_imx708->imx708_para.fixed_exposure_en = ae_locked;
                ESP_LOGI(TAG, "3A_LOCK=%d (AE %s)", *value, ae_locked ? "locked" : "unlocked");
                ret = ESP_OK;
            } else {
                ret = ESP_ERR_INVALID_STATE;
            }
        }
        break;
        
    // AE Level Control
    case ESP_CAM_SENSOR_AE_LEVEL:
        ESP_RETURN_ON_FALSE(size == sizeof(int), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for AE_LEVEL");
        {
            int *value = (int *)arg;
            ESP_LOGD(TAG, "set_para_value: AE_LEVEL set to %d (Note: not fully implemented)", *value);
            // For now, just log - full implementation would adjust AE target
            ret = ESP_OK;
        }
        break;
        
    // Sharpness Control
    case ESP_CAM_SENSOR_SHARPNESS:
        ESP_RETURN_ON_FALSE(size == sizeof(int), ESP_ERR_INVALID_SIZE, TAG, "Invalid size for SHARPNESS");
        {
            int *value = (int *)arg;
            ESP_LOGD(TAG, "set_para_value: SHARPNESS set to %d (Note: processed by ISP)", *value);
            // Sharpness is typically handled by the ISP, not the sensor
            ret = ESP_OK;
        }
        break;
        
    default:
        ESP_LOGE(TAG, "set_para_value: Parameter ID 0x%" PRIx32 " is not supported", id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "set_para_value: ID 0x%" PRIx32 " set successfully", id);
    } else {
        ESP_LOGE(TAG, "set_para_value: Failed to set ID 0x%" PRIx32 ", error: %s", id, esp_err_to_name(ret));
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
    struct imx708_cam *cam_imx708 = (struct imx708_cam *)dev->priv;
    
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

        // AWB/ACC left to IPA/ISP pipeline (no sensor-side AWB/ACC initialization)

        // Start from neutral gains (1x) and let IPA control exposure within FPS guard
        // Force gains to 1.0x explicitly
        (void)imx708_set_analog_gain(dev, IMX708_ANA_GAIN_DEFAULT);   // 1x analog per datasheet
        (void)imx708_set_digital_gain(dev, 0x0100); // 1x digital
        // And keep total gain at 1.0x
        (void)imx708_set_total_gain(dev, IMX708_TOTAL_GAIN_MIN);
        // Gains already forced to 1x above; keep para in sync
        if (cam_imx708) {
            cam_imx708->imx708_para.gain_val = IMX708_TOTAL_GAIN_MIN;
            cam_imx708->imx708_para.analog_gain = IMX708_ANA_GAIN_DEFAULT;
            cam_imx708->imx708_para.digital_gain = 0x0100;
            cam_imx708->imx708_para.fixed_exposure_en = false; // allow IPA to control exposure
            // Initialize rate limiter to roughly one update per 2 frames (more conservative)
            uint32_t fps_local = (format && format->fps) ? format->fps : 15;
            uint32_t frame_time_ms = (fps_local > 0) ? (1000 / fps_local) : 66;
            cam_imx708->min_update_interval_ticks = pdMS_TO_TICKS(frame_time_ms * 2);
            cam_imx708->last_param_update_tick = xTaskGetTickCount();
            cam_imx708->pending_exposure_valid = false;
            cam_imx708->pending_gain_valid = false;
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
    case ESP_CAM_SENSOR_IOC_S_BRIGHTNESS:
        ret = imx708_set_brightness(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_S_CONTRAST:
        ret = imx708_set_contrast(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_S_SATURATION:
        ret = imx708_set_saturation(dev, *(int *)arg);
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
    ESP_LOGD(TAG, "imx708_delete: Deleting device (%p)", dev);
    
    if (dev) {
        // Free camera context if allocated
        if (dev->priv) {
            ESP_LOGD(TAG, "imx708_delete: Freeing camera context");
            free(dev->priv);
            dev->priv = NULL;
        }
        
        ESP_LOGD(TAG, "imx708_delete: Freeing device structure");
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
        ESP_LOGE(TAG, "No memory for camera device");
        return NULL;
    }

    // Allocate and initialize camera parameter context
    struct imx708_cam *cam_imx708 = calloc(1, sizeof(struct imx708_cam));
    if (cam_imx708 == NULL) {
        ESP_LOGE(TAG, "No memory for camera context");
        free(dev);
        return NULL;
    }

    // Initialize default parameter values
    cam_imx708->imx708_para.exposure_val = IMX708_EXPOSURE_DEFAULT;
    cam_imx708->imx708_para.exposure_max = 0; // Will be set based on format
    cam_imx708->imx708_para.gain_val = IMX708_TOTAL_GAIN_MIN;
    cam_imx708->imx708_para.analog_gain = IMX708_ANA_GAIN_DEFAULT;
    cam_imx708->imx708_para.digital_gain = IMX708_DGTL_GAIN_DEFAULT;
    cam_imx708->imx708_para.fixed_exposure_en = false;
    
    // Image quality defaults (neutral settings)
    cam_imx708->imx708_para.brightness = 0;
    cam_imx708->imx708_para.contrast = 0;
    cam_imx708->imx708_para.saturation = 0;
    
    // Color balance defaults
    cam_imx708->imx708_para.red_gain = IMX708_COLOR_BALANCE_RED_DEFAULT;
    cam_imx708->imx708_para.blue_gain = IMX708_COLOR_BALANCE_BLUE_DEFAULT;
    
    // Orientation defaults
    cam_imx708->imx708_para.vflip_en = 0;
    cam_imx708->imx708_para.hmirror_en = 0;
    
    // Auto control defaults (disabled for manual control)
    cam_imx708->imx708_para.ae_enable = 0;
    cam_imx708->imx708_para.awb_enable = 0;
    cam_imx708->imx708_para.agc_enable = 0;

    ESP_LOGD(TAG, "IMX708 detect: Camera context initialized with default parameters");

    dev->name = IMX708_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &imx708_ops;
    dev->priv = cam_imx708;  // Store camera context in device private data

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
    if (dev && dev->priv) {
        free(dev->priv);  // Free camera context
    }
    if (dev) {
        free(dev);
    }
    return NULL;
}

// Global reference to the IMX708 device for hardware register access
static esp_cam_sensor_device_t *g_imx708_device = NULL;
static SemaphoreHandle_t s_imx708_sccb_mutex = NULL;

static inline bool imx708_sccb_lock(TickType_t ticks)
{
    if (s_imx708_sccb_mutex == NULL) {
        // Lazy create
        s_imx708_sccb_mutex = xSemaphoreCreateMutex();
        if (s_imx708_sccb_mutex == NULL) {
            return false;
        }
    }
    return xSemaphoreTake(s_imx708_sccb_mutex, ticks) == pdTRUE;
}

static inline void imx708_sccb_unlock(void)
{
    if (s_imx708_sccb_mutex) {
        xSemaphoreGive(s_imx708_sccb_mutex);
    }
}

/**
 * @brief Set global IMX708 device reference for hardware access
 * This function should be called after successful device detection
 */
void imx708_set_global_device(esp_cam_sensor_device_t *device)
{
    g_imx708_device = device;
    ESP_LOGI(TAG, "IMX708 global device reference set for hardware access");
    if (s_imx708_sccb_mutex == NULL) {
        s_imx708_sccb_mutex = xSemaphoreCreateMutex();
        if (!s_imx708_sccb_mutex) {
            ESP_LOGW(TAG, "Failed to create IMX708 SCCB mutex");
        }
    }
}

/**
 * @brief Public function to write IMX708 hardware registers
 * Provides shell command access to direct register control
 */
esp_err_t imx708_hw_write_register(uint16_t reg, uint8_t value)
{
    if (!g_imx708_device || !g_imx708_device->sccb_handle) {
        ESP_LOGE(TAG, "IMX708 device not initialized for hardware access");
        return ESP_ERR_INVALID_STATE;
    }

    // Try to acquire SCCB quickly; avoid blocking streaming for long
    if (!imx708_sccb_lock(pdMS_TO_TICKS(10))) {
        ESP_LOGW(TAG, "IMX708 SCCB busy, write 0x%04X skipped", reg);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = imx708_write(g_imx708_device->sccb_handle, reg, value);
    imx708_sccb_unlock();
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "IMX708 HW Write: 0x%04X = 0x%02X", reg, value);
    } else {
        ESP_LOGE(TAG, "IMX708 HW Write failed: 0x%04X = 0x%02X, error: %s", reg, value, esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Public function to read IMX708 hardware registers
 * Provides shell command access to direct register monitoring
 */
esp_err_t imx708_hw_read_register(uint16_t reg, uint8_t *value)
{
    if (!g_imx708_device || !g_imx708_device->sccb_handle) {
        ESP_LOGE(TAG, "IMX708 device not initialized for hardware access");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!value) {
        ESP_LOGE(TAG, "IMX708 HW Read: NULL value pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (!imx708_sccb_lock(pdMS_TO_TICKS(10))) {
        ESP_LOGW(TAG, "IMX708 SCCB busy, read 0x%04X skipped", reg);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = imx708_read(g_imx708_device->sccb_handle, reg, value);
    imx708_sccb_unlock();
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "IMX708 HW Read: 0x%04X = 0x%02X", reg, *value);
    } else {
        ESP_LOGE(TAG, "IMX708 HW Read failed: 0x%04X, error: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
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
        // Set global device reference for hardware register access
        imx708_set_global_device(result);
        ESP_LOGI(TAG, "IMX708 auto-detect: Global device reference configured for hardware access");
    } else {
        ESP_LOGE(TAG, "IMX708 auto-detect: Detection failed!");
    }
    return result;
}
#endif









