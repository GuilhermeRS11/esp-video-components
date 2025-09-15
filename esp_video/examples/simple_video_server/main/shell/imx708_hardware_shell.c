/*
 * IMX708 Hardware Control Shell Commands
 * Focus on direct hardware register control only
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imx708.h"

// IMX708 Register definitions for hardware control
#define IMX708_REG_AE_ENABLE    0x3100  // Auto Exposure Enable/Disable
#define IMX708_REG_AGC_ENABLE   0x3200  // Auto Gain Control Enable/Disable

static const char* TAG = "imx708_hw_shell";

/**
 * @brief Direct Sensor Exposure Control
 */
static int cmd_sensor_exposure_direct(int argc, char **argv)
{
    if (argc == 1) {
        printf("Direct Sensor Exposure Control\n");
        printf("==============================\n");
        printf("Usage: sensor_exposure_direct <exposure_lines>\n");
        printf("       sensor_exposure_direct current\n");
        printf("       sensor_exposure_direct auto <enable|disable>\n\n");
        printf("Values: 1-8000 lines (typical)\n");
        printf("Lower = darker, Higher = brighter\n");
        printf("This directly writes to registers 0x0202/0x0203\n");
        return 0;
    }
    
    const char *param = argv[1];
    
    if (strcmp(param, "current") == 0) {
        uint8_t exp_h, exp_l, ae_enable;
        imx708_hw_read_register(0x0202, &exp_h);
        imx708_hw_read_register(0x0203, &exp_l);
        imx708_hw_read_register(IMX708_REG_AE_ENABLE, &ae_enable);
        
        uint16_t exposure = (exp_h << 8) | exp_l;
        
        printf("Current Exposure Settings:\n");
        printf("  Exposure: %d lines (0x%04X)\n", exposure, exposure);
        printf("  Auto Exposure: %s (register 0x3100 = 0x%02X)\n", 
               ae_enable ? "ENABLED" : "DISABLED", ae_enable);
        
        if (exposure < 500) {
            printf("  → Very fast exposure (bright scenes)\n");
        } else if (exposure < 2000) {
            printf("  → Normal exposure\n");
        } else {
            printf("  → Long exposure (low light)\n");
        }
        
        return 0;
    }
    
    if (strcmp(param, "auto") == 0) {
        if (argc != 3) {
            printf("Usage: sensor_exposure_direct auto <enable|disable>\n");
            return 1;
        }
        
        bool enable = strcmp(argv[2], "enable") == 0;
        uint8_t ae_val = enable ? 0x01 : 0x00;
        
        ESP_LOGI(TAG, "%s sensor auto exposure", enable ? "Enabling" : "Disabling");
        esp_err_t ret = imx708_hw_write_register(IMX708_REG_AE_ENABLE, ae_val);
        
        if (ret == ESP_OK) {
            printf("Sensor Auto Exposure %s\n", enable ? "ENABLED" : "DISABLED");
            printf("Register 0x3100 = 0x%02X\n", ae_val);
        }
        return ret == ESP_OK ? 0 : 1;
    }
    
    // Manual exposure setting
    int exposure = atoi(param);
    if (exposure < 1 || exposure > 8000) {
        printf("Exposure must be 1-8000 lines\n");
        return 1;
    }
    
    ESP_LOGI(TAG, "Setting direct exposure to %d lines", exposure);
    
    uint8_t exp_h = (exposure >> 8) & 0xFF;
    uint8_t exp_l = exposure & 0xFF;
    
    esp_err_t ret = ESP_OK;
    ret |= imx708_hw_write_register(0x0202, exp_h);
    ret |= imx708_hw_write_register(0x0203, exp_l);
    
    if (ret == ESP_OK) {
        printf("Exposure set to %d lines\n", exposure);
        printf("  Register 0x0202 = 0x%02X (high byte)\n", exp_h);
        printf("  Register 0x0203 = 0x%02X (low byte)\n", exp_l);
        
        if (exposure < 500) {
            printf("  Effect: FASTER exposure (darker image)\n");
        } else if (exposure > 2000) {
            printf("  Effect: SLOWER exposure (brighter image)\n");
        } else {
            printf("  Effect: NORMAL exposure\n");
        }
    }
    
    return ret == ESP_OK ? 0 : 1;
}

/**
 * @brief Direct sensor gain control
 */
static int cmd_sensor_gain_direct(int argc, char **argv)
{
    if (argc == 1) {
        printf("Direct Sensor Gain Control\n");
        printf("==========================\n");
        printf("Usage: sensor_gain_direct <gain_value>\n");
        printf("       sensor_gain_direct current\n");
        printf("       sensor_gain_direct auto <enable|disable>\n\n");
        printf("Values: 112-960 (IMX708 analog gain range)\n");
        printf("Lower = less sensitive, Higher = more sensitive\n");
        return 0;
    }
    
    const char *param = argv[1];
    
    if (strcmp(param, "current") == 0) {
        uint8_t gain_h, gain_l, agc_enable;
        imx708_hw_read_register(0x0204, &gain_h);
        imx708_hw_read_register(0x0205, &gain_l);
        imx708_hw_read_register(IMX708_REG_AGC_ENABLE, &agc_enable);
        
        uint16_t gain = (gain_h << 8) | gain_l;
        
        printf("Current Gain Settings:\n");
        printf("  Analog Gain: %d (0x%04X)\n", gain, gain);
        printf("  Auto Gain: %s\n", agc_enable ? "ENABLED" : "DISABLED");
        
        return 0;
    }
    
    if (strcmp(param, "auto") == 0) {
        if (argc != 3) {
            printf("Usage: sensor_gain_direct auto <enable|disable>\n");
            return 1;
        }
        
        bool enable = strcmp(argv[2], "enable") == 0;
        uint8_t agc_val = enable ? 0x01 : 0x00;
        
        esp_err_t ret = imx708_hw_write_register(IMX708_REG_AGC_ENABLE, agc_val);
        
        if (ret == ESP_OK) {
            printf("Sensor Auto Gain %s\n", enable ? "ENABLED" : "DISABLED");
        }
        return ret == ESP_OK ? 0 : 1;
    }
    
    // Manual gain setting
    int gain = atoi(param);
    if (gain < 112 || gain > 960) {
        printf("Gain must be 112-960 (IMX708 range)\n");
        return 1;
    }
    
    uint8_t gain_h = (gain >> 8) & 0xFF;
    uint8_t gain_l = gain & 0xFF;
    
    esp_err_t ret = ESP_OK;
    ret |= imx708_hw_write_register(0x0204, gain_h);
    ret |= imx708_hw_write_register(0x0205, gain_l);
    
    if (ret == ESP_OK) {
        printf("Analog gain set to %d\n", gain);
        printf("  Register 0x0204 = 0x%02X (high byte)\n", gain_h);
        printf("  Register 0x0205 = 0x%02X (low byte)\n", gain_l);
    }
    
    return ret == ESP_OK ? 0 : 1;
}

/**
 * @brief Light adaptation profiles for different conditions
 */
static int cmd_hw_light_profile(int argc, char **argv)
{
    if (argc == 1) {
        printf("Hardware Light Adaptation Profiles\n");
        printf("===================================\n");
        printf("Usage: hw_light_profile <profile>\n");
        printf("       hw_light_profile current\n\n");
        printf("Profiles:\n");
        printf("  bright    - High light conditions (fast exposure, low gain)\n");
        printf("  normal    - Normal lighting (balanced settings)\n");
        printf("  dim       - Low light (longer exposure, higher gain)\n");
        printf("  dark      - Very low light (max sensitivity)\n");
        printf("  custom <exp> <gain> - Custom exposure and gain values\n\n");
        printf("This bypasses ESP IPA and controls hardware directly\n");
        return 0;
    }
    
    const char *profile = argv[1];
    
    if (strcmp(profile, "current") == 0) {
        uint8_t exp_h, exp_l, gain_h, gain_l;
        imx708_hw_read_register(0x0202, &exp_h);
        imx708_hw_read_register(0x0203, &exp_l);
        imx708_hw_read_register(0x0204, &gain_h);
        imx708_hw_read_register(0x0205, &gain_l);
        
        uint16_t exposure = (exp_h << 8) | exp_l;
        uint16_t gain = (gain_h << 8) | gain_l;
        
        printf("Current Hardware Settings:\n");
        printf("  Exposure: %d lines\n", exposure);
        printf("  Gain: %d\n", gain);
        
        // Determine profile
        if (exposure < 500 && gain < 200) {
            printf("  Profile: BRIGHT conditions\n");
        } else if (exposure < 1500 && gain < 400) {
            printf("  Profile: NORMAL conditions\n");
        } else if (exposure < 3000 && gain < 600) {
            printf("  Profile: DIM conditions\n");
        } else {
            printf("  Profile: DARK conditions\n");
        }
        
        return 0;
    }
    
    uint16_t exposure, gain;
    
    if (strcmp(profile, "bright") == 0) {
        exposure = 400;   // Fast exposure
        gain = 150;       // Low gain
        printf("Applying BRIGHT profile (exposure=%d, gain=%d)\n", exposure, gain);
    } else if (strcmp(profile, "normal") == 0) {
        exposure = 1000;  // Normal exposure
        gain = 300;       // Medium gain
        printf("Applying NORMAL profile (exposure=%d, gain=%d)\n", exposure, gain);
    } else if (strcmp(profile, "dim") == 0) {
        exposure = 2500;  // Longer exposure
        gain = 500;       // Higher gain
        printf("Applying DIM profile (exposure=%d, gain=%d)\n", exposure, gain);
    } else if (strcmp(profile, "dark") == 0) {
        exposure = 4000;  // Long exposure
        gain = 800;       // High gain
        printf("Applying DARK profile (exposure=%d, gain=%d)\n", exposure, gain);
    } else if (strcmp(profile, "custom") == 0) {
        if (argc != 4) {
            printf("Usage: hw_light_profile custom <exposure> <gain>\n");
            return 1;
        }
        exposure = atoi(argv[2]);
        gain = atoi(argv[3]);
        
        if (exposure < 1 || exposure > 8000 || gain < 112 || gain > 960) {
            printf("Invalid values. Exposure: 1-8000, Gain: 112-960\n");
            return 1;
        }
        
        printf("Applying CUSTOM profile (exposure=%d, gain=%d)\n", exposure, gain);
    } else {
        printf("Unknown profile: %s\n", profile);
        return 1;
    }
    
    // Apply settings
    esp_err_t ret = ESP_OK;
    
    // Set exposure
    uint8_t exp_h = (exposure >> 8) & 0xFF;
    uint8_t exp_l = exposure & 0xFF;
    ret |= imx708_hw_write_register(0x0202, exp_h);
    ret |= imx708_hw_write_register(0x0203, exp_l);
    
    // Set gain
    uint8_t gain_h = (gain >> 8) & 0xFF;
    uint8_t gain_l = gain & 0xFF;
    ret |= imx708_hw_write_register(0x0204, gain_h);
    ret |= imx708_hw_write_register(0x0205, gain_l);
    
    if (ret == ESP_OK) {
        printf("Profile applied successfully!\n");
        printf("Hardware now optimized for light conditions\n");
    } else {
        printf("Error applying profile\n");
    }
    
    return ret == ESP_OK ? 0 : 1;
}

/**
 * @brief Read all hardware registers status for diagnostics
 */
static int cmd_hw_registers_dump(int argc, char **argv)
{
    // Refuse to run while streaming is active to avoid SCCB contention and stalls
    printf("IMX708 hardware register dump is disabled during streaming. Stop the stream first.\n");
    return 1;

    // The detailed dump below is intentionally disabled in streaming mode.
    // If needed for offline diagnostics, re-enable after stopping the pipeline.
    printf("IMX708 Hardware Registers Status:\n");
    printf("=====================================\n");
    
    // Exposure Registers
    printf("\nExposure Control:\n");
    uint8_t exp_h, exp_l;
    if (imx708_hw_read_register(0x0202, &exp_h) == ESP_OK &&
        imx708_hw_read_register(0x0203, &exp_l) == ESP_OK) {
        uint16_t exposure = (exp_h << 8) | exp_l;
        printf("  Exposure Time:   0x0202/0x0203 = 0x%04X (%d lines)\n", exposure, exposure);
    }
    
    // Auto Exposure
    uint8_t ae_enable;
    if (imx708_hw_read_register(IMX708_REG_AE_ENABLE, &ae_enable) == ESP_OK) {
        printf("  AE Enable:       0x%04X = 0x%02X (%s)\n", 
               IMX708_REG_AE_ENABLE, ae_enable, ae_enable ? "ON" : "OFF");
    }
    
    // Gain Registers
    printf("\nGain Control:\n");
    uint8_t gain_h, gain_l;
    if (imx708_hw_read_register(0x0204, &gain_h) == ESP_OK &&
        imx708_hw_read_register(0x0205, &gain_l) == ESP_OK) {
        uint16_t gain = (gain_h << 8) | gain_l;
        printf("  Analog Gain:     0x0204/0x0205 = 0x%04X (%d)\n", gain, gain);
    }
    
    // Color Balance Registers (read-only)
    printf("\nColor Balance (read-only):\n");
    uint8_t red_h, red_l, blue_h, blue_l;
    if (imx708_hw_read_register(0x0B90, &red_h) == ESP_OK &&
        imx708_hw_read_register(0x0B91, &red_l) == ESP_OK) {
        uint16_t red_gain = (red_h << 8) | red_l;
        printf("  Red Gain:        0x0B90/0x0B91 = 0x%04X (%d)\n", red_gain, red_gain);
    }
    if (imx708_hw_read_register(0x0B92, &blue_h) == ESP_OK &&
        imx708_hw_read_register(0x0B93, &blue_l) == ESP_OK) {
        uint16_t blue_gain = (blue_h << 8) | blue_l;
        printf("  Blue Gain:       0x0B92/0x0B93 = 0x%04X (%d)\n", blue_gain, blue_gain);
    }
    
    printf("\nNOTES:\n");
    printf("  • Exposure and gain controls work directly\n");
    printf("  • Color processing handled by ESP IPA\n");
    printf("  • Use hw_light_profile for quick adaptation\n");
    
    return 0;
}

/**
 * @brief Register hardware control commands
 */
void imx708_hardware_shell_register_commands(void)
{
    const esp_console_cmd_t hw_commands[] = {
        {
            .command = "sensor_exposure_direct",
            .help = "Direct sensor exposure control",
            .hint = "<lines|current|auto>",
            .func = &cmd_sensor_exposure_direct,
        },
        {
            .command = "sensor_gain_direct", 
            .help = "Direct sensor gain control",
            .hint = "<gain|current|auto>",
            .func = &cmd_sensor_gain_direct,
        },
        {
            .command = "hw_light_profile",
            .help = "Hardware light adaptation profiles",
            .hint = "<bright|normal|dim|dark|custom>",
            .func = &cmd_hw_light_profile,
        },
        {
            .command = "hw_registers_dump",
            .help = "Hardware register diagnostics",
            .hint = NULL,
            .func = &cmd_hw_registers_dump,
        }
    };
    
    for (int i = 0; i < sizeof(hw_commands) / sizeof(hw_commands[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&hw_commands[i]));
    }
    
    ESP_LOGI(TAG, "IMX708 Hardware Control Commands Registered!");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "HARDWARE COMMANDS:");
    ESP_LOGI(TAG, "  sensor_exposure_direct  - Direct exposure control (bypasses IPA)");
    ESP_LOGI(TAG, "  sensor_gain_direct      - Direct gain control (bypasses IPA)");
    ESP_LOGI(TAG, "  hw_light_profile        - Light adaptation profiles");
    ESP_LOGI(TAG, "  hw_registers_dump       - Register diagnostics");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "STRATEGY: Direct hardware control for light adaptation");
}
