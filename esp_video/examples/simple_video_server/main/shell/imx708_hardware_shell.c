/*
 * IMX708 Hardware Register Control Shell Commands
 * Direct register access for advanced camera parameter control
 * These commands write directly to IMX708 hardware registers
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "imx708.h"
#include "imx708_hardware_controls.h"
#include "imx708_hardware_shell.h"

static const char* TAG = "imx708_hw_shell";

/**
 * @brief Hardware AWB Speed Control
 * Controls convergence speed of AWB algorithm
 */
static int cmd_hw_awb_speed(int argc, char **argv)
{
    if (argc == 1) {
        // Read current value
        uint8_t current_value = 0;
        esp_err_t ret = imx708_hw_read_register(IMX708_REG_AWB_SPEED, &current_value);
        if (ret == ESP_OK) {
            printf("AWB Speed: 0x%02X (%d)\n", current_value, current_value);
            printf("Range: 0x10-0xFF (faster convergence with higher values)\n");
        } else {
            printf("Failed to read AWB speed register\n");
        }
        return 0;
    }
    
    int speed = strtol(argv[1], NULL, 0); // Support hex (0x) and decimal
    if (speed < 0x10 || speed > 0xFF) {
        printf("Invalid speed value. Range: 0x10-0xFF (16-255)\n");
        return 1;
    }
    
    esp_err_t ret = imx708_hw_write_register(IMX708_REG_AWB_SPEED, (uint8_t)speed);
    if (ret == ESP_OK) {
        printf("AWB Speed set to: 0x%02X (%d)\n", speed, speed);
        printf("Higher values = faster convergence\n");
    } else {
        printf("Failed to write AWB speed register\n");
        return 1;
    }
    
    return 0;
}

/**
 * @brief Hardware Auto Exposure Target
 * Sets target brightness for AE algorithm
 */
static int cmd_hw_ae_target(int argc, char **argv)
{
    if (argc == 1) {
        // Read current value
        uint8_t current_value = 0;
        esp_err_t ret = imx708_hw_read_register(IMX708_REG_AE_TARGET, &current_value);
        if (ret == ESP_OK) {
            printf("AE Target: 0x%02X (%d)\n", current_value, current_value);
            printf("Range: 0-255 (target brightness level)\n");
        } else {
            printf("Failed to read AE target register\n");
        }
        return 0;
    }
    
    int target = strtol(argv[1], NULL, 0);
    if (target < 0 || target > 255) {
        printf("Invalid target value. Range: 0-255\n");
        return 1;
    }
    
    esp_err_t ret = imx708_hw_write_register(IMX708_REG_AE_TARGET, (uint8_t)target);
    if (ret == ESP_OK) {
        printf("AE Target set to: 0x%02X (%d)\n", target, target);
        printf("Lower values = darker image, higher = brighter\n");
    } else {
        printf("Failed to write AE target register\n");
        return 1;
    }
    
    return 0;
}

/**
 * @brief Hardware Auto Exposure Enable/Disable
 */
static int cmd_hw_ae_enable(int argc, char **argv)
{
    if (argc == 1) {
        // Read current value
        uint8_t current_value = 0;
        esp_err_t ret = imx708_hw_read_register(IMX708_REG_AE_ENABLE, &current_value);
        if (ret == ESP_OK) {
            printf("Hardware AE: %s (0x%02X)\n", 
                   current_value ? "ENABLED" : "DISABLED", current_value);
        } else {
            printf("Failed to read AE enable register\n");
        }
        return 0;
    }
    
    uint8_t enable_value = 0;
    if (strcmp(argv[1], "enable") == 0 || strcmp(argv[1], "1") == 0) {
        enable_value = 1;
    } else if (strcmp(argv[1], "disable") == 0 || strcmp(argv[1], "0") == 0) {
        enable_value = 0;
    } else {
        printf("Usage: hw_ae_enable <enable|disable|1|0>\n");
        return 1;
    }
    
    esp_err_t ret = imx708_hw_write_register(IMX708_REG_AE_ENABLE, enable_value);
    if (ret == ESP_OK) {
        printf("Hardware AE %s\n", enable_value ? "ENABLED" : "DISABLED");
        printf("Register 0x%04X = 0x%02X\n", IMX708_REG_AE_ENABLE, enable_value);
    } else {
        printf("Failed to write AE enable register\n");
        return 1;
    }
    
    return 0;
}

/**
 * @brief Hardware Noise Reduction Control
 */
static int cmd_hw_noise_reduction(int argc, char **argv)
{
    if (argc == 1) {
        // Read current values
        uint8_t enable = 0, strength = 0;
        esp_err_t ret1 = imx708_hw_read_register(IMX708_REG_NR_ENABLE, &enable);
        esp_err_t ret2 = imx708_hw_read_register(IMX708_REG_NR_STRENGTH, &strength);
        
        if (ret1 == ESP_OK && ret2 == ESP_OK) {
            printf("Noise Reduction: %s\n", enable ? "ENABLED" : "DISABLED");
            printf("Strength: 0x%02X (%d)\n", strength, strength);
            printf("Usage: hw_noise_reduction <enable|disable> [strength]\n");
        } else {
            printf("Failed to read noise reduction registers\n");
        }
        return 0;
    }
    
    uint8_t enable_value = 0;
    if (strcmp(argv[1], "enable") == 0) {
        enable_value = 1;
    } else if (strcmp(argv[1], "disable") == 0) {
        enable_value = 0;
    } else {
        printf("Usage: hw_noise_reduction <enable|disable> [strength]\n");
        return 1;
    }
    
    // Write enable/disable
    esp_err_t ret = imx708_hw_write_register(IMX708_REG_NR_ENABLE, enable_value);
    if (ret != ESP_OK) {
        printf("Failed to write NR enable register\n");
        return 1;
    }
    
    // Write strength if provided
    if (argc >= 3 && enable_value) {
        int strength = strtol(argv[2], NULL, 0);
        if (strength < 0x10 || strength > 0x80) {
            printf("Invalid strength. Range: 0x10-0x80 (16-128)\n");
            return 1;
        }
        
        ret = imx708_hw_write_register(IMX708_REG_NR_STRENGTH, (uint8_t)strength);
        if (ret == ESP_OK) {
            printf("Noise Reduction ENABLED, Strength: 0x%02X (%d)\n", strength, strength);
        } else {
            printf("Failed to write NR strength register\n");
            return 1;
        }
    } else {
        printf("Noise Reduction %s\n", enable_value ? "ENABLED" : "DISABLED");
    }
    
    return 0;
}

/**
 * @brief Hardware Color Balance Direct Register Control
 */
static int cmd_hw_color_balance(int argc, char **argv)
{
    if (argc == 1) {
        // Read current values
        uint8_t red_gain = 0, blue_gain = 0;
        esp_err_t ret1 = imx708_hw_read_register(IMX708_REG_COLOR_BALANCE_RED, &red_gain);
        esp_err_t ret2 = imx708_hw_read_register(IMX708_REG_COLOR_BALANCE_BLUE, &blue_gain);
        
        if (ret1 == ESP_OK && ret2 == ESP_OK) {
            printf("Hardware Color Balance:\n");
            printf("Red Gain:  0x%02X (%d) - %.2fx\n", red_gain, red_gain, red_gain/100.0);
            printf("Blue Gain: 0x%02X (%d) - %.2fx\n", blue_gain, blue_gain, blue_gain/100.0);
            printf("Usage: hw_color_balance <red_gain> <blue_gain>\n");
            printf("Range: 100-255 (1.0x-2.55x gain)\n");
        } else {
            printf("Failed to read color balance registers\n");
        }
        return 0;
    }
    
    if (argc < 3) {
        printf("Usage: hw_color_balance <red_gain> <blue_gain>\n");
        printf("Range: 100-255 (1.0x-2.55x gain)\n");
        return 1;
    }
    
    int red_gain = strtol(argv[1], NULL, 0);
    int blue_gain = strtol(argv[2], NULL, 0);
    
    if (red_gain < 100 || red_gain > 255 || blue_gain < 100 || blue_gain > 255) {
        printf("Invalid gain values. Range: 100-255\n");
        return 1;
    }
    
    esp_err_t ret1 = imx708_hw_write_register(IMX708_REG_COLOR_BALANCE_RED, (uint8_t)red_gain);
    esp_err_t ret2 = imx708_hw_write_register(IMX708_REG_COLOR_BALANCE_BLUE, (uint8_t)blue_gain);
    
    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        printf("Hardware Color Balance Set:\n");
        printf("Red Gain:  0x%02X (%d) - %.2fx\n", red_gain, red_gain, red_gain/100.0);
        printf("Blue Gain: 0x%02X (%d) - %.2fx\n", blue_gain, blue_gain, blue_gain/100.0);
    } else {
        printf("Failed to write color balance registers\n");
        return 1;
    }
    
    return 0;
}

/**
 * @brief Hardware Low-Light Optimization
 */
static int cmd_hw_lowlight_opt(int argc, char **argv)
{
    if (argc == 1) {
        // Read current values
        uint8_t red_bias = 0, blue_bias = 0, green_supp = 0;
        esp_err_t ret1 = imx708_hw_read_register(IMX708_REG_LOWLIGHT_RED_BIAS, &red_bias);
        esp_err_t ret2 = imx708_hw_read_register(IMX708_REG_LOWLIGHT_BLUE_BIAS, &blue_bias);
        esp_err_t ret3 = imx708_hw_read_register(IMX708_REG_LOWLIGHT_GREEN_SUPP, &green_supp);
        
        if (ret1 == ESP_OK && ret2 == ESP_OK && ret3 == ESP_OK) {
            printf("Low-Light Optimization:\n");
            printf("Red Bias:   0x%02X (%d)\n", red_bias, red_bias);
            printf("Blue Bias:  0x%02X (%d)\n", blue_bias, blue_bias);
            printf("Green Supp: 0x%02X (%d)\n", green_supp, green_supp);
            printf("Usage: hw_lowlight_opt <red_bias> <blue_bias> <green_supp>\n");
        } else {
            printf("Failed to read low-light registers\n");
        }
        return 0;
    }
    
    if (argc < 4) {
        printf("Usage: hw_lowlight_opt <red_bias> <blue_bias> <green_supp>\n");
        printf("Range: 0-255 for each parameter\n");
        return 1;
    }
    
    int red_bias = strtol(argv[1], NULL, 0);
    int blue_bias = strtol(argv[2], NULL, 0);
    int green_supp = strtol(argv[3], NULL, 0);
    
    if (red_bias < 0 || red_bias > 255 || 
        blue_bias < 0 || blue_bias > 255 || 
        green_supp < 0 || green_supp > 255) {
        printf("Invalid values. Range: 0-255\n");
        return 1;
    }
    
    esp_err_t ret1 = imx708_hw_write_register(IMX708_REG_LOWLIGHT_RED_BIAS, (uint8_t)red_bias);
    esp_err_t ret2 = imx708_hw_write_register(IMX708_REG_LOWLIGHT_BLUE_BIAS, (uint8_t)blue_bias);
    esp_err_t ret3 = imx708_hw_write_register(IMX708_REG_LOWLIGHT_GREEN_SUPP, (uint8_t)green_supp);
    
    if (ret1 == ESP_OK && ret2 == ESP_OK && ret3 == ESP_OK) {
        printf("Low-Light Optimization Set:\n");
        printf("Red Bias:   0x%02X (%d)\n", red_bias, red_bias);
        printf("Blue Bias:  0x%02X (%d)\n", blue_bias, blue_bias);
        printf("Green Supp: 0x%02X (%d)\n", green_supp, green_supp);
    } else {
        printf("Failed to write low-light registers\n");
        return 1;
    }
    
    return 0;
}

/**
 * @brief Read all hardware registers status
 */
static int cmd_hw_registers_dump(int argc, char **argv)
{
    printf("IMX708 Hardware Registers Status:\n");
    printf("=====================================\n");
    
    // AWB Registers
    printf("\nAuto White Balance:\n");
    uint8_t value;
    if (imx708_hw_read_register(IMX708_REG_AWB_ENABLE, &value) == ESP_OK) {
        printf("  AWB Enable:      0x%04X = 0x%02X (%s)\n", 
               IMX708_REG_AWB_ENABLE, value, value ? "ON" : "OFF");
    }
    if (imx708_hw_read_register(IMX708_REG_AWB_SPEED, &value) == ESP_OK) {
        printf("  AWB Speed:       0x%04X = 0x%02X (%d)\n", 
               IMX708_REG_AWB_SPEED, value, value);
    }
    
    // AE Registers
    printf("\nAuto Exposure:\n");
    if (imx708_hw_read_register(IMX708_REG_AE_ENABLE, &value) == ESP_OK) {
        printf("  AE Enable:       0x%04X = 0x%02X (%s)\n", 
               IMX708_REG_AE_ENABLE, value, value ? "ON" : "OFF");
    }
    if (imx708_hw_read_register(IMX708_REG_AE_TARGET, &value) == ESP_OK) {
        printf("  AE Target:       0x%04X = 0x%02X (%d)\n", 
               IMX708_REG_AE_TARGET, value, value);
    }
    
    // Color Balance
    printf("\nColor Balance:\n");
    if (imx708_hw_read_register(IMX708_REG_COLOR_BALANCE_RED, &value) == ESP_OK) {
        printf("  Red Gain:        0x%04X = 0x%02X (%.2fx)\n", 
               IMX708_REG_COLOR_BALANCE_RED, value, value/100.0);
    }
    if (imx708_hw_read_register(IMX708_REG_COLOR_BALANCE_BLUE, &value) == ESP_OK) {
        printf("  Blue Gain:       0x%04X = 0x%02X (%.2fx)\n", 
               IMX708_REG_COLOR_BALANCE_BLUE, value, value/100.0);
    }
    
    // Noise Reduction
    printf("\nNoise Reduction:\n");
    if (imx708_hw_read_register(IMX708_REG_NR_ENABLE, &value) == ESP_OK) {
        printf("  NR Enable:       0x%04X = 0x%02X (%s)\n", 
               IMX708_REG_NR_ENABLE, value, value ? "ON" : "OFF");
    }
    if (imx708_hw_read_register(IMX708_REG_NR_STRENGTH, &value) == ESP_OK) {
        printf("  NR Strength:     0x%04X = 0x%02X (%d)\n", 
               IMX708_REG_NR_STRENGTH, value, value);
    }
    
    return 0;
}

/**
 * @brief Register all hardware control commands
 */
void imx708_hardware_shell_register_commands(void)
{
    const esp_console_cmd_t hw_commands[] = {
        {
            .command = "hw_awb_speed",
            .help = "Control AWB convergence speed (0x10-0xFF)",
            .hint = NULL,
            .func = &cmd_hw_awb_speed,
        },
        {
            .command = "hw_ae_target",
            .help = "Set Auto Exposure target brightness (0-255)",
            .hint = NULL,
            .func = &cmd_hw_ae_target,
        },
        {
            .command = "hw_ae_enable",
            .help = "Enable/disable hardware Auto Exposure",
            .hint = NULL,
            .func = &cmd_hw_ae_enable,
        },
        {
            .command = "hw_noise_reduction",
            .help = "Control hardware noise reduction",
            .hint = NULL,
            .func = &cmd_hw_noise_reduction,
        },
        {
            .command = "hw_color_balance",
            .help = "Direct hardware color balance control",
            .hint = NULL,
            .func = &cmd_hw_color_balance,
        },
        {
            .command = "hw_lowlight_opt",
            .help = "Low-light optimization parameters",
            .hint = NULL,
            .func = &cmd_hw_lowlight_opt,
        },
        {
            .command = "hw_registers_dump",
            .help = "Dump all hardware register values",
            .hint = NULL,
            .func = &cmd_hw_registers_dump,
        }
    };
    
    for (int i = 0; i < sizeof(hw_commands) / sizeof(hw_commands[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&hw_commands[i]));
    }
    
    ESP_LOGI(TAG, "IMX708 Hardware control commands registered!");
    ESP_LOGI(TAG, "Available: hw_awb_speed, hw_ae_target, hw_ae_enable, hw_noise_reduction");
    ESP_LOGI(TAG, "          hw_color_balance, hw_lowlight_opt, hw_registers_dump");
}
