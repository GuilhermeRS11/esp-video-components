/*
 * Camera control shell commands for V4L2 interface
 * This file implements essential shell commands for camera control via V4L2
 * 
 * Focus: Basic V4L2 controls that complement hardware register access
 * Hardware-specific controls are handled in imx708_hardware_shell.c
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "linux/videodev2.h"
#include "camera_shell.h"

#define VIDEO_DEVICE "/dev/video0"

static const char* TAG = "camera_shell";

/**
 * @brief Display current camera settings via V4L2
 */
static int cmd_cam_status(int argc, char **argv)
{
    int video_fd = open(VIDEO_DEVICE, O_RDWR);
    if (video_fd < 0) {
        printf("Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
        return 1;
    }

    printf("\nCamera V4L2 Status:\n");
    printf("========================\n");
    
    struct v4l2_ext_controls controls = {0};
    struct v4l2_ext_control control[1];
    
    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count = 1;
    controls.controls = control;
    
    // Get AWB status first (most important for color)
    control[0].id = V4L2_CID_AUTO_WHITE_BALANCE;
    if (ioctl(video_fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
        printf("AWB Status: %s %s\n", 
               control[0].value ? "ENABLED" : "DISABLED",
               control[0].value ? "(Hardware auto)" : "(Manual control)");
    } else {
        printf("AWB Status: Error reading\n");
    }
    
    // Get red balance
    control[0].id = V4L2_CID_RED_BALANCE;
    if (ioctl(video_fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
        printf("Red Balance: %ld (%.1fx gain)\n", control[0].value, control[0].value / 100.0);
    } else {
        printf("Red Balance: Error reading\n");
    }
    
    // Get blue balance
    control[0].id = V4L2_CID_BLUE_BALANCE;
    if (ioctl(video_fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
        printf("Blue Balance: %ld (%.1fx gain)\n", control[0].value, control[0].value / 100.0);
    } else {
        printf("Blue Balance: Error reading\n");
    }
    
    printf("========================\n");
    printf("Use hw_* commands for direct hardware control\n");
    printf("Use hw_registers_dump for complete hardware status\n\n");
    
    close(video_fd);
    return 0;
}

/**
 * @brief Hardware AWB control command via V4L2
 * Controls the IMX708 hardware Auto White Balance system
 */
static int cmd_hw_awb(int argc, char **argv)
{
    if (argc == 1) {
        printf("Usage: hw_awb <enable|disable|status>\n");
        printf("  enable  - Enable hardware AWB algorithm\n");
        printf("  disable - Disable hardware AWB, switch to manual control\n");
        printf("  status  - Show current AWB status\n");
        return 0;
    }
    
    int video_fd = open(VIDEO_DEVICE, O_RDWR);
    if (video_fd < 0) {
        printf("Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
        return 1;
    }
    
    struct v4l2_ext_controls controls = {0};
    struct v4l2_ext_control control[1];
    
    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count = 1;
    controls.controls = control;
    control[0].id = V4L2_CID_AUTO_WHITE_BALANCE;
    
    if (strcmp(argv[1], "enable") == 0) {
        control[0].value = 1;
        if (ioctl(video_fd, VIDIOC_S_EXT_CTRLS, &controls) == 0) {
            printf("Hardware AWB enabled successfully\n");
            printf("AWB algorithm will automatically adjust color balance\n");
            printf("Use hw_* commands for fine-tuning hardware registers\n");
        } else {
            printf("Failed to enable hardware AWB\n");
        }
    } else if (strcmp(argv[1], "disable") == 0) {
        control[0].value = 0;
        if (ioctl(video_fd, VIDIOC_S_EXT_CTRLS, &controls) == 0) {
            printf("Hardware AWB disabled successfully\n");
            printf("Switched to manual color balance mode\n");
            printf("Use hw_color_balance for direct hardware control\n");
        } else {
            printf("Failed to disable hardware AWB\n");
        }
    } else if (strcmp(argv[1], "status") == 0) {
        if (ioctl(video_fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
            printf("Hardware AWB Status: %s\n", control[0].value ? "ENABLED" : "DISABLED");
            if (control[0].value) {
                printf("Automatic color balance is active\n");
                printf("Hardware registers 0x3200-0x320A are controlling AWB\n");
            } else {
                printf("Manual color balance mode active\n");
                printf("Use hw_color_balance for direct register control\n");
            }
        } else {
            printf("Failed to get AWB status\n");
        }
    } else {
        printf("Invalid option: %s\n", argv[1]);
        printf("Usage: hw_awb <enable|disable|status>\n");
    }
    
    close(video_fd);
    return 0;
}

/**
 * @brief Register essential camera control commands
 * Focus on V4L2 status and AWB control that complements hardware commands
 */
void camera_shell_register_commands(void)
{
    const esp_console_cmd_t commands[] = {
        {
            .command = "cam_status",
            .help = "Display current camera V4L2 status",
            .hint = NULL,
            .func = &cmd_cam_status,
        },
        {
            .command = "hw_awb",
            .help = "Control hardware Auto White Balance (enable/disable/status)",
            .hint = NULL,
            .func = &cmd_hw_awb,
        }
    };
    
    for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&commands[i]));
    }
    
    ESP_LOGI(TAG, "Essential camera control commands registered!");
    ESP_LOGI(TAG, "Available: cam_status, hw_awb");
    ESP_LOGI(TAG, "Use hw_* commands for direct hardware register control");
    ESP_LOGI(TAG, "Use hw_registers_dump for complete hardware status");
}
