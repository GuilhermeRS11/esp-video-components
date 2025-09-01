/*
 * Camera control shell commands for dynamic parameter adjustment via V4L2
 * This file implements shell commands to control camera parameters in real-time
 * via V4L2 ioctl interface while video streaming is active
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "linux/videodev2.h"
#include "camera_shell.h"

#define VIDEO_DEVICE "/dev/video0"

static const char* TAG = "camera_shell";

/**
 * @brief Camera brightness command - works with video streaming
 */
static int cmd_cam_brightness(int argc, char **argv)
{
    if (argc == 1) {
        // Show current brightness
        int video_fd = open(VIDEO_DEVICE, O_RDWR);
        if (video_fd < 0) {
            printf("❌Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
            return 1;
        }
        
        struct v4l2_ext_controls controls = {0};
        struct v4l2_ext_control control[1];
        
        controls.ctrl_class = V4L2_CTRL_CLASS_USER;
        controls.count = 1;
        controls.controls = control;
        control[0].id = V4L2_CID_BRIGHTNESS;
        
        if (ioctl(video_fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
            printf("Current brightness: %ld\n", control[0].value);
        } else {
            printf("Error: Failed to get brightness\n");
        }
        close(video_fd);
        return 0;
    } else if (argc != 2) {
        printf("Usage: cam_brightness [<level>]\n");
        printf("  Range: -2 to +2\n");
        printf("  Examples:\n");
        printf("    cam_brightness       # Show current value\n");
        printf("    cam_brightness -1    # Darker\n");
        printf("    cam_brightness 0     # Default\n");
        printf("    cam_brightness 1     # Brighter\n");
        return 1;
    }
    
    int level = atoi(argv[1]);
    
    if (level < -2 || level > 2) {
        printf("Error: Brightness level must be between -2 and +2\n");
        return 1;
    }
    
    printf("Setting camera brightness to level %d...\n", level);
    
    // Open video device
    int video_fd = open(VIDEO_DEVICE, O_RDWR);
    if (video_fd < 0) {
        printf("❌Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
        return 1;
    }
    
    // Set up V4L2 control structure
    struct v4l2_ext_controls controls = {0};
    struct v4l2_ext_control control[1];
    
    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count = 1;
    controls.controls = control;
    
    control[0].id = V4L2_CID_BRIGHTNESS;
    control[0].value = level;
    
    // Send ioctl
    int ret = ioctl(video_fd, VIDIOC_S_EXT_CTRLS, &controls);
    close(video_fd);
    
    if (ret == 0) {
        printf("Brightness set to level %d\n", level);
    } else {
        printf("Error: Failed to set brightness (ret=%d)\n", ret);
        return 1;
    }
    
    return 0;
}

/**
 * @brief Camera contrast command - works with video streaming
 */
static int cmd_cam_contrast(int argc, char **argv)
{
    if (argc == 1) {
        // Show current contrast
        int video_fd = open(VIDEO_DEVICE, O_RDWR);
        if (video_fd < 0) {
            printf("❌Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
            return 1;
        }
        
        struct v4l2_ext_controls controls = {0};
        struct v4l2_ext_control control[1];
        
        controls.ctrl_class = V4L2_CTRL_CLASS_USER;
        controls.count = 1;
        controls.controls = control;
        control[0].id = V4L2_CID_CONTRAST;
        
        if (ioctl(video_fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
            printf("Current contrast: %ld\n", control[0].value);
        } else {
            printf("Error: Failed to get contrast\n");
        }
        close(video_fd);
        return 0;
    } else if (argc != 2) {
        printf("Usage: cam_contrast [<level>]\n");
        printf("  Range: -2 to +2\n");
        printf("  Examples:\n");
        printf("    cam_contrast         # Show current value\n");
        printf("    cam_contrast -1      # Lower contrast\n");
        printf("    cam_contrast 0       # Default\n");
        printf("    cam_contrast 1       # Higher contrast\n");
        return 1;
    }
    
    int level = atoi(argv[1]);
    
    if (level < -2 || level > 2) {
        printf("Error: Contrast level must be between -2 and +2\n");
        return 1;
    }
    
    printf("Setting camera contrast to level %d...\n", level);
    
    // Open video device
    int video_fd = open(VIDEO_DEVICE, O_RDWR);
    if (video_fd < 0) {
        printf("❌Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
        return 1;
    }
    
    // Set up V4L2 control structure
    struct v4l2_ext_controls controls = {0};
    struct v4l2_ext_control control[1];
    
    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count = 1;
    controls.controls = control;
    
    control[0].id = V4L2_CID_CONTRAST;
    control[0].value = level;
    
    // Send ioctl
    int ret = ioctl(video_fd, VIDIOC_S_EXT_CTRLS, &controls);
    close(video_fd);
    
    if (ret == 0) {
        printf("Contrast set to level %d\n", level);
    } else {
        printf("Error: Failed to set contrast (ret=%d)\n", ret);
        return 1;
    }
    
    return 0;
}

/**
 * @brief Camera saturation command - works with video streaming
 */
static int cmd_cam_saturation(int argc, char **argv)
{
    if (argc == 1) {
        // Show current saturation
        int video_fd = open(VIDEO_DEVICE, O_RDWR);
        if (video_fd < 0) {
            printf("❌Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
            return 1;
        }
        
        struct v4l2_ext_controls controls = {0};
        struct v4l2_ext_control control[1];
        
        controls.ctrl_class = V4L2_CTRL_CLASS_USER;
        controls.count = 1;
        controls.controls = control;
        control[0].id = V4L2_CID_SATURATION;
        
        if (ioctl(video_fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
            printf("Current saturation: %ld\n", control[0].value);
        } else {
            printf("Error: Failed to get saturation\n");
        }
        close(video_fd);
        return 0;
    } else if (argc != 2) {
        printf("Usage: cam_saturation [<level>]\n");
        printf("  Range: -2 to +2\n");
        printf("  Examples:\n");
        printf("    cam_saturation       # Show current value\n");
        printf("    cam_saturation -1    # Less saturated\n");
        printf("    cam_saturation 0     # Default\n");
        printf("    cam_saturation 1     # More saturated\n");
        return 1;
    }
    
    int level = atoi(argv[1]);
    
    if (level < -2 || level > 2) {
        printf("Error: Saturation level must be between -2 and +2\n");
        return 1;
    }
    
    printf("Setting camera saturation to level %d...\n", level);
    
    // Open video device
    int video_fd = open(VIDEO_DEVICE, O_RDWR);
    if (video_fd < 0) {
        printf("❌Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
        return 1;
    }
    
    // Set up V4L2 control structure
    struct v4l2_ext_controls controls = {0};
    struct v4l2_ext_control control[1];
    
    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count = 1;
    controls.controls = control;
    
    control[0].id = V4L2_CID_SATURATION;
    control[0].value = level;
    
    // Send ioctl
    int ret = ioctl(video_fd, VIDIOC_S_EXT_CTRLS, &controls);
    close(video_fd);
    
    if (ret == 0) {
        printf("Saturation set to level %d\n", level);
    } else {
        printf("Error: Failed to set saturation (ret=%d)\n", ret);
        return 1;
    }
    
    return 0;
}

/**
 * @brief Camera red color balance command - works with video streaming
 */
static int cmd_cam_red_balance(int argc, char **argv)
{
    if (argc == 1) {
        // Show current red balance
        int video_fd = open(VIDEO_DEVICE, O_RDWR);
        if (video_fd < 0) {
            printf("❌Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
            return 1;
        }
        
        struct v4l2_ext_controls controls = {0};
        struct v4l2_ext_control control[1];
        
        controls.ctrl_class = V4L2_CTRL_CLASS_USER;
        controls.count = 1;
        controls.controls = control;
        control[0].id = V4L2_CID_RED_BALANCE;
        
        if (ioctl(video_fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
            printf("Current red balance: %ld\n", control[0].value);
        } else {
            printf("Error: Failed to get red balance\n");
        }
        close(video_fd);
        return 0;
    } else if (argc != 2) {
        printf("Usage: cam_red_balance [<gain>]\n");
        printf("  Range: 100 to 2000 (1.0x to 20.0x gain)\n");
        printf("  Examples:\n");
        printf("    cam_red_balance        # Show current value\n");
        printf("    cam_red_balance 400    # Set to 4.0x gain (default)\n");
        printf("    cam_red_balance 600    # Increase red (reduce green tint)\n");
        printf("    cam_red_balance 200    # Decrease red\n");
        return 1;
    }
    
    int gain = atoi(argv[1]);
    
    if (gain < 100 || gain > 2000) {
        printf("Error: Red balance gain must be between 100 and 2000\n");
        return 1;
    }
    
    printf("Setting camera red balance to %d (%.1fx gain)...\n", gain, gain / 100.0);
    
    // Open video device
    int video_fd = open(VIDEO_DEVICE, O_RDWR);
    if (video_fd < 0) {
        printf("❌Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
        return 1;
    }
    
    // Set up V4L2 control structure
    struct v4l2_ext_controls controls = {0};
    struct v4l2_ext_control control[1];
    
    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count = 1;
    controls.controls = control;
    
    control[0].id = V4L2_CID_RED_BALANCE;
    control[0].value = gain;
    
    // Send ioctl
    int ret = ioctl(video_fd, VIDIOC_S_EXT_CTRLS, &controls);
    close(video_fd);
    
    if (ret == 0) {
        printf("Red balance set to %d (%.1fx gain)\n", gain, gain / 100.0);
    } else {
        printf("Error: Failed to set red balance (ret=%d)\n", ret);
        return 1;
    }
    
    return 0;
}

/**
 * @brief Camera blue color balance command - works with video streaming
 */
static int cmd_cam_blue_balance(int argc, char **argv)
{
    if (argc == 1) {
        // Show current blue balance
        int video_fd = open(VIDEO_DEVICE, O_RDWR);
        if (video_fd < 0) {
            printf("❌Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
            return 1;
        }
        
        struct v4l2_ext_controls controls = {0};
        struct v4l2_ext_control control[1];
        
        controls.ctrl_class = V4L2_CTRL_CLASS_USER;
        controls.count = 1;
        controls.controls = control;
        control[0].id = V4L2_CID_BLUE_BALANCE;
        
        if (ioctl(video_fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
            printf("Current blue balance: %ld\n", control[0].value);
        } else {
            printf("Error: Failed to get blue balance\n");
        }
        close(video_fd);
        return 0;
    } else if (argc != 2) {
        printf("Usage: cam_blue_balance [<gain>]\n");
        printf("  Range: 100 to 2000 (1.0x to 20.0x gain)\n");
        printf("  Examples:\n");
        printf("    cam_blue_balance        # Show current value\n");
        printf("    cam_blue_balance 280    # Set to 2.8x gain (default)\n");
        printf("    cam_blue_balance 400    # Increase blue\n");
        printf("    cam_blue_balance 150    # Decrease blue\n");
        return 1;
    }
    
    int gain = atoi(argv[1]);
    
    if (gain < 100 || gain > 2000) {
        printf("Error: Blue balance gain must be between 100 and 2000\n");
        return 1;
    }
    
    printf("Setting camera blue balance to %d (%.1fx gain)...\n", gain, gain / 100.0);
    
    // Open video device
    int video_fd = open(VIDEO_DEVICE, O_RDWR);
    if (video_fd < 0) {
        printf("❌Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
        return 1;
    }
    
    // Set up V4L2 control structure
    struct v4l2_ext_controls controls = {0};
    struct v4l2_ext_control control[1];
    
    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count = 1;
    controls.controls = control;
    
    control[0].id = V4L2_CID_BLUE_BALANCE;
    control[0].value = gain;
    
    // Send ioctl
    int ret = ioctl(video_fd, VIDIOC_S_EXT_CTRLS, &controls);
    close(video_fd);
    
    if (ret == 0) {
        printf("Blue balance set to %d (%.1fx gain)\n", gain, gain / 100.0);
    } else {
        printf("Error: Failed to set blue balance (ret=%d)\n", ret);
        return 1;
    }
    
    return 0;
}

/**
 * @brief Display current camera settings
 */
static int cmd_cam_status(int argc, char **argv)
{
    int video_fd = open(VIDEO_DEVICE, O_RDWR);
    if (video_fd < 0) {
        printf("Error:Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
        return 1;
    }

    printf("\nCurrent Camera Settings:\n");
    printf("===========================\n");
    
    // Get brightness
    struct v4l2_ext_controls controls = {0};
    struct v4l2_ext_control control[1];
    
    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count = 1;
    controls.controls = control;
    
    control[0].id = V4L2_CID_BRIGHTNESS;
    if (ioctl(video_fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
        printf("Brightness: %ld\n", control[0].value);
    } else {
        printf("Brightness: Error reading\n");
    }
    
    // Get contrast
    control[0].id = V4L2_CID_CONTRAST;
    if (ioctl(video_fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
        printf("Contrast:   %ld\n", control[0].value);
    } else {
        printf("Contrast:   Error reading\n");
    }
    
    // Get saturation
    control[0].id = V4L2_CID_SATURATION;
    if (ioctl(video_fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
        printf("Saturation: %ld\n", control[0].value);
    } else {
        printf("Saturation: Error reading\n");
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
    
    printf("===========================\n\n");
    
    close(video_fd);
    return 0;
}

/**
 * @brief Register all camera control commands
 */
void camera_shell_register_commands(void)
{
    const esp_console_cmd_t commands[] = {
        {
            .command = "cam_brightness",
            .help = "Get/set brightness (-2 to +2)",
            .hint = NULL,
            .func = &cmd_cam_brightness,
        },
        {
            .command = "cam_contrast",
            .help = "Get/set contrast (-2 to +2)",
            .hint = NULL,
            .func = &cmd_cam_contrast,
        },
        {
            .command = "cam_saturation",
            .help = "Get/set saturation (-2 to +2)",
            .hint = NULL,
            .func = &cmd_cam_saturation,
        },
        {
            .command = "cam_red_balance",
            .help = "Get/set red color balance (100-2000, 1.0x-20.0x gain)",
            .hint = NULL,
            .func = &cmd_cam_red_balance,
        },
        {
            .command = "cam_blue_balance",
            .help = "Get/set blue color balance (100-2000, 1.0x-20.0x gain)",
            .hint = NULL,
            .func = &cmd_cam_blue_balance,
        },
        {
            .command = "cam_status",
            .help = "Display current camera settings",
            .hint = NULL,
            .func = &cmd_cam_status,
        }
    };
    
    for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&commands[i]));
    }
    
    ESP_LOGI(TAG, "Camera control commands registered successfully!");
    ESP_LOGI(TAG, "Available commands: cam_brightness, cam_contrast, cam_saturation, cam_red_balance, cam_blue_balance, cam_status");
    ESP_LOGI(TAG, "Color balance commands use gain values: 100-2000 (1.0x-20.0x gain multiplier)");
    ESP_LOGI(TAG, "These commands work while video streaming is active!");
}
