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
#include "linux/v4l2-controls.h"

#define VIDEO_DEVICE "/dev/video0"

static const char* TAG = "camera_shell";

// Helper: fetch min/max/step for a V4L2 control
static int get_ctrl_range(int fd, uint32_t id, int32_t *min, int32_t *max, int32_t *step)
{
    struct v4l2_query_ext_ctrl q = {0};
    q.id = id;
    if (ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &q) != 0) {
        return -1;
    }
    if (min)  *min  = (int32_t)q.minimum;
    if (max)  *max  = (int32_t)q.maximum;
    if (step) *step = (int32_t)q.step;
    return 0;
}

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
 * @brief Set/Get motor focus position via V4L2 (maps to ESP_CAM_MOTOR_POSITION_CODE)
 * Usage:
 *   cam_focus <pos>    # 0..1023 typical DW9714 range
 *   cam_focus current  # read current focus position
 */
static int cmd_cam_focus(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage:\n");
        printf("  cam_focus <position>   # set absolute focus position (0..1023)\n");
        printf("  cam_focus current      # read current focus position\n");
        return 0;
    }

    int video_fd = open(VIDEO_DEVICE, O_RDWR);
    if (video_fd < 0) {
        printf("Error: Failed to open %s - is video streaming active?\n", VIDEO_DEVICE);
        return 1;
    }

    struct v4l2_ext_controls ctrls = {0};
    struct v4l2_ext_control ctrl  = {0};
    ctrls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
    ctrls.count = 1;
    ctrls.controls = &ctrl;
    ctrl.id = V4L2_CID_FOCUS_ABSOLUTE;

    if (strcmp(argv[1], "current") == 0) {
        if (ioctl(video_fd, VIDIOC_G_EXT_CTRLS, &ctrls) == 0) {
            printf("Focus position: %ld\n", ctrl.value);
        } else {
            printf("Failed to get focus position (errno=%d)\n", errno);
        }
        close(video_fd);
        return 0;
    }

    long pos = strtol(argv[1], NULL, 0);
    if (pos < 0) pos = 0;
    if (pos > 1023) pos = 1023; // DW9714 typical max
    ctrl.value = (int32_t)pos;

    if (ioctl(video_fd, VIDIOC_S_EXT_CTRLS, &ctrls) == 0) {
        printf("Set focus position to %ld\n", pos);
    } else {
        printf("Failed to set focus position (errno=%d)\n", errno);
    }

    close(video_fd);
    return 0;
}

/**
 * @brief Set/Get exposure in microseconds with proper rounding to device step
 * Usage:
 *   cam_exposure <us>   # sets exposure (rounded to valid step)
 *   cam_exposure current
 */
static int cmd_cam_exposure(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage:\n");
        printf("  cam_exposure <microseconds>\n");
        printf("  cam_exposure current\n");
        return 0;
    }

    int fd = open(VIDEO_DEVICE, O_RDWR);
    if (fd < 0) {
        printf("Error: Failed to open %s\n", VIDEO_DEVICE);
        return 1;
    }

    struct v4l2_ext_controls ctrls = {0};
    struct v4l2_ext_control  ctrl  = {0};
    ctrls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
    ctrls.count = 1;
    ctrls.controls = &ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;

    if (strcmp(argv[1], "current") == 0) {
        if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls) == 0) {
            printf("Exposure: %ld us\n", ctrl.value);
        } else {
            printf("Failed to get exposure (errno=%d)\n", errno);
        }
        close(fd);
        return 0;
    }

    long us = strtol(argv[1], NULL, 0);
    int32_t min=0, max=0, step=1;
    if (get_ctrl_range(fd, V4L2_CID_EXPOSURE_ABSOLUTE, &min, &max, &step) != 0) {
        printf("Failed to query exposure range\n");
        close(fd);
        return 1;
    }
    if (step <= 0) step = 1;
    // Some drivers validate as (value % step == 0), ignoring the min offset.
    // To satisfy both patterns, round to nearest multiple of step, then clamp to [min,max].
    long k = (us + step/2) / step;           // nearest multiple of step
    long us_rounded = k * step;
    if (us_rounded < min) {
        us_rounded = ((min + step - 1) / step) * step; // round up to first valid multiple
    }
    if (us_rounded > max) {
        us_rounded = (max / step) * step;             // round down to last valid multiple
    }

    ctrl.value = (int32_t)us_rounded;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) == 0) {
        printf("Set exposure to %ld us (requested %ld, min=%d max=%d step=%d)\n",
               us_rounded, us, (int)min, (int)max, (int)step);
    } else {
        printf("Failed to set exposure (errno=%d)\n", errno);
    }

    close(fd);
    return 0;
}

/**
 * @brief Set/Get analog gain with step-aware rounding
 * Usage:
 *   cam_gain <value> | current
 */
static int cmd_cam_gain(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: cam_gain <value>|current\n");
        return 0;
    }
    int fd = open(VIDEO_DEVICE, O_RDWR);
    if (fd < 0) {
        printf("Error: Failed to open %s\n", VIDEO_DEVICE);
        return 1;
    }
    struct v4l2_ext_controls ctrls = {0};
    struct v4l2_ext_control  ctrl  = {0};
    ctrls.ctrl_class = V4L2_CTRL_CLASS_USER;
    ctrls.count = 1;
    ctrls.controls = &ctrl;
    ctrl.id = V4L2_CID_GAIN;

    if (strcmp(argv[1], "current") == 0) {
        if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls) == 0) {
            printf("Gain: %ld\n", ctrl.value);
        } else {
            printf("Failed to get gain (errno=%d)\n", errno);
        }
        close(fd);
        return 0;
    }

    long val = strtol(argv[1], NULL, 0);
    int32_t min=0, max=0, step=1;
    if (get_ctrl_range(fd, V4L2_CID_GAIN, &min, &max, &step) != 0) {
        printf("Failed to query gain range\n");
        close(fd);
        return 1;
    }
    if (step <= 0) step = 1;
    if (val < min) val = min;
    if (val > max) val = max;
    long k = (val - min + step/2) / step;
    long val_rounded = min + k * step;
    ctrl.value = (int32_t)val_rounded;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) == 0) {
        printf("Set gain to %ld (rounded from %ld, min=%d step=%d)\n",
               val_rounded, val, (int)min, (int)step);
    } else {
        printf("Failed to set gain (errno=%d)\n", errno);
    }

    close(fd);
    return 0;
}

/**
 * @brief Lock/unlock 3A (AE/AWB/AF) via V4L2_CID_3A_LOCK
 * Usage:
 *   cam_3a_lock ae on|off
 *   cam_3a_lock awb on|off
 *   cam_3a_lock af on|off
 *   cam_3a_lock status
 */
static int cmd_cam_3a_lock(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage:\n");
        printf("  cam_3a_lock ae|awb|af on|off\n");
        printf("  cam_3a_lock status\n");
        return 0;
    }

    int fd = open(VIDEO_DEVICE, O_RDWR);
    if (fd < 0) {
        printf("Error: Failed to open %s\n", VIDEO_DEVICE);
        return 1;
    }

    struct v4l2_ext_controls ctrls = {0};
    struct v4l2_ext_control  ctrl  = {0};
    ctrls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
    ctrls.count = 1;
    ctrls.controls = &ctrl;
    ctrl.id = V4L2_CID_3A_LOCK;

    // Read current state
    if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls) != 0) {
        printf("Failed to get 3A lock (errno=%d)\n", errno);
        close(fd);
        return 1;
    }
    int32_t cur = ctrl.value;

    if (strcmp(argv[1], "status") == 0) {
        printf("3A lock bits: 0x%02x (AWB=1, AE=2, AF=4)\n", (unsigned)cur);
        close(fd);
        return 0;
    }

    if (argc < 3) {
        printf("Invalid usage. Example: cam_3a_lock ae off\n");
        close(fd);
        return 1;
    }

    int bit = 0;
    if (strcmp(argv[1], "awb") == 0) bit = 1;
    else if (strcmp(argv[1], "ae") == 0) bit = 2;
    else if (strcmp(argv[1], "af") == 0) bit = 4;
    else {
        printf("Invalid component: %s (use ae|awb|af)\n", argv[1]);
        close(fd);
        return 1;
    }

    if (strcmp(argv[2], "on") == 0) cur |= bit;
    else if (strcmp(argv[2], "off") == 0) cur &= ~bit;
    else {
        printf("Invalid state: %s (use on|off)\n", argv[2]);
        close(fd);
        return 1;
    }

    ctrl.value = cur;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) == 0) {
        printf("Set 3A lock to 0x%02x\n", (unsigned)cur);
    } else {
        printf("Failed to set 3A lock (errno=%d)\n", errno);
    }

    close(fd);
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
            .command = "cam_exposure",
            .help = "Set/Get exposure in microseconds (rounded to valid step)",
            .hint = NULL,
            .func = &cmd_cam_exposure,
        },
        {
            .command = "cam_gain",
            .help = "Set/Get analog gain (rounded to valid step)",
            .hint = NULL,
            .func = &cmd_cam_gain,
        },
        {
            .command = "cam_3a_lock",
            .help = "Lock/unlock AE/AWB/AF (bitmask via V4L2_CID_3A_LOCK)",
            .hint = NULL,
            .func = &cmd_cam_3a_lock,
        },
        {
            .command = "cam_focus",
            .help = "Set/Get motor absolute focus position (V4L2_CID_FOCUS_ABSOLUTE)",
            .hint = NULL,
            .func = &cmd_cam_focus,
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
    ESP_LOGI(TAG, "Available: cam_status, cam_focus, hw_awb");
    ESP_LOGI(TAG, "Use hw_* commands for direct hardware register control");
    ESP_LOGI(TAG, "Use hw_registers_dump for complete hardware status");
}
