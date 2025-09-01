/*
 * Camera control shell header
 * Provides shell command interface for dynamic camera parameter control
 * via ESP-IDF sensor API
 */

#pragma once

#include "esp_cam_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register all camera control commands with the console
 * 
 * This function registers the following camera control commands:
 * - cam_brightness [<level>]       : Get/set brightness (-2 to +2)
 * - cam_contrast [<level>]         : Get/set contrast (-2 to +2) 
 * - cam_saturation [<level>]       : Get/set saturation (-2 to +2)
 * - cam_status                     : Display current camera settings
 * 
 * Note: This implementation uses V4L2 ioctl interface for camera control.
 * All commands work while video streaming is active.
 */
void camera_shell_register_commands(void);

#ifdef __cplusplus
}
#endif
