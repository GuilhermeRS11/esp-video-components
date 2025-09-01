/*
 * Shell interface for video server example
 * This file provides the shell task entry point
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Shell task for camera control and system commands
 * @param pvParameters Task parameters (unused)
 */
void shell_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
