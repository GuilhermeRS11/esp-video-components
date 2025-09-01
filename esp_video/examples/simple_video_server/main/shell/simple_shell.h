/*
 * Simple shell wrapper for the video server example
 * This file provides the main shell interface including camera controls
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and start the simple shell console
 * This will register all available commands and start the console
 */
void simple_shell_start(void);

/**
 * @brief Stop the simple shell console
 */
void simple_shell_stop(void);

#ifdef __cplusplus
}
#endif
