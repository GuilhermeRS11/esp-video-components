/**
 * @file video_debug.h
 * @brief Debug utilities for video streaming
 */

#ifndef VIDEO_DEBUG_H
#define VIDEO_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register shell activity for debugging video freezes
 * 
 * This function should be called from shell command handlers to help
 * correlate video streaming issues with shell command execution.
 */
void register_shell_activity(void);

#ifdef __cplusplus
}
#endif

#endif /* VIDEO_DEBUG_H */
