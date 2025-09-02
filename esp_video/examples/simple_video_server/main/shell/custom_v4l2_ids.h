/*
 * Custom V4L2 Control IDs for IMX708 specific controls
 * These IDs are used to extend V4L2 standard controls with sensor-specific parameters
 */

#pragma once

#include "linux/videodev2.h"

#ifdef __cplusplus
extern "C" {
#endif

// Note: V4L2_CID_RED_BALANCE and V4L2_CID_BLUE_BALANCE are already defined in standard V4L2
// so we don't need to redefine them here. They map to standard color balance controls.

#ifdef __cplusplus
}
#endif
