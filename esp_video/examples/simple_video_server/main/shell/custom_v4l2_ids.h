/*
 * Custom V4L2 Control IDs for IMX708 specific controls
 * These IDs are used to extend V4L2 standard controls with sensor-specific parameters
 */

#pragma once

#include "linux/videodev2.h"

#ifdef __cplusplus
extern "C" {
#endif

// Custom V4L2 IDs for IMX708 color balance controls
// Using V4L2_CID_PRIVATE_BASE as starting point for custom controls
#define V4L2_CID_RED_BALANCE    (V4L2_CID_PRIVATE_BASE + 0x100)    /*!< Red color balance gain control */
#define V4L2_CID_BLUE_BALANCE   (V4L2_CID_PRIVATE_BASE + 0x101)    /*!< Blue color balance gain control */

#ifdef __cplusplus
}
#endif
