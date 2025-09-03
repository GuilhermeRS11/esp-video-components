#ifndef IMX708_HARDWARE_CONTROLS_H
#define IMX708_HARDWARE_CONTROLS_H

// Auto White Balance (AWB) Controls
#define IMX708_REG_AWB_ENABLE           0x3200  // Enable/disable AWB algorithm
#define IMX708_REG_AWB_CONTROL          0x3201  // AWB control enable
#define IMX708_REG_AWB_GAIN_CONTROL     0x3202  // AWB gain control enable
#define IMX708_REG_AWB_SPEED            0x3203  // Convergence speed (0x10-0xFF)
#define IMX708_REG_AWB_STABILITY        0x3204  // Stability threshold
#define IMX708_REG_AWB_SENSITIVITY      0x3205  // Low-light sensitivity
#define IMX708_REG_AWB_GAIN_STEP        0x3206  // Gain step size
#define IMX708_REG_AWB_RED_MIN          0x3207  // Red gain minimum
#define IMX708_REG_AWB_RED_MAX          0x3208  // Red gain maximum
#define IMX708_REG_AWB_BLUE_MIN         0x3209  // Blue gain minimum
#define IMX708_REG_AWB_BLUE_MAX         0x320A  // Blue gain maximum

// Auto Exposure (AE) Controls
#define IMX708_REG_AE_ENABLE            0x3100  // Enable/disable AE algorithm
#define IMX708_REG_AE_TARGET            0x3101  // Target brightness (0-255)
#define IMX708_REG_AE_SPEED             0x3102  // AE convergence speed
#define IMX708_REG_AE_TOLERANCE         0x3103  // AE stability tolerance
#define IMX708_REG_AE_MIN_GAIN          0x3104  // Minimum gain limit
#define IMX708_REG_AE_MAX_GAIN          0x3105  // Maximum gain limit

// Auto Gain Control (AGC)
#define IMX708_REG_AGC_ENABLE           0x3110  // Enable/disable AGC algorithm

// Noise Reduction Controls
#define IMX708_REG_NR_ENABLE            0x3500  // Enable noise reduction
#define IMX708_REG_NR_STRENGTH          0x3501  // NR strength (0x10-0x80)
#define IMX708_REG_NR_THRESHOLD         0x3502  // Noise detection threshold
#define IMX708_REG_NR_TEMPORAL          0x3503  // Temporal NR (between frames)
#define IMX708_REG_NR_SPATIAL           0x3504  // Spatial NR (within frame)

// Enhancement Controls
#define IMX708_REG_CONTRAST_ENHANCE     0x3602  // Contrast enhancement
#define IMX708_REG_SHADOW_BOOST         0x3603  // Shadow detail boost
#define IMX708_REG_HIGHLIGHT_COMP       0x3604  // Highlight compression

// Low-Light Optimization
#define IMX708_REG_LOWLIGHT_RED_BIAS    0x320B  // Red bias in low light
#define IMX708_REG_LOWLIGHT_BLUE_BIAS   0x320C  // Blue bias in low light
#define IMX708_REG_LOWLIGHT_GREEN_SUPP  0x320D  // Green suppression

// Manual Color Balance (from previous implementation)
#define IMX708_REG_COLOR_BALANCE_RED    0x0B90  // Red gain register
#define IMX708_REG_COLOR_BALANCE_BLUE   0x0B92  // Blue gain register

#endif // IMX708_HARDWARE_CONTROLS_H
