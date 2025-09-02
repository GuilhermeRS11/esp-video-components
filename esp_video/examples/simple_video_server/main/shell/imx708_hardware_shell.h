/*
 * IMX708 Hardware Register Control Shell Commands Header
 * Declaration for direct hardware register access commands
 * ONLY commands that perform direct hardware register reads/writes
 */

#ifndef IMX708_HARDWARE_SHELL_H
#define IMX708_HARDWARE_SHELL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register all IMX708 hardware control shell commands
 * 
 * This function registers the following commands:
 * - hw_awb_speed: Control AWB convergence speed
 * - hw_ae_target: Set Auto Exposure target brightness
 * - hw_ae_enable: Enable/disable hardware Auto Exposure
 * - hw_noise_reduction: Control hardware noise reduction
 * - hw_color_balance: Direct hardware color balance control
 * - hw_lowlight_opt: Low-light optimization parameters
 * - hw_registers_dump: Dump all hardware register values
 */
void imx708_hardware_shell_register_commands(void);

#ifdef __cplusplus
}
#endif

#endif /* IMX708_HARDWARE_SHELL_H */
