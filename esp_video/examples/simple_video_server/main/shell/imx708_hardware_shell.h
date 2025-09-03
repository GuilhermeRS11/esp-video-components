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
 * @brief Register IMX708 hardware control shell commands - Clean Version
 * 
 * This function registers only the confirmed working commands:
 * - sensor_exposure_direct: Direct sensor exposure control (ONLY WORKING DIRECT COMMAND)
 * - hw_registers_dump: Dump all hardware register values for diagnostics
 * 
 * All other direct sensor commands were removed because they didn't work.
 * The original color stability issue was caused by ESP32-P4 IPA, not sensor registers.
 * For color control, use V4L2 commands like cam_red_balance and cam_blue_balance.
 */
void imx708_hardware_shell_register_commands(void);

#ifdef __cplusplus
}
#endif

#endif /* IMX708_HARDWARE_SHELL_H */
