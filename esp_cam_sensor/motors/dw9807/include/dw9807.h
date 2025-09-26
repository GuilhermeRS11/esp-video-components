/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_cam_motor_types.h"

/* 7-bit I2C address (0x18 >> 1) */
#define DW9807_SCCB_ADDR           0x0c

/* Registers */
#define DW9807_CTL_ADDR            0x02
#define DW9807_MSB_ADDR            0x03
#define DW9807_LSB_ADDR            0x04
#define DW9807_STATUS_ADDR         0x05
#define DW9807_MODE_ADDR           0x06
#define DW9807_RESONANCE_ADDR      0x07

#define DW9807_WAIT_STABLE_TIME    12    // ms

esp_cam_motor_device_t *dw9807_detect(esp_cam_motor_config_t *config);

#ifdef __cplusplus
}
#endif

