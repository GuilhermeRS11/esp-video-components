/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_cam_sensor_types.h"

#define IMX708_SCCB_ADDR   0x1A

/**
 * @brief Power on camera sensor device and detect the device connected to the designated sccb bus.
 *
 * @param[in] config Configuration related to device power-on and detection.
 * @return
 *      - Camera device handle on success, otherwise, failed.
 */
esp_cam_sensor_device_t *imx708_detect(esp_cam_sensor_config_t *config);

/**
 * @brief Set global IMX708 device reference for hardware register access
 * 
 * @param[in] device Detected IMX708 device handle
 */
void imx708_set_global_device(esp_cam_sensor_device_t *device);

/**
 * @brief Write to IMX708 hardware register
 * 
 * @param[in] reg Register address
 * @param[in] value Value to write
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imx708_hw_write_register(uint16_t reg, uint8_t value);

/**
 * @brief Read from IMX708 hardware register
 * 
 * @param[in] reg Register address  
 * @param[out] value Pointer to store read value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imx708_hw_read_register(uint16_t reg, uint8_t *value);

#ifdef __cplusplus
}
#endif
