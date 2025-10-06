/*
 * Shell task implementation for video server example
 * This file provides the shell task that integrates camera control
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "shell.h"
#include "shell/simple_shell.h"

static const char* TAG = "shell";

/**
 * @brief Shell task for camera control and system commands
 * @param pvParameters Task parameters (unused)
 */
void shell_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting shell task...");
    
    // Small delay to ensure other systems are initialized
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Camera control via V4L2 interface will be available when video streaming starts");
    
    // Start the shell console
    simple_shell_start();
    
    ESP_LOGI(TAG, "Shell task started successfully");
    
    // Keep task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Sleep for 10 seconds
    }
}
