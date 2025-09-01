/*
 * Simple shell implementation for the video server example
 * Integrates camera controls and other system commands
 */

#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_chip_info.h"
#include "spi_flash_mmap.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "simple_shell.h"
#include "camera_shell.h"

static const char* TAG = "simple_shell";

#define PROMPT_STR "esp32p4"

static bool shell_running = false;

/**
 * @brief Initialize console REPL environment
 */
static void initialize_console(void)
{
    /* Drain stdout before reconfiguring it */
    fflush(stdout);
    fsync(fileno(stdout));

    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

    /* Configure UART. Note that REF_TICK is used so that the baud rate remains
     * correct while APB frequency is changing in light sleep mode.
     */
    const uart_config_t uart_config = {
        .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
                                         256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config));

    /* Tell VFS to use UART driver */
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    /* Initialize the console */
    esp_console_config_t console_config = {
        .max_cmdline_args = 8,
        .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
        .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(100);

    /* Load command history from filesystem */
    linenoiseHistoryLoad("/spiffs/history.txt");
}

/**
 * @brief System restart command
 */
static int restart_cmd(int argc, char **argv)
{
    ESP_LOGI(TAG, "Restarting system...");
    esp_restart();
}

/**
 * @brief System info command
 */
static int info_cmd(int argc, char **argv)
{
    printf("\n🔧 ESP32-P4 Video Server System Info:\n");
    printf("=====================================\n");
    
    printf("Chip: %s\n", CONFIG_IDF_TARGET);
    printf("Free heap: %lu bytes\n", esp_get_free_heap_size());
    printf("Minimum free heap: %lu bytes\n", esp_get_minimum_free_heap_size());
    
    printf("\nCamera Shell Commands Available:\n");
    printf("  exposure     - Control exposure time\n");
    printf("  gain         - Control sensor gain\n");
    printf("  brightness   - Adjust brightness\n");
    printf("  contrast     - Adjust contrast\n");
    printf("  saturation   - Adjust saturation\n");
    printf("  flip         - Set horizontal/vertical flip\n");
    printf("  cam_status   - Show current camera settings\n");
    printf("  quick_setup  - Apply preset configurations\n");
    printf("\nType 'help' for more commands\n\n");
    
    return 0;
}

/**
 * @brief Help command
 */
static int help_cmd(int argc, char **argv)
{
    printf("\nAvailable Commands:\n");
    printf("=====================\n");
    printf("System Commands:\n");
    printf("  help         - Show this help\n");
    printf("  info         - System information\n");
    printf("  restart      - Restart the system\n");
    printf("  free         - Show memory info\n");
    printf("  version      - Show IDF version\n");
    printf("\nCamera Commands:\n");
    printf("  cam_brightness   - Get/set brightness (-2 to +2)\n");
    printf("  cam_contrast     - Get/set contrast (-2 to +2)\n");
    printf("  cam_saturation   - Get/set saturation (-2 to +2)\n");
    printf("  cam_status       - Display current camera settings\n");
    printf("\nExamples:\n");
    printf("  cam_brightness       # Show current brightness\n");
    printf("  cam_brightness 1     # Set brightness to 1\n");
    printf("  cam_contrast -1      # Set contrast to -1\n");
    printf("  cam_status           # Show all current settings\n");
    printf("\n");
    
    return 0;
}

/**
 * @brief Register basic system commands
 */
static void register_system_commands(void)
{
    const esp_console_cmd_t commands[] = {
        {
            .command = "restart",
            .help = "Restart the system",
            .hint = NULL,
            .func = &restart_cmd,
        },
        {
            .command = "info",
            .help = "Show system information and available camera commands",
            .hint = NULL,
            .func = &info_cmd,
        },
        {
            .command = "help",
            .help = "Show help for all commands",
            .hint = NULL,
            .func = &help_cmd,
        }
    };
    
    for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&commands[i]));
    }
}

/**
 * @brief Console task - handles the REPL loop
 */
static void console_task(void *args)
{
    const char* prompt = LOG_COLOR_I PROMPT_STR "> " LOG_RESET_COLOR;

    printf("\n");
    printf("🎥 ESP32-P4 IMX708 Video Server with Camera Control Shell\n");
    printf("=========================================================\n");
    printf("Type 'help' for available commands\n");
    printf("Type 'info' for system information\n");
    printf("Type 'cam_status' for current camera settings\n\n");

    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    while (shell_running) {
        char* line = linenoise(prompt);
        if (line == NULL) { /* Ignore empty lines */
            continue;
        }
        /* Add the command to the history */
        linenoiseHistoryAdd(line);
        /* Save command history to filesystem */
        linenoiseHistorySave("/spiffs/history.txt");

        /* Try to run the command */
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command: '%s'\n", line);
            printf("Type 'help' for available commands\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
        /* linenoise allocates line buffer on the heap, so need to free it */
        linenoiseFree(line);
    }
    
    ESP_LOGI(TAG, "Console task ending");
    vTaskDelete(NULL);
}

/**
 * @brief Initialize and start the simple shell console
 */
void simple_shell_start(void)
{
    if (shell_running) {
        ESP_LOGW(TAG, "Shell already running");
        return;
    }
    
    ESP_LOGI(TAG, "Initializing console...");
    
    // Initialize console
    initialize_console();
    
    // Register ESP system commands (like free, version, etc.)
    esp_console_register_help_command();
    
    // Register our system commands
    register_system_commands();
    
    // Register camera control commands
    camera_shell_register_commands();
    
    shell_running = true;
    
    // Start console task
    xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "📟 Shell console started successfully!");
}

/**
 * @brief Stop the simple shell console
 */
void simple_shell_stop(void)
{
    if (!shell_running) {
        ESP_LOGW(TAG, "Shell not running");
        return;
    }
    
    shell_running = false;
    ESP_LOGI(TAG, "Shell console stopped");
}
