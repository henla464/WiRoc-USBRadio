/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "cp210x_usb.hpp"
#include "usb/usb_host.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"

using namespace esp_usb;

// Change these values to match your needs
#define SPORTIDENT_BAUDRATE     (38400)
#define SPORTIDENT_STOP_BITS    (0)      // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
#define SPORTIDENT_PARITY       (0)      // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
#define SPORTIDENT_DATA_BITS    (8)

static const char *TAG = "SportIdent Bluetooth Adapter";

static SemaphoreHandle_t device_disconnected_sem;

static void handle_rx(uint8_t *data, size_t data_len, void *arg)
{
    printf("%.*s", data_len, data);
     // Write data to UART.
    ESP_LOGI(TAG, "Data length: %d", data_len);
    ESP_LOGI(TAG, "first char: %d", data[0]);
    uart_write_bytes(UART_NUM_1, (const char*)data, data_len);
}

static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
        case CDC_ACM_HOST_ERROR:
            ESP_LOGE(TAG, "CDC-ACM error has occurred, err_no = %d", event->data.error);
            break;
        case CDC_ACM_HOST_DEVICE_DISCONNECTED:
            ESP_LOGI(TAG, "Device suddenly disconnected");
            xSemaphoreGive(device_disconnected_sem);
            break;
        case CDC_ACM_HOST_SERIAL_STATE:
            ESP_LOGI(TAG, "serial state notif 0x%04X", event->data.serial_state.val);
            break;
        case CDC_ACM_HOST_NETWORK_CONNECTION:
        default: break;
    }
}

void usb_lib_task(void *arg)
{
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
        }
    }
}



// UART

uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB
};

void uart_init() {
     ESP_LOGI(TAG, "Installing uart 11");

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    //QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    //ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, uart_buffer_size, 0, 0, NULL, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
   
    // Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void button_init() {
    ESP_LOGI(TAG, "Button init");
    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_1, GPIO_PULLUP_ONLY);
}

bool was_button_pressed() {
    if (gpio_get_level(GPIO_NUM_1) == 0)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        if (gpio_get_level(GPIO_NUM_1) == 1)
        {
            ESP_LOGI(TAG, "Button pressed and released!");
            return true;
        }
    }
    return false;
}

void send_bt_name(const char* bt_name) {
    ESP_LOGI(TAG, "Set BT name: %s", bt_name);
    char at_command[30];
    snprintf(at_command, sizeof(at_command), "%s%s\n", "AT+NAME", bt_name);
    uart_write_bytes(UART_NUM_1, at_command, strlen(at_command));
    vTaskDelay(500 / portTICK_PERIOD_MS);

    uint8_t replyFromHC02[128];
    size_t length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_1, (size_t*)&length));
    ESP_LOGI(TAG, "Reply length %i", length);
    length = uart_read_bytes(UART_NUM_1, replyFromHC02, length, 100);
    replyFromHC02[length] = '\0';
    ESP_LOGI(TAG, "Reply %s", replyFromHC02);
}

void configure_bluetooth_name(void) {
    ESP_LOGI(TAG, "%s", "Start bluetooth name configuration");
    
    struct timeval current_time;
    gettimeofday(&current_time, NULL);

    int last_key_press = 0;
    int no_of_pressed = 0;
    for(;;) {

        if (was_button_pressed()) {
            gettimeofday(&current_time, NULL);
            last_key_press = current_time.tv_sec;
            no_of_pressed++;
        }
        gettimeofday(&current_time, NULL);
        int current_time_seconds = current_time.tv_sec;
        if (last_key_press + 3 <= current_time_seconds) {
            ESP_LOGI(TAG, "3 seconds since last button press");
            break;
        }
    }

    if (no_of_pressed >= 1) {
        char bt_name[11];
        snprintf(bt_name, sizeof(bt_name), "%s%i", "WiRocBT", no_of_pressed);
        send_bt_name(bt_name);
    }

    ESP_LOGI(TAG, "%s", "End bluetooth name configuration");
}




extern "C" void app_main(void)
{
    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    uart_init();
    button_init();
    configure_bluetooth_name();

    //Install USB Host driver. Should only be called once in entire application
    ESP_LOGI(TAG, "Installing USB Host");
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Create a task that will handle USB library events
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Installing CDC-ACM driver");
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    while (true) {
        const cdc_acm_host_device_config_t dev_config = {
            .connection_timeout_ms = 10000,
            .out_buffer_size = 64,
            .event_cb = handle_event,
            .data_cb = handle_rx,
            .user_arg = NULL,
        };

        CP210x *vcp;
        try {
            ESP_LOGI(TAG, "Opening CP210X device");
            vcp = CP210x::open_cp210x(CP210x_SportIdent_PID, &dev_config);
        }
        catch (esp_err_t err) {
            ESP_LOGE(TAG, "The required device was not opened.\nExiting...");
            return;
        }

        ESP_LOGI(TAG, "Setting up line coding");
        cdc_acm_line_coding_t line_coding = {
            .dwDTERate = SPORTIDENT_BAUDRATE,
            .bCharFormat = SPORTIDENT_STOP_BITS,
            .bParityType = SPORTIDENT_PARITY,
            .bDataBits = SPORTIDENT_DATA_BITS,
        };
        ESP_ERROR_CHECK(vcp->line_coding_set(&line_coding));

        /*
        Now the USB-to-UART converter is configured and receiving data.
        You can use standard CDC-ACM API to interact with it. E.g.

        ESP_ERROR_CHECK(vcp->set_control_line_state(false, true));
        ESP_ERROR_CHECK(vcp->tx_blocking((uint8_t *)"Test string", 12));
        */

        // We are done. Wait for device disconnection and start over
        xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
        delete vcp;
    }
}
