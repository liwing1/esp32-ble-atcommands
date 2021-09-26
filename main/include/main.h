#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

typedef struct {
    char header[3];
    char send_addr[2];
    char dest_addr[2];
    char payload[5];
    char endWord[2];
}ble_msg_t;

static void uart_init( void );
static void ble_mesh_config( void );
uint8_t ble_mesh_parse(const char* data);
static void uart_event_task(void *pvParameters);