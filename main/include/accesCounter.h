#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
// #include "Iothub_Mensagens.h"

#define _MODE_MAIN_

typedef enum{
    SENSOR_CMD_B_A = 1,
    SENSOR_CMD_A_B,
    SENSOR_CMD_OBSTR,
    SENSOR_CMD_PROVI
}ble_cmd_t;

typedef struct{
    char header[3];
    char send_addr[2];
    char dest_addr[2];
    char payload[5];
}ble_msg_t;

void pins_init(void);
void uart_init( void );
void ble_config( void );
uint8_t ble_mesh_parse(const char* data);
void uart_event_task(void *pvParameters);
void iniciar_simulador(void *pvParameters);