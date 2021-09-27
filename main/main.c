/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "main.h"

static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_1
#define PATTERN_CHR_NUM    (1)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)
static QueueHandle_t uart1_queue;


static const char *cmd_ble_default = "AT+DEFAULT\r\n";
static const char *cmd_ble_role5 = "AT+ROLE5\r\n";
static const char *cmd_ble_netid = "AT+NETID1133\r\n";
static const char *cmd_ble_maddr = "AT+MADDRFF00\r\n";
static const char *cmd_ble_reset = "AT+RESET\r\n";
static const char *cmd_ble_sleep = "AT+SLEEP2\r\n";

static const ble_msg_t ble_msg_example ={
    .header[0] = 0xf1,
    .header[1] = 0xdd,
    .header[2] = 0x07,

    .send_addr[0] = 0x00,
    .send_addr[1] = 0x88,

    .dest_addr[0] = 0xFF,
    .dest_addr[1] = 0x00,

    .payload[0] = 0x01,
    .payload[1] = 0x7f,
    .payload[2] = 0x7f,
    .payload[3] = 0x80,
    .payload[4] = 0x1e,

    .endWord[0] = 0x0d,
    .endWord[1] = 0x0a,
};


static void uart_init(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        // .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart1_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '\n', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);
}


static void ble_mesh_config(void)
{
    uart_write_bytes(EX_UART_NUM, cmd_ble_default, strlen(cmd_ble_default));
    uart_write_bytes(EX_UART_NUM, cmd_ble_role5, strlen(cmd_ble_role5));
    uart_write_bytes(EX_UART_NUM, cmd_ble_netid, strlen(cmd_ble_netid));
    uart_write_bytes(EX_UART_NUM, cmd_ble_maddr, strlen(cmd_ble_maddr));
    uart_write_bytes(EX_UART_NUM, cmd_ble_reset, strlen(cmd_ble_reset));
    uart_write_bytes(EX_UART_NUM, cmd_ble_sleep, strlen(cmd_ble_sleep));

    uart_write_bytes(EX_UART_NUM, (char*)&ble_msg_example, 14);
}


uint8_t ble_mesh_parse(const char* data)
{
    char* token = strstr(data, "\xf1\xdd\x07");
    uint8_t msg_sens_cmd = 0;
    uint32_t msg_n_access = 0;
    uint8_t msg_battery = 0;

    if( token != NULL)
    {
        ble_msg_t* p_rx_data = (ble_msg_t*)data;

        if(p_rx_data->endWord[0] == 0x0d)
        {
            printf("MsgBLE:\r\n");

            msg_sens_cmd = p_rx_data->payload[0] ;

            msg_n_access = (p_rx_data->payload[1]<<16 | p_rx_data->payload[2]<<8 | p_rx_data->payload[3]) - (0x7F7F7F);

            msg_battery = (p_rx_data->payload[4]);

            printf("%d\r\n%d\r\n%d\r\n", msg_sens_cmd, msg_n_access, msg_battery);
        
        }

        switch( msg_sens_cmd )
        {
            case SENSOR_CMD_ACCES:
            printf("chegou %d pessoas!!\r\n". msg_n_access);
            break;

            case SENSOR_CMD_EGRESS:
            printf("saiu gentes!!\r\n");
            break;

            case SENSOR_CMD_OBSTR:
            printf("obstruido!!\r\n");
            break;
            
            case SENSOR_CMD_PROVI:
            printf("provisionamento!!\r\n");
            break;
        }
    }

    return 0;
}


static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart1_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[DATA EVT]:");
                    uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    } else {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : 0x%x", *pat);
                        ble_mesh_parse((char*)dtmp);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{
    uart_init();
    ble_mesh_config();

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}
