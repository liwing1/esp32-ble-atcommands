/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "accesCounter.h"

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

#define EX_UART_NUM UART_NUM_2
#define PATTERN_CHR_NUM    (1)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

#define TXD_PIN (GPIO_NUM_19)
#define RXD_PIN (GPIO_NUM_18)

#define GPIO_BLE_VSOURCE        GPIO_NUM_23
#define GPIO_OUTPUT_PIN_SEL     (1ULL<<GPIO_BLE_VSOURCE) 

static QueueHandle_t uart1_queue;

static const char *cmd_ble_default = "AT+DEFAULT\r\n";
static const char *cmd_ble_role5 = "AT+ROLE5\r\n";
static const char *cmd_ble_netid = "AT+NETID1133\r\n";
static const char *cmd_ble_maddr = "AT+MADDRFF00\r\n";
static const char *cmd_ble_reset = "AT+RESET\r\n";
// static const char *cmd_ble_sleep = "AT+SLEEP2\r\n";

static const char *cmd_ble_provisioning = "\x41\x54\x2b\x4d\x45\x53\x48\x41\x00\x02\xE3\xF1\x03\x00\x77\x0D\x0A";

static uint16_t nextSensorAddr = 0;

void pins_init(void)
{
    gpio_config_t io_config = {
        .intr_type      =   GPIO_INTR_DISABLE,
        .mode           =   GPIO_MODE_OUTPUT,
        .pin_bit_mask   =   GPIO_OUTPUT_PIN_SEL,
        .pull_down_en   =   GPIO_PULLDOWN_DISABLE,
        .pull_up_en     =   GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_config);

    gpio_set_level(GPIO_NUM_23, pdTRUE);
}


void uart_init(void)
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


void ble_config(void)
{
    uart_write_bytes(EX_UART_NUM, cmd_ble_default, strlen(cmd_ble_default));
    vTaskDelay(pdMS_TO_TICKS(1000));

    uart_write_bytes(EX_UART_NUM, cmd_ble_role5, strlen(cmd_ble_role5));
    vTaskDelay(pdMS_TO_TICKS(1000));

    uart_write_bytes(EX_UART_NUM, cmd_ble_netid, strlen(cmd_ble_netid));
    vTaskDelay(pdMS_TO_TICKS(1000));

    uart_write_bytes(EX_UART_NUM, cmd_ble_maddr, strlen(cmd_ble_maddr));
    vTaskDelay(pdMS_TO_TICKS(1000));

    uart_write_bytes(EX_UART_NUM, cmd_ble_reset, strlen(cmd_ble_reset));
}


uint8_t ble_mesh_parse(const char* data)
{
    char* tokenHeader = strstr(data, "\xf1\xdd\x07");
    uint16_t msg_address_sender = 0;
    uint8_t a_b = 0;
    uint8_t b_a = 0;
    ble_cmd_t msg_sens_cmd = 0;
    uint8_t msg_battery = 0;
    uint8_t obstrucao = 0;

    if( tokenHeader != NULL)
    {
        ble_msg_t* p_rx_data = (ble_msg_t*)data;

        printf("MsgBLE:\r\n");

        msg_sens_cmd = (ble_cmd_t)p_rx_data->payload[0] ;

        msg_address_sender = p_rx_data->send_addr;

        msg_battery = (p_rx_data->payload[4]);

        printf("%d\r\n%d\r\n", msg_sens_cmd, msg_battery);
    

        switch( msg_sens_cmd )
        {
            case SENSOR_CMD_B_A:
            {
                b_a = (p_rx_data->payload[1]<<16 | p_rx_data->payload[2]<<8 | p_rx_data->payload[3]) - (0x7F7F7F);
                printf("chegou %d pessoas!!\r\n", b_a);
                a_b = 0;
            }
            break;

            case SENSOR_CMD_A_B:
            {
                a_b = (p_rx_data->payload[1]<<16 | p_rx_data->payload[2]<<8 | p_rx_data->payload[3]) - (0x7F7F7F);
                printf("saiu %d pessoas!!\r\n", a_b);
                b_a = 0;
            }
            break;

            case SENSOR_CMD_OBSTR:
            {
                a_b = 0;
                b_a = 0;
                obstrucao = 1;
                printf("obstruido!!\r\n");
            }
            break;
            
            case SENSOR_CMD_PROVI:
            {
                printf("provisionamento!!\r\n");
                uart_write_bytes(EX_UART_NUM, cmd_ble_provisioning, strlen(cmd_ble_provisioning));
                
            }
            break;
        }

        #ifndef _MODE_MAIN_
        Enviar_Mensagem_Iothub(
            a_b, 
            b_a, 
            0, 
            msg_battery, 
            obstrucao, 
            msg_address_sender
        );
        #endif
    }

    return 0;
}

void iniciar_simulador(void *pvParameters)
{
    while (1)
    {
        // char *simu_data = "\xf1\xdd\x07\x00\x88\xFF\x00\x01\x7F\x7F\x80\x1E\x0d\x0a";
        // ble_mesh_parse(simu_data);

        uart_write_bytes(EX_UART_NUM, "at\r\n", strlen("at\r\n"));
        vTaskDelay(pdMS_TO_TICKS(1000));

        vTaskDelay(5000/portTICK_PERIOD_MS);
    }
}


void uart_event_task(void *pvParameters)
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
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

#ifdef _MODE_MAIN_
void app_main(void)
{
    pins_init();
    uart_init();
    ble_config();

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    xTaskCreate(iniciar_simulador, "iniciar simulador", 2048, NULL, 5, NULL);
}
#endif