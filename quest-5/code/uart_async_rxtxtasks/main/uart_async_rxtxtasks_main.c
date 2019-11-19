/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/rmt.h"

static const int RX_BUF_SIZE = 1024;

#define RMT_PIN (GPIO_NUM_26) //A0
#define TXD_PIN (GPIO_NUM_25) //A1
#define RXD_PIN (GPIO_NUM_34) //A2
#define BTN_PIN (GPIO_NUM_18) //MO
#define GRN_LED (GPIO_NUM_14) //A6
#define BLU_LED (GPIO_NUM_32) //A7
#define RED_LED (GPIO_NUM_15) //A8

#define RMT_TX_CHANNEL    1     /*!< RMT channel for transmitter */
//#define RMT_TX_GPIO_NUM  18     /*!< GPIO number for transmitter signal */
#define RMT_RX_CHANNEL    0     /*!< RMT channel for receiver */
//#define RMT_RX_GPIO_NUM  19     /*!< GPIO number for receiver */
#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

int ledCounter = 0; //0 off, 1 red, 2 blue, 3 green

/*
 * @brief RMT transmitter initialization
 */
static void nec_tx_init(void)
{
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_PIN;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 1;
    rmt_tx.tx_config.idle_level = 1;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

void led_init(void) {
    gpio_pad_select_gpio(RED_LED);
    gpio_pad_select_gpio(BLU_LED);
    gpio_pad_select_gpio(GRN_LED);

    gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(BLU_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GRN_LED, GPIO_MODE_OUTPUT);
}


void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 2400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_rts(UART_NUM_1, 1);
    uart_set_line_inverse(UART_NUM_1, UART_INVERSE_RXD);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {

        if (ledCounter == 0) {
            sendData(TX_TASK_TAG, "off");
        } else if (ledCounter == 1) {
            sendData(TX_TASK_TAG, "red");
        } else if (ledCounter == 2) {
            sendData(TX_TASK_TAG, "blue");
        } else if (ledCounter == 3) {
            sendData(TX_TASK_TAG, "green");
        } 

        if (ledCounter >= 3) {
            ledCounter = 0;
        } else {
            ledCounter++;
        }

        
        vTaskDelay(500 / portTICK_PERIOD_MS);

    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 100 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            printf("%s\n", data);

            if (strcmp((char*)data, "red") == 0) {
                gpio_set_level(RED_LED, 1);
                gpio_set_level(BLU_LED, 0);
                gpio_set_level(GRN_LED, 0);
            } else if (strcmp((char*)data, "blue") == 0) {
                gpio_set_level(RED_LED, 0);
                gpio_set_level(BLU_LED, 1);
                gpio_set_level(GRN_LED, 0);
            } else if (strcmp((char*)data, "green") == 0) {
                gpio_set_level(RED_LED, 0);
                gpio_set_level(BLU_LED, 0);
                gpio_set_level(GRN_LED, 1);
            } else if (strcmp((char*)data, "off") == 0) {
                gpio_set_level(RED_LED, 0);
                gpio_set_level(BLU_LED, 0);
                gpio_set_level(GRN_LED, 0);
            }
            //ESP_LOGI("test: ", "result: %s\n", data);
            /*int i;

            for (i = 0; i < rxBytes; i++) {
                if (data[i] == 0x59 && data[i+1] == 0x59) {
                    break;
                }
            }

            for (i+=2; i < rxBytes; i+= 9) {
                //ESP_LOGI(RX_TASK_TAG, "Lower byte %d: %x", i, data[i]);
                //ESP_LOGI(RX_TASK_TAG, "Higher byte %d: %x", i, data[i+1]);
                int distConcat = (((uint16_t)data[i+1] << 8) | data[i]);
                ESP_LOGI(RX_TASK_TAG, "Distance: %d", distConcat);
            }*/
        }

        vTaskDelay(100/portTICK_RATE_MS);
    }
    free(data);
}

void app_main(void)
{
    init();
    led_init();
    nec_tx_init();
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
