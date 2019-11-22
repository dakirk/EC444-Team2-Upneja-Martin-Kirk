//This is base code for controlling the hub and fob. We will probably need two different programs because of their differing purposes.
//NOTE: the delay on the RX loop must be SHORTER than the delay on the TX loop, or else the RX loop will receive multiple messages jammed together in the same buffer.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/rmt.h"

//networking
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "esp_wifi.h"

static const int RX_BUF_SIZE = 1024;

#define HUB_ID "Table_1"

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

//int ledCounter = 0; //0 off, 1 red, 2 blue, 3 green

////WIFI & SOCKET SETUP///////////////////////////////////////////////////////////////////

//socket variables
#define HOST_IP_ADDR "192.168.1.122"                    //target server ip
#define PORT 3333                                       //target server port
char rx_buffer[128];
char addr_str[128];
int addr_family;
int ip_protocol;
int sock;                                               //socket id?
struct sockaddr_in dest_addr;                           //socket destination info

//wifi variables
#define EXAMPLE_ESP_WIFI_SSID "Group_2"
#define EXAMPLE_ESP_WIFI_PASS "smartkey"
#define EXAMPLE_ESP_MAXIMUM_RETRY 10                    //CONFIG_ESP_MAXIMUM_RETRY
static EventGroupHandle_t s_wifi_event_group;           //FreeRTOS event group to signal when we are connected
const int WIFI_CONNECTED_BIT = BIT0;                    //The event group allows multiple bits for each event, but we only care about one event - are we connected to the AP with an IP?
static const char *TAG = "wifi station";
static int s_retry_num = 0;
bool running = true;//false;

int sendData(const char* logName, const char* data);

////WIFI SETUP/////////////////////////////////////////////////////////////////////

//wifi event handler
static void event_handler(void* arg, esp_event_base_t event_base, 
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

//connect to wifi
void wifi_init_sta(void) {

    //Initialize NVS
    printf("Wifi setup part 1\n");
    esp_err_t ret = nvs_flash_init();
    printf("Wifi setup part 2\n");
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }

    printf("Wifi setup part 3\n");

    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    s_wifi_event_group = xEventGroupCreate();
    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);

}

////SOCKET SETUP/////////////////////////////////////////////////////////////////////

static void udp_init() {

    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    //socket setup struct
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

    //establish socket connection
    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
}

static void udp_client_receive() {

    while(1) {

        while(1) {

            printf("waiting for a message\n");

            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            printf("got one!\n");

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                //ESP_LOGI(TAG, "%s", rx_buffer);
                printf("received string: %s\n", rx_buffer);

                if (strcmp((char*)rx_buffer, "unlock") == 0) {
                    printf("unlocking...\n");
                    sendData("TX_TASK", "unlocked");
                    /*
                    gpio_set_level(GRN_LED, 1);
                    vTaskDelay(1000/portTICK_RATE_MS);
                    gpio_set_level(GRN_LED, 0);
                    */
                }

                /*
                if (strstr(rx_buffer, "start") != NULL) { //0 if equal
                    running = true;
                } else if (strstr(rx_buffer, "stop") != NULL) {
                    running = false;
                }*/

            }

        }

        //shut down socket if anything failed
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
            udp_init();
        }

    }

    vTaskDelete(NULL);

}

static void udp_client_send(char* message) {

    //assuming ip4v only
    printf("%s\n", message);

    //send message, and print error if anything failed
    int err = sendto(sock, message, strlen(message), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    }
}
/*
 * @brief RMT transmitter initialization
 */

//Sets up the 38kHz signal for IR transmission
static void nec_tx_init(void)
{
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_PIN;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    rmt_tx.tx_config.carrier_freq_hz = 38000; //IMPORTANT PART?
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


void uart_init(void) {
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

//Single-use function to send data over IR UART
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

        //Send ID and unlock code from here

        char* formattedMSG = "0 exampleunlockcode";

        sendData(TX_TASK_TAG, formattedMSG);

        /*
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
        */
        
        vTaskDelay(200 / portTICK_PERIOD_MS);

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

            //printf("%s\n", data);

            if (strstr((char*)data, "hub: ") != NULL) {
                printf("received data!");

                char udpBuf[100];

                //from here: https://stackoverflow.com/questions/15472299/split-string-into-tokens-and-save-them-in-an-array
                char* split[4];
                int i = 0;

                split[i] = strtok((char*)data, " ");

                while(split[i] != NULL) {
                   split[++i] = strtok(NULL, " ");
                }

                sprintf(udpBuf, "{\"fob_id\": \"%s\", \"hub_id\": \"%s\", \"code\": \"%s\"}", split[1], HUB_ID, split[2]);

                udp_client_send(udpBuf);
            } else {
                gpio_set_level(RED_LED, 1);
                vTaskDelay(1000/portTICK_RATE_MS);
                gpio_set_level(RED_LED, 0);
            }

            //Incoming data analysis here

            /*
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
            */

        }

    }
    free(data);
}

void app_main(void)
{

    wifi_init_sta();
    udp_init();
    uart_init();
    led_init();
    nec_tx_init();
    xTaskCreate(rx_task, "uart_rx_task", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(udp_client_receive, "udp_client_receive", 4096, NULL, configMAX_PRIORITIES-1, NULL);
}
