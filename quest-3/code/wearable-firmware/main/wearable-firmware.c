//BASED ON: GPIO interrupt example code, I2C example code, UDP client example code

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//rtos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

//gpio
#include "driver/gpio.h"

//networking
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
//#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output
 * GPIO19: output
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Test:
 * Connect GPIO18 with GPIO4
 * Connect GPIO19 with GPIO5
 * Generate pulses on GPIO18/19, that triggers interrupt on GPIO4/5
 *
 */

/*
 * PLAN: create the following
 *
 * global variables:
 * int steps                     number of steps taken by user (counter)
 * int vibration_enabled         vibration sensor interrupt enable flag
 * int thermistor_enabled        thermistor enable flag
 * int battery_enabled           battery voltage enable flag
 * int water_alarm_enabled       water alert enable flag
 * int water_interval            time interval for the water alarm
 * int sock                      socket id?
 * struct sockaddr_in dest_addr  socket destination info
 *
 * single-run functions:
 * void socket_init()            sets up a UDP socket
 * void wifi_init()              sets up wifi (may just use example_connect())
 * void timer_init()             sets up timer
 * int read_battery()            reads battery voltage
 * int read_thermistor()         reads thermistor temperature
 * void gpio_interrupt()         handles GPIO interrupts (vibration sensor, increments step counter)
 * void socket_reconnect()       attempts to reconnect socket if anything failed
 * void ping_led()               turns on LED and then turns it off (async if possible)
 * 
 * RTOS task functions:
 * void socket_receive()         scans for new inbound data and sets relevant settings (low delay loop)
 * void socket_send(char*)       sends sensor data outbound
 * void water_timer()            timer for the "drink water" alarm
 *
 * STRUCTURE:
 * 
 * gpio_interrupt()
 *   get vibration_enabled
 *   set steps
 * app_main:
 *   sock_init()
 *     get sock
 *   gpio_init()
 *   wifi_init()
 *   timer_init()
 *   task - socket_receive()
 *     get sock
 *     get dest_addr
 *     set vibration_enabled
 *     set thermistor_enabled
 *     set battery_enabled
 *     set water_alarm_enabled
 *     task - ping_led()
 *     on failure - socket_reconnect()
 *   task - socket_send()
 *     get sock
 *     get battery_enabled
 *     get thermistor_enabled
 *     read_battery()
 *     read_thermistor()
 *     on failure - socket_reconnect()
 *   task - water_timer()
 *     get water_alarm_enabled
 *     
 *     
 * 
 */


//debouncer setup
#define GPIO_OUTPUT_IO_0    18
#define GPIO_OUTPUT_IO_1    19
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0
#define MAX_DEBOUNCE_COUNT 35
#define DEBOUNCE_TIMEOUT 50

int bounceCount;                                        //number of bounces
int startTime;                                          //for recording elapsed time to determine if debouncer has timed out
int interrupt_enabled = 1;                              //flag to disable interrupts during debouncing
int steps;                                              //number of steps taken by user (counter)
static xQueueHandle gpio_evt_queue = NULL;              //event queue

//flags
int vibration_enabled;                                  //vibration sensor interrupt enable flag
int thermistor_enabled;                                 //thermistor enable flag
int battery_enabled;                                    //battery voltage enable flag
int water_alarm_enabled;                                //water alert enable flag

//timer setup
int water_interval;                                     //time interval for the water alarm

//socket setup
#define HOST_IP_ADDR "192.168.1.101"                    //target server ip
#define PORT 8080                                       //target server port
char rx_buffer[128];
char addr_str[128];
int addr_family;
int ip_protocol;
int sock;                                               //socket id?
struct sockaddr_in dest_addr;                           //socket destination info

//wifi setup
#define EXAMPLE_ESP_WIFI_SSID "Group_2"
#define EXAMPLE_ESP_WIFI_PASS "smartkey"
#define EXAMPLE_ESP_MAXIMUM_RETRY 10//CONFIG_ESP_MAXIMUM_RETRY

static EventGroupHandle_t s_wifi_event_group;           //FreeRTOS event group to signal when we are connected
const int WIFI_CONNECTED_BIT = BIT0;                    //The event group allows multiple bits for each event, but we only care about one event - are we connected to the AP with an IP?
static const char *TAG = "wifi station";
static int s_retry_num = 0;


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
void wifi_init_sta(void)
{
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

//set up GPIO interrupts
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

//GPIO interrupt task
static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY) && interrupt_enabled) {

            int currTime = xTaskGetTickCount();

            //initialize debouncer
            if (bounceCount == 0) {
                startTime = currTime;
                //printf("Starting debouncer: %d\n", startTime);
                bounceCount++;
            }

            //if this is a real buttonpress, there will have been enough bounces in the given time period
            else if (bounceCount >= MAX_DEBOUNCE_COUNT && currTime - startTime <= DEBOUNCE_TIMEOUT) {
                interrupt_enabled = 0; //disable interrupts during the following commands

                bounceCount = 0;
                gpio_set_level(GPIO_OUTPUT_IO_0, 1);
                printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
                vTaskDelay(500/portTICK_RATE_MS);
                gpio_set_level(GPIO_OUTPUT_IO_0, 0);

                interrupt_enabled = 1; //reenable interrupts

            }

            //if too much time has elapsed, reset debouncer
            else if (currTime - startTime > DEBOUNCE_TIMEOUT) {
                //printf("timed out. resetting...");
                bounceCount = 0;
            }

            //increment bounce count if no other cases met
            else {
                bounceCount++;
            }

            


        }
    }
}


static void gpio_setup() {

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    bounceCount = 0;
}

void app_main(void)
{

    gpio_setup();

    int cnt = 0;
    while(1) {
        printf("cnt: %d\n", cnt++);
        vTaskDelay(1000 / portTICK_RATE_MS);
        //gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        //gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);
    }
}

