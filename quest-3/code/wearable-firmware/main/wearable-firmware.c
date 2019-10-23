//TODO:
// 1. thermistor function
// 2. set up water alarm (timer)
// 3. make ping_led light an led and be asynchronous
// 4. format socket output to be in JSON
// 5. connect with Kyle's Node.js server
// 6. reorganize for better readablity


//BASED ON: GPIO interrupt example code, I2C example code, UDP client example code

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//rtos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

//gpio & adc
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_vfs_dev.h"

//networking
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "tcpip_adapter.h"
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

//debouncer variables
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
int steps = 0;                                          //number of steps taken by user (counter)
static xQueueHandle gpio_evt_queue = NULL;              //event queue

//adc setup
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

//initializing attenuation variables
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

//bat_monitor adc
static const adc_channel_t channel1 = ADC_CHANNEL_4;     //GPIO32 
//thermistor_monitor adc
static const adc_channel_t channel2 = ADC_CHANNEL_0;     //GPIO36
//ultrasonic_monitor adc
static const adc_channel_t channel3 = ADC_CHANNEL_6;     //GPIO34 
//rangefinder_monitor adc
static const adc_channel_t channel4 = ADC_CHANNEL_3;     //GPIO39

//flags
int vibration_enabled = 1;                              //vibration sensor interrupt enable flag
int thermistor_enabled = 1;                             //thermistor enable flag
int battery_enabled = 1;                                //battery voltage enable flag
int water_alarm_enabled = 1;                            //water alert enable flag

//timer variables
int water_interval = 3600;                              //time interval for the water alarm

//socket variables
#define HOST_IP_ADDR "192.168.1.101"                    //target server ip
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
#define EXAMPLE_ESP_MAXIMUM_RETRY 10//CONFIG_ESP_MAXIMUM_RETRY

static EventGroupHandle_t s_wifi_event_group;           //FreeRTOS event group to signal when we are connected
const int WIFI_CONNECTED_BIT = BIT0;                    //The event group allows multiple bits for each event, but we only care about one event - are we connected to the AP with an IP?
static const char *TAG = "wifi station";
static int s_retry_num = 0;

static void ping_led();
static int battery();


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
void wifi_init_sta(void)
{

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
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

    //tcpip_adapter_init();

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

            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                //expects string of format "10101 1234"

                vibration_enabled = rx_buffer[0] - 48;
                thermistor_enabled = rx_buffer[1] - 48;
                battery_enabled = rx_buffer[2] - 48;
                water_alarm_enabled = rx_buffer[3] - 48;

                if (rx_buffer[4] - 48) { //if 5th bit is 1, run the "ping_led() function"
                    ping_led();
                }

                printf("number at the end: %s", (rx_buffer+5));
            }

        }

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
    printf("attempting to send\n");

    int batteryVoltage = battery();


    printf("battery: %d\n", batteryVoltage);

    int err = sendto(sock, message, strlen(message), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        //break;
    }
}

////VIBRATION SENSOR SETUP/////////////////////////////////////////////////////////////////////

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
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY) && interrupt_enabled && vibration_enabled) {

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
                steps++;
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

static int battery() {

    if (battery_enabled) {
        //Sample ADC1
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel1);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel1, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV

        return esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    } else {
        return -1;
    }

}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void adc_init() {

    //Configure ADC channels for each sensor
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        //battery
        adc1_config_channel_atten(channel1, atten);
        //thermistor
        adc1_config_channel_atten(channel2, atten);
        //ultrasonic
        adc1_config_channel_atten(channel3, atten);
        //rangefinder
        adc1_config_channel_atten(channel4, atten);
    } else {
        //battery
        adc2_config_channel_atten((adc2_channel_t)channel1, atten);
        //thermistor
        adc2_config_channel_atten((adc2_channel_t)channel2, atten);
        //ultrasonic
        adc2_config_channel_atten((adc2_channel_t)channel3, atten);
        //rangefinder
        adc2_config_channel_atten((adc2_channel_t)channel4, atten);
    }
    
    printf("checkpoint 2\n");

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));

    printf("checkpoint 3\n");

    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    printf("checkpoint 4\n");

    print_char_val_type(val_type);

}

static void ping_led() {
    printf("led pinged!\n");
}

static void test_task() {
    char stepBuf[10];

    int cnt = 0;
    while(1) {
        printf("cnt: %d\n", cnt++);
        vTaskDelay(1000 / portTICK_RATE_MS);

        itoa(steps, stepBuf, 10);

        udp_client_send(stepBuf);
        //gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        //gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);
    }
}

void app_main(void)
{

    gpio_setup();
    wifi_init_sta();
    udp_init();
    adc_init();

    xTaskCreate(udp_client_receive, "udp_client_receive", 4096, NULL, 5, NULL);
    xTaskCreate(test_task, "test_task", 4096, NULL, 4, NULL);

}

