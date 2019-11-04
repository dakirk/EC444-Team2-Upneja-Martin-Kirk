/* servo motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/i2c.h"
#include "soc/mcpwm_periph.h"
#include "./ADXL343.h"

////DRIVING SETUP///////////////////////////////////////////////////////////////////

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define DRIVE_MIN_PULSEWIDTH 900 //Minimum pulse width in microsecond
#define DRIVE_MAX_PULSEWIDTH 1900 //Maximum pulse width in microsecond
#define DRIVE_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate
#define STEERING_MIN_PULSEWIDTH 700 //Minimum pulse width in microsecond
#define STEERING_MAX_PULSEWIDTH 2100 //Maximum pulse width in microsecond
#define STEERING_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate

////I2C SETUP///////////////////////////////////////////////////////////////////

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// ADXL343
#define ACCEL_ADDR                         ADXL343_ADDRESS // 0x53

// 14-Segment Display
#define DISPLAY_ADDR                       0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

////UART SETUP///////////////////////////////////////////////////////////////////
static const int RX_BUF_SIZE = 128;

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

////I2C FUNCTIONS///////////////////////////////////////////////////////////////////

// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK) {printf("- initialized: yes\n");}

    // Data in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
        printf("- No I2C devices found!" "\n");
    printf("\n");
}

////I2C FUNCTIONS///////////////////////////////////////////////////////////////////

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

int uart_sendData(const char* logName, const char* data)
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
        uart_sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

int rx_task()
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    int avgDist = 0;
    int distConcat = 0;

    //while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 100 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            //printf("%s\n", data);
            //ESP_LOGI("test: ", "result: %s\n", data);
            int i;
            int distCounter = 0;

            for (i = 0; i < rxBytes; i++) {
                if (data[i] == 0x59 && data[i+1] == 0x59) {
                    break;
                }
            }

            for (i+=2; i < rxBytes; i+= 9) {
                //ESP_LOGI(RX_TASK_TAG, "Lower byte %d: %x", i, data[i]);
                //ESP_LOGI(RX_TASK_TAG, "Higher byte %d: %x", i, data[i+1]);
                distConcat = (((uint16_t)data[i+1] << 8) | data[i]);
                avgDist += distConcat;
                distCounter++;

                //ESP_LOGI(RX_TASK_TAG, "Distance: %d", distConcat);
            }

            avgDist /= distCounter;

            ESP_LOGI(RX_TASK_TAG, "Distance: %d", avgDist);

        }

        //vTaskDelay(100/portTICK_RATE_MS);
    //}
    free(data);

    return distConcat;
}

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 12);    //Set GPIO 12 as PWM0A, to which drive wheels are connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 19);    //Set GPIO 19 as PWM0A, to which steering servo is connected
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t drive_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (DRIVE_MIN_PULSEWIDTH + (((DRIVE_MAX_PULSEWIDTH - DRIVE_MIN_PULSEWIDTH) * (degree_of_rotation)) / (DRIVE_MAX_DEGREE)));
    return cal_pulsewidth;
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t steering_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (STEERING_MIN_PULSEWIDTH + (((STEERING_MAX_PULSEWIDTH - STEERING_MIN_PULSEWIDTH) * (degree_of_rotation)) / (STEERING_MAX_DEGREE)));
    return cal_pulsewidth;
}

void pwm_init() {
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
}

void calibrateESC() {
    vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds - backwards
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700);  // LOW signal in microseconds - forwards
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

/**
 * @brief Configure MCPWM module
 */
void drive_control(void *arg)
{
    uint32_t angle, count;

    while (1) {
        //for (count = 0; count < DRIVE_MAX_DEGREE; count++) {
        //count = 180;
        //    printf("Angle of rotation: %d\n", count);
        //    angle = drive_per_degree_init(count);
        //    printf("pulse width: %dus\n", angle);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1100);
        //vTaskDelay(100/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V

        int avgDist = rx_task();
        //printf("dist: %d\n", avgDist);

        //if (avgDist < 90) {
        //    break;
        //}

        //}
    }

    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);

    vTaskDelete(NULL);
}

/**
 * @brief Configure MCPWM module
 */
void steering_control(void *arg)
{
    uint32_t angle, count;

    while (1) {

        //for (count = 0; count < STEERING_MAX_DEGREE; count++) {
            count = 90;
            //printf("Angle of rotation: %d\n", count);
            angle = steering_per_degree_init(count);
            //printf("pulse width: %dus\n", angle);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
            vTaskDelay(100/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        //}


    }
}

void app_main(void)
{

    // I2C startup routine
    i2c_master_init();
    i2c_scanner();

    // Drive & steering startup routine
    pwm_init();
    calibrateESC();

    uart_init();
    //xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);

    printf("Testing servo motor.......\n");
    xTaskCreate(steering_control, "steering_control", 4096, NULL, 5, NULL);
    xTaskCreate(drive_control, "drive_control", 4096, NULL, 4, NULL);

}
