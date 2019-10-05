/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
TO DO:
Set up ADC channels for each sensor.
Test and debug.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <string.h>
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/i2c.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd
#define MAX 5

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

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

//bat_monitor
static const adc_channel_t channel1 = ADC_CHANNEL_4;     //GPIO32 
//thermistor_monitor
static const adc_channel_t channel2 = ADC_CHANNEL_9;     //GPIO36
//ultrasonic_monitor
static const adc_channel_t channel3 = ADC_CHANNEL_6;     //GPIO34 
//rangefinder_monitor
static const adc_channel_t channel4 = ADC_CHANNEL_3;     //GPIO39


uint32_t bat_voltage;
uint32_t temp;
uint32_t us_distance;
uint32_t ir_distance;

//stores battery voltage
static void battery() {
    //Continuously sample ADC1
    while (1) {
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
        bat_voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//converts voltage across thermistor to celsius and stores it
static void thermistor() {
    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel2);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel2, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    
        float resistance = ((voltage/1000.000)*(20000.000))/(3.300*(1.000-(voltage/3300.000)));
        temp = 1/((1.000/298.000) + (1.000/3435.000) * log(resistance/10000)) - 273.000;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//converts voltage across ultrasonic to mm and stores it
static void ultrasonic() {
    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel3);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel3, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        us_distance = (voltage/6.4) * 25.4;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//converts voltage across rangefinder to cm and stores it
static void rangefinder() {
    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel4);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel4, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        ir_distance = (-0.0619047619048*voltage) + 125.238095238;
        if (ir_distance < 20) {
            ir_distance = 20;
        } else if (ir_distance > 150) {
            ir_distance = 150;
        } else {
            ir_distance = ir_distance;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//displays sensor values on the console
static void display_console() {
    printf("Voltage: %dmV\n", bat_voltage);
    printf("Temperature: %dC\n", temp);
    printf("Ultrasonic Distance: %dmm\n", us_distance);
    printf("Rangefinder Distance: %dcm\n", ir_distance);
}

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
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

void app_main(void)
{ 
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel1, atten);
        adc1_config_channel_atten(channel2, atten);
        adc1_config_channel_atten(channel3, atten);
        adc1_config_channel_atten(channel4, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel1, atten);
        adc2_config_channel_atten((adc2_channel_t)channel2, atten);
        adc2_config_channel_atten((adc2_channel_t)channel3, atten);
        adc2_config_channel_atten((adc2_channel_t)channel4, atten);
    }
    
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    xTaskCreate(battery, "battery", 4096, NULL, 1, NULL);
    xTaskCreate(thermistor, "thermistor", 4096, NULL, 2, NULL);
    xTaskCreate(ultrasonic, "ultrasonic", 4096, NULL, 3, NULL);
    xTaskCreate(rangefinder, "rangefinder", 4096, NULL, 4, NULL);
    xTaskCreate(display_console, "display_console", 4096, NULL, 5, NULL);

}


