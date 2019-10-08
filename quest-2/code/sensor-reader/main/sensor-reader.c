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
#include <math.h>
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

//stores battery voltage
static uint32_t battery() {
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

}

//converts voltage across thermistor to Celsius and stores it
static uint32_t thermistor() {
    //Sample ADC1
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
    
    //calculate resistance across thermistor using voltage divider formula
    double resistance = (33000.0/((double)voltage/1000.0)) - 10000.0;
    //convert resistance across thermistor to Kelvin (T0 = 298K, B = 3435, R0 = 10kohm)
    double temperatureKelvin = -(1 / ((log(10000.0/resistance)/3435.0) - (1/298.0)));
    //convert Kelvin to Celsius
    return (temperatureKelvin - 273.15);
    
}

//converts voltage across ultrasonic to cm and stores it
static uint32_t ultrasonic() {
    //Sample ADC1
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
    //convert voltage to distance in centimeters
    return ((double)voltage/6.4) * 25.4 / 10;

}

//converts voltage across rangefinder to cm and stores it
static uint32_t rangefinder() {
    //Sample ADC1
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
    //Convert voltage to distance in centimeters
    uint32_t ir_distance = (-0.0619047619048*(double)voltage) + 125.238095238;
    //Limit the rangefinder to its functional range (between 20 and 150 cm)
    if (ir_distance < 20) {
        ir_distance = 20;
    } else if (ir_distance > 150) {
        ir_distance = 150;
    } else {
        ir_distance = ir_distance;
    }

    return ir_distance;
}

//displays sensor values on the console
static void display_console() {
    //print csv header to the serial port
    printf("Battery voltage (mV), temperature (C), ultrasonic distance (cm), infrared distance (cm)");

    uint32_t bat_voltage, temp, us_distance, ir_distance;
    
    //continuously print sensor readings to the serial port
    while (1) {

        bat_voltage = battery();
        temp = thermistor();
        us_distance = ultrasonic();
        ir_distance = rangefinder();
        
        printf("%d, %d, %d, %d\n", bat_voltage, temp, us_distance, ir_distance);

        vTaskDelay(pdMS_TO_TICKS(1000));

    }


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

    printf("checkpoint 1\n");

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
    
    //set up task for printing sensor results
    xTaskCreate(display_console, "display_console", 4096, NULL, 5, NULL);

}


