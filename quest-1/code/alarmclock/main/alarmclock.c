/*
  Adapted I2C example code to work with the Adafruit 14-segment Alphanumeric Display. Key notes: MSB!!
  Emily Lam, Sept 2018, Updated Aug 2019
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "esp_vfs_dev.h"
#include "esp_types.h"
#include "sdkconfig.h"
#include "displaychars.h"
#include "pins.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

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

#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (1)    // Sample test interval for the first timer
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

static void led_counter();
static void flag_alarm();

int direction = 1; // 1 for up, -1 for down
int alarmSetting = 86400;
int currentTime = 0;
int alarm_flag = 0;

// Timer Functions //////////////////////////////////////////////////////////

// A simple structure to pass "events" to main task
typedef struct {
    int flag;     // flag for enabling stuff in main code
} timer_event_t;

// Initialize queue handler for timer-based events
xQueueHandle timer_queue;

// ISR handler
void IRAM_ATTR timer_group0_isr(void *para) {

    // Prepare basic event data, aka set flag
    timer_event_t evt;
    evt.flag = 1;

    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;

    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

// Initialize timer 0 in group 0 for 1 sec alarm interval and auto reload
static void alarm_init() {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
        (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

// Handles timing, in 1-second increments
static void timer_evt_task(void *arg) {
    while (1) {
        // Create dummy structure to store structure from queue
        timer_event_t evt;

        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        // Do something if triggered!
        if (evt.flag == 1) {
            //printf("Time: %d\n", currentTime);

            //led_counter();
            flag_alarm();

            if (currentTime >= 86399) {
              currentTime = 0;
            } else {
              currentTime++;
            }
        }
    }
}



//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 460  //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2450 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate
#define SERVO2_MIN_PULSEWIDTH 450  //Minimum pulse width in microsecond
#define SERVO2_MAX_PULSEWIDTH 2450 //Maximum pulse width in microsecond
#define SERVO2_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 12);    //Set GPIO 18 as PWM0A, to which servo is connected
}
static void mcpwm_example_gpio_initialize2(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, 13);    //Set GPIO 18 as PWM0A, to which servo is connected
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

static uint32_t servo2_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO2_MIN_PULSEWIDTH + (((SERVO2_MAX_PULSEWIDTH - SERVO2_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO2_MAX_DEGREE)));
    return cal_pulsewidth;
}

/**
 * @brief Configure MCPWM module
 */
void servo_seconds(void *arg)
{
    uint32_t angle, count;
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
    while (1) {
        for (count = 0; count < SERVO_MAX_DEGREE; count++) {
            angle = servo_per_degree_init((currentTime%60)*3);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
            vTaskDelay(10 / portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        }
    }
}
void servo_minutes(void *arg)
{
    uint32_t angle, count, pulse_width;
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize2();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    while (1) {
        for (count = 0; count < SERVO2_MAX_DEGREE; count++) {
            angle = ((currentTime/60)%60)*3;
            pulse_width = servo2_per_degree_init(angle);
            //printf("Angle of rotation: %d\n", angle);
            //printf("pulse width: %dus\n", pulse_width);
            mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, pulse_width);
            vTaskDelay(100 / portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        }
    }
}

// Function to initiate i2c -- note the MSB declaration!
static void i2c_example_master_init(){
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
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK) {printf("- initialized: yes\n\n");}

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

//given empty 4-char string outputTime, fills with given hours and mins
void formatTime(char outputTime[], int secondsSinceMidnight) {

  char hours[8];
  char mins[8];

  int intHours = currentTime / 3600;
  int intMins = (currentTime / 60) % 60;

  //intHours = currentTime / 60 % 60;
  //intMins = currentTime % 60;

  //itoa(intHours, hours, 10);
  //itoa(intMins, mins, 10);
  sprintf(hours, "%02d", intHours);
  sprintf(mins, "%02d", intMins);

  strcpy(outputTime, hours);
  strcat(outputTime, mins);

  //printf("concatenated: %s hours: %d mins: %d\n", outputTime, intHours, intMins);
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

////////////////////////////////////////////////////////////////////////////////

// Alphanumeric Functions //////////////////////////////////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

////////////////////////////////////////////////////////////////////////////////

void process_input() {
    char time[5];
    char chrs[3];
    char cmins[3];
    char mode[2];
    int hrs;
    int mins;
    while(1) {
      printf("Enter A for alarm set, T for time set\n");
    
      gets(mode);

      if (mode[0] == 'A' && mode[1] == '\0') {
          printf("Enter alarm time in military time\n");
          gets(time);
          printf("%s\n", time);
          chrs[0] = time[0];
          chrs[1] = time[1];
          cmins[0] = time[2];
          cmins[1] = time[3];
          hrs = atoi(chrs);
          mins = atoi(cmins);
          alarmSetting = (hrs*3600) + (mins*60);
      } else if (mode[0] == 'T' && mode[1] == '\0'){
          printf("Enter currentTime time in military time\n");
          gets(time);
          printf("%s\n", time);
          chrs[0] = time[0];
          chrs[1] = time[1];
          cmins[0] = time[2];
          cmins[1] = time[3];
          hrs = atoi(chrs);
          mins = atoi(cmins);
          currentTime = (hrs*3600) + (mins*60);
      } else {
          printf("Invalid entry.");
      }
    }

    
}

void flag_alarm() {
  if (currentTime == alarmSetting && alarm_flag == 0) {
    alarm_flag = 1;
  }
}

static void run_alarm() {

  while(1) {

    int button_pressed;

    if (alarm_flag == 1) {

      button_pressed = gpio_get_level(A2);
      //printf("Button val: %d\n", button_pressed);

      if (button_pressed) {
        alarm_flag = 0;
        gpio_set_level(A6, 0);
        gpio_set_level(A7, 0);
        gpio_set_level(A8, 0);
        gpio_set_level(A9, 0);
      } else {

        gpio_set_level(A6, 1);
        gpio_set_level(A7, 1);
        gpio_set_level(A8, 1);
        gpio_set_level(A9, 1);  

      }

      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

static void test_alpha_display() {


    // Debug
    int ret;
    printf(">> Test Alphanumeric Display: \n");

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}

    // Continually writes the same command
    while (1) {

      uint16_t displaybuffer[8];
      int i = 0;
      char timeStr[5];

      formatTime(timeStr, currentTime);
      //printf("%s, %d\n", timeStr, currentTime);

      /*
      //set display direction
      if (direction == -1) {
        strcpy(dirStr, "DOWN");
      } else {
        strcpy(dirStr, "UP  ");
      }*/

      //put in display buffer
      while (i < 4 && timeStr[i] != '\0') {
        displaybuffer[i] = alphafonttable[(int)timeStr[i]];
        ++i;
      }

      // Send commands characters to display over I2C
      i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
      i2c_master_start(cmd4);
      i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
      for (uint8_t i=0; i<8; i++) {
        i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
      }
      i2c_master_stop(cmd4);
      ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
      i2c_cmd_link_delete(cmd4);

       //for (int i = 0; i < 8; i++) {
       //    printf("%04x\n", displaybuffer[i]);
       //}

      if(ret == ESP_OK) {
        //printf("- wrote: T.D.C.L. \n\n");
      }
    }


}

// Init  Functions //////////////////////////////////////////////////////////

//initialize input pin
static void button_init() {
  gpio_pad_select_gpio(A2);
  gpio_set_direction(A2, GPIO_MODE_INPUT);
}


//set up LED output
static void led_init() {
  gpio_pad_select_gpio(A6);
  gpio_pad_select_gpio(A7);
  gpio_pad_select_gpio(A8);
  gpio_pad_select_gpio(A9);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(A6, GPIO_MODE_OUTPUT);
  gpio_set_direction(A7, GPIO_MODE_OUTPUT);
  gpio_set_direction(A8, GPIO_MODE_OUTPUT);
  gpio_set_direction(A9, GPIO_MODE_OUTPUT);
}

static void led_counter() {

  gpio_set_level(A9, (currentTime % 4 == 0));
  gpio_set_level(A8, (currentTime % 4 == 1));
  gpio_set_level(A7, (currentTime % 4 == 2));
  gpio_set_level(A6, (currentTime % 4 == 3));

}

void app_main() {

  // Create a FIFO queue for timer-based
  timer_queue = xQueueCreate(10, sizeof(timer_event_t));

  /* Install UART driver for interrupt-driven reads and writes */
  ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
    256, 0, 0, NULL, 0) );

  /* Tell VFS to use UART driver */
  esp_vfs_dev_uart_use_driver(UART_NUM_0);

  //initializations
  button_init();
  led_init();
  i2c_example_master_init();
  i2c_scanner();

  //parallel tasks
  xTaskCreate(timer_evt_task, "timer_evt_task", 4096, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(test_alpha_display,"test_alpha_display", 4096, NULL, configMAX_PRIORITIES-1, NULL);
  xTaskCreate(servo_seconds, "servo_seconds", 4096, NULL, configMAX_PRIORITIES-2, NULL);
  xTaskCreate(servo_minutes, "servo_minutes", 4096, NULL, configMAX_PRIORITIES-3, NULL);
  xTaskCreate(process_input, "process_input", 4096, NULL, configMAX_PRIORITIES-4, NULL);
  xTaskCreate(run_alarm, "run_alarm", 4096, NULL, configMAX_PRIORITIES-5, NULL);

  // Initiate alarm using timer API
  alarm_init();
}