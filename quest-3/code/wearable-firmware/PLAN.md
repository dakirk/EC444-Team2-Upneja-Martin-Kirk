
#PLAN: create the following

##global variables:
* int steps                     number of steps taken by user (counter)
* int vibration_enabled         vibration sensor interrupt enable flag
* int thermistor_enabled        thermistor enable flag
* int battery_enabled           battery voltage enable flag
* int water_alarm_enabled       water alert enable flag
* int water_interval            time interval for the water alarm
* int sock                      socket id?
* struct sockaddr_in dest_addr  socket destination info

##single-run functions:
* void socket_init()            sets up a UDP socket
* void wifi_init()              sets up wifi (may just use example_connect())
* void timer_init()             sets up timer
* int read_battery()            reads battery voltage
* int read_thermistor()         reads thermistor temperature
* void gpio_interrupt()         handles GPIO interrupts (vibration sensor, increments step counter)
* void socket_reconnect()       attempts to reconnect socket if anything failed
* void ping_led()               turns on LED and then turns it off (async if possible)

##RTOS task functions:
* void socket_receive()         scans for new inbound data and sets relevant settings (low delay loop)
* void socket_send(char[])       sends sensor data outbound
* void water_timer()            timer for the "drink water" alarm

#STRUCTURE:

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
