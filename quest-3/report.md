# Quest 3
Authors: Ayush Upneja, David Kirk, Kyle Martin

2019-10-24

## Summary

In this quest we wired three different devices to the ESP32: a battery, a thermistor, and a vibration sensor. The ESP32 collects data from these devices and sends them over WiFi to the Node backend through a UDP socket, while simultaneously receiving control signals from the Node backend through the same socket.

Insert what happens in Node backend

In the front end, we read all of the inputs and graphed them using ChartJS in an instantaneous updating. 



## Evaluation Criteria



## Solution Design
Hardware:
David worked on the hardware. The ESP32 reads three different sensors (battery voltage, thermistor, and vibration sensor), and outputs to two LEDs. The firmware running on the ESP32 runs three parallel tasks: a UDP socket receiver, an output handler, and a timer handler. Additionally, there is a GPIO interrupt for reading, debouncing, and counting steps detected by the vibration sensor.

The socket receiver, udp_client_receive(), waits for a response from the target server (determined by the HOST_IP_ADDR and PORT macros). On receiving one, it parses and acts on the result. The first 5 characters of the response always form a binary string, with each bit corresponding to a feature. From left to right, they enable the vibration sensor, the thermistor, the battery reader, the "drink water" alarm, and the "find my device" LED. Any digits after that are interpreted as a new interval for the "drink water" alarm.

The output task, output_task(), reads from the thermistor and battery every 0.1 seconds, and formats a JSON object from those two readings and the global step counter. It then sends the resulting string through the UDP socket to the server.

The timer task, timer_evt_task(), flashes the blue LED on and off for five seconds when triggered. This trigger happens according to an interval set by UDP input.

The interrupt task, vibration_interrupt_task(), waits for a GPIO trigger from the vibration sensor. When it detects one, it attempts to count 20 sensor bounces in the next 50ms. If this happens, it increments the global steps counter.

Back-end:
Kyle worked on the back-end.

Front-end:
Ayush worked on the front-end. The front-end reads data from the socket when there is a change in step/battery/temperature.
There are bootstrap 5 toggle buttons to turn on/off the data reading. The on-toggle command is evaluated through jquery functions to take away the necessity of a submit button. The water alarm button cannot be set to a value other than a positive number and provides an alert when anything else is entered. There is a find my device button as well that lights up the red led when pressed.

The data is read from the TCP Socket and is graphed into ChartJS at a rate of every 1/10 of a second. Toggling off the data collection also hides the relevant line.


## Sketches and Photos
<center><img src="./images/example.png" width="70%" /></center>  
<center> </center>


## Supporting Artifacts
- [Link to repo]()
- [Link to video demo]()


## References

-----

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
