## Architecture:
### Hardware:
* ESP-32 for reading sensors and controlling crawler
* Laptop for starting and stopping crawler
* H-bridge + MCU for controlling crawler
* Accelerometer for reading speed (via acceleration changes) and yaw
* MicroLIDAR for sensing walls (sides?)
* LIDAR or ultrasonic sensor for sensing walls (front?)

### Firmware/Embedded:
* 1 or 2 parallel tasks speed and steering PID
* 1 parallel task for reading UDP messages
* 4 functions for reading the 4 sensors (3x rangefinder, 1x accelerometer)
* 1 function for controlling steering
* 1 function for controlling drive motors

### Software:
* Node.js server with UDP connection

## Division of work:

### Completed Tasks:
* Drive control
* Steering control
* MicroLIDAR reading
* Accelerometer reading

### Tasks to do:

Kyle:
* PID control (steering/orientation, speed, stopping)

David:
* Wireless communication (UDP)
* Speed measurement (accelerometer)
* Yaw measurement (accelerometer)

Ayush (tentative):
* Node server for UDP communication with crawler
* Finishing LIDAR
