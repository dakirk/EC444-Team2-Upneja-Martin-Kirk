# Quest 4 - Crawler
Authors: Kyle Martin, David Kirk, Ayush Upneja

2019-11-08

## Summary

In this quest, we implemented autonomous control into a crawler issued by EC444.  We used two algorithms to control the crawler: one to control steering and one to control speed.  The sensor uses two microlidars, one in the front and one on the side, to track distance, and uses a pulse counter to track linear speed.  We use an alphanumeric display to visualize distance from the front wall.  We were also able to start and stop the crawler wirelessly.

## Evaluation Criteria

- Uses PID for speed control for setpoint of 0.1-0.4 m/s
- Stops within 10 cm of a wall
- Drives in the center of the track +/-25cm
- Traverses A -> B in one go, no nudges or hits
- Uses alpha display to show current position or speed
- Controlled remotely, start and stop

## Solution Design

### Circuit

In this crawler, everything is powered off of the 7.2V battery. The ESP32 is powered from the 5V output line of the electronic speed control unit (ESC), as are the microlidars and the steering servo. The 14-segment display is powered off 3.3V from the ESP32, and all devices share a common ground. For communication, the PWM line on the steering servo is upscaled to 5V through an H-bridge from a GPIO pin on the ESP32, while the PWM pin on the ESC is connected directly to a GPIO pin without any upscaling. Both microlidars have their RX and TX lines connected to GPIOs for UART communication, and the 14-segment display has its SDA and SCL lines connected to the desegnated pins on the controller for I2C communication.

### Reading from Sensors

The microlidars output 9-byte hexadecimal values, where the first two bytes are equal to 0x59 and the following two bytes correspond to the distance.  To read the hexdump from the microlidar, every time we read two 0x59s in a row, we read the next two bytes into a single 16 bit value.  This value corresponded to the distance from the wall in centimeters.

The pulse counter increases in voltage at lower brightness. To interpret wheel speed from the pulse counter, we lined the back right wheel with a donut-shaped piece of paper with alternating black and white sections.  With each sharp increase in voltage, we incremented a count.  Each time the count reached 6, the wheel completed one cycle. Since the wheel is about 0.6 meters in circumfrence, we can easily determine wheel speed if we sample the number of wheel rotations over a known time interval.  In our implementation, we sampled the rotation count every one second.  A sample rate smaller than this would yield very limited readings, as we would not be able to get a useful black-stripe count.

### Speed and Steering Control

In the speed-control algorithm, we maintained an integral, derivative, and previous error term.  We initialized the setpoint to 0.3 m/s.  In order to determine the output of the algorithm, we executed the following steps:

- error = setpoint_sp - speed;
- integral_sp = integral_sp + error * dt;
- derivative_sp = (error - previous_error_sp) / dt;
- previous_error_sp = error;
- output = kp * error + kd * derivative_sp + ki * integral_sp;

Where ki = kp = kd = 0.1 and dt = 1.  To determine the new duty to be sent to the wheel servos, we added 0.001 * output to the current duty.  

In the steering control algorithm, we calculate an error between the current distance from the side wall and a setpoint of 90 centimeters.  If the error is less than -25 centimeters or greater than 25 centimeters, we turn the wheels right or left.  Otherwise, we set the wheels straight.    

### Actuation

We define two tasks that control actuation in the servos: drive control and steering control.  In the driving control task, we read from the pulse counter and the front microlidar once every second, update the linear speed, run the speed PID algorithm, and set the duty of the motors.  If the distance read by the front microlidar is less than 40 centimeters, we stop the crawler.  We also write the distance to the alphanumeric display.

In the steering control task, we read the distance from the side microlidar, update the current distance from the side wall, run the logic-based steering control algorithm, and update the duty of the servo that turns the front wheels.

### Socket Communication

To control the crawler wirelessly, we connected a node.js server to the ESP32 through a UDP socket connection using our Linksys router.  From the server, we could send "start" and "stop" messages via the command line.  Upon recieving these messages, the embedded program sets a "running" boolean to true or false.  Every while loop in the program is enabled by this variable, so when the boolean is true they are permitted to run.  When false, the crawler stops.   

## Sketches and Photos
<center><img src="./images/example.png" width="70%" /></center>  
<center> </center>


## Supporting Artifacts
- [Link to repo]()
- [Link to video demo](https://www.youtube.com/watch?v=4e3qMkAvSgg&feature=youtu.be)


## Investigative Question

We would want a sensor that can more accurately determine wheel speed.  The accuracy of the pulse counter is reliant upon how many black sections there are in the wheel lining.  A device that reads pulses generated by the wheel and converts to rpms.  A tachometer would be helpful in this scenario.

## References

- Investigative: https://www.instrumart.com/MoreAboutCategory?CategoryID=5553
- Standard idf.py toolchain
- MCPWM example code
- https://cdn.sparkfun.com/assets/5/e/4/7/b/benewake-tfmini-datasheet.pdf
- https://github.com/espressif/esp-idf/tree/master/examples/peripherals/uart
- http://whizzer.bu.edu/briefs/design-patterns/dp-pid

-----

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
