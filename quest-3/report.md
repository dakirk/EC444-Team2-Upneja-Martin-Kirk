# Quest 3
Authors: Ayush Upneja, David Kirk, Kyle Martin

2019-10-24

## Summary

In this quest, we built a wearable that tracks biometric data and communicates information to a central graphical hub.  We wired three different devices to the ESP32: a battery, a thermistor, and a vibration sensor.  We also wired two LEDs to the ESP32, where one blinked if pinged by the user from the front-end and another blinked to notify the user to drink water.  The battery level, temperature, and step count were sent to the node server through a UDP socket and then sent to the front-end through a TCP socket.  To control the device from the front-end, we used HTTP POST requests, which carried payload information that was sent back to the ESP32 through the UDP socket.  In the front end we plotted real-time sensor data using Canvas.js and embedded buttons, switches, and text fields to control the post requests.  

The division of labor was as follows:

- David wired the device and wrote the embedded C code to interface with it.
- Kyle wrote the node.js server and set up port-forwarding.
- Ayush wrote the front-end HTML file that plotted sensor data and interfaced with the user.

## Evaluation Criteria



## Solution Design


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
