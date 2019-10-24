# Quest 3
Authors: Ayush Upneja, David Kirk, Kyle Martin

2019-10-24

## Summary

In this quest we wired three different devices to the ESP32: a battery, a thermister, and a vibration sensor.

Insert what happens in hardware

Insert what happens in Node backend

In the front end, we read all of the inputs and graphed them using ChartJS in an instantaneous updating. 



## Evaluation Criteria



## Solution Design
Hardware:
David worked on the hardware.

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
