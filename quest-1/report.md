# Retro Alarm Clock
Authors: Kyle Martin, David Kirk, Ayush Upneja

2019-09-20

## Summary

In this skill quest, we connected the i2c alphanumeric display board and two servo motors to the ESP32 to create a retro alarm clock.  The alphanumeric board reads hours and minutes in military time, while the two servo motors display minutes and seconds.  Users can set both the current time and the alarm time through the command line.  For the alarm, we use four LEDs that light up when the current time is equal to the alarm time.  The lights can be turned off by button press.

## Evaluation Criteria

We successfully demonstrated:

- Two servos that indicate time in seconds and minutes
- An alphanumeric display that indicates time in hours and minutes
- The ability for a user to set the time
- The ability for a user to set an alarm

## Solution Design

Relevant pinouts for our design are as follows:

Servo that reads seconds -> 12
Servo that reads minutes -> 13
I2C Alphanumeric Display -> SDA and SCL
Button that stops alarm -> A2
Alarm LEDs -> A6-A9

User interaction with the system is done through the command line, and user inputs in the form of HH:MM are stored as strings.  These strings are parsed into two integers, one for hours and one for minutes.  These values are converted to seconds, added together, and stored as an integer number of "seconds since midnight".      




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
