# Quest 6
Authors: Kyle Martin, David Kirk, Ayush Upneja

2019-12-11

## Summary



## Evaluation Criteria



## Solution Design

### Hardware

The crawler uses the chassis we were given for Quest 4, with several additions. It is controlled with an ESP32, which drives the motor controller directly and the steering motors through and H-bridge. Unlike Quest 4, we have changed our power system so that the ESP32 is now powered off a separate battery, which also powers the Raspberry Pi. We are using two HC-SR04 ultrasonic sensors for rangefinding on the front and left side of the car, and an infrared sensor for picking up the traffic signals. On top, we have a 14-segment display for showing split times. The Raspberry Pi is taped to the top as well, and powers the webcam, which is mounted to the front bumper.

### Firmware

### Backend

We based this code off of the Quest 5 Node.js server, which uses TingoDB to store data. Each time the ESP32 transmits a spit time to the server, it stores it, along with the timestamp when it was received, in the databse. The server also provides an endpoint ```/logs```, which sends the database's contents as a JSON string. Additionally, the server can receive URL parameters (speed, steering, start/stop) for controlling the crawler, which are received from the ```/params``` endpoint as ```speed```, ```steer```, and ```start```. Speed and steering are received as ±1, with the number added onto an internal speed value to control the direction of movement. The start parameter is binary, and toggles the crawler between moving and stopped. All of these instructions are passed on over UDP to the crawler.

### Frontend



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
