# Quest 2: Sensors
Authors: Kyle Martin, David Kirk, Ayush Upneja

2019-10-07

## Summary



## Evaluation Criteria



## Solution Design

### Wiring
We used the same wiring scheme as the skills to connect our four sensors. We wired the battery reader with a 2/3 voltage divider from USB input (5V) and the thermistor in series with a 10k resistor. We connected the ultrasonic and infrared rangefinders directly with 5V and 3.3V power and ground respectively, feeding their analog outputs directly into the ESP32's ADC pins. 

### Microcontroller

### Node.js and Canvas.js
We used a Node.js web app with Express.js for REST calls and Canvas.js on the front end for graphing. The program reads the serial output of the ESP32 using the serialport module, and then saves each line to a csv file. When the user loads the webpage for the first time, it reads from this file to generate the graph. Every second after that, it adds data from the most recent serial reading to the end of the graph, providing a ticker tape-like effect.

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
