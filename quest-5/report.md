# Quest 5
Authors: Kyle Martin, Ayush Upneja, David Kirk

2019-11-22

## Summary



## Evaluation Criteria



## Solution Design

For explanation purposes, anything in brackets is a placeholder for real data.

### Circuit/Hardware
![Circuit diagram](images/ir-circuit-diagram.png)

We used the same circuit as described in the IR RX/TX skill for both the fob and the hub. We changed a few of the pin assignments for convenience, but otherwise the circuits are identical. In it, the ESP32 generates a 38kHz signal and a UART signal, which are combined using an H-bridge and transmitted through an IR LED. The IR receiver is connected directly to a GPIO input. The button is wired with a 10kΩ pull-up resistor and is also connected to a GPIO input. The 3 colored LEDs are connected to GPIO outputs through 220Ω current-limiting resistors.

For the server, we are using the provided Raspberry Pi Zero.

### Firmware

The fob and hub programs are both based on the IR RX/TX skill. They use the RMT library to generate a 38kHz signal and the UART library to generate and receive data signals through IR communications. 

#### Fob
The fob uses a GPIO interrupt to trigger a UART transmission of the following format when the button is pressed:

```
hub: [fob_id] [password]
```
Both of the above values are hardcoded. In a parallel task, the fob also listens for unlock signals of the following format:
```
unlock
```
If it receives an unlock message, it lights its green LED.
#### Hub

The hub listens for an incoming IR UART transmission beginning with the string "hub:". It also connects to wifi and sets up UDP communication using methods and libraries we've used in previous quests. Once it recognizes a valid IR message, it reformats this message into JSON of the following format, adding its own ID, and transmits it over UDP to the server:
```
{"fob_id": "[fob_id]", "hub_id": "[hub_id]", "code", "[code]"}
```
In a separate parallel task, the hub also listens for incoming UDP messages of the following formats:
```
granted
denied
```
If it receives "granted", it lights its green LED for a second and transmits "unlocked" over IR to the fob. If it receives anything else, including "denied", it lights its red LED for a second.

### Backend/Database

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
