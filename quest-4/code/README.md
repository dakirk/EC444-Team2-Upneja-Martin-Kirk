# Code Readme

Brief explanation on how to navigate your code folder. For example, main consists of the entry function, and lib are where the resource libraries are located.

NOTE FOR UDP:
If you're using the socket-tester UDP server from quest 3, you need to start that server BEFORE starting the crawler. Also, as always, disable your firewall before trying to use UDP, and be sure to change the crawler's IP target and port to socket-tester's IP and port.

Commands:
* start - starts the rover
* stop - stops the rover (cannot be restarted)

Roles:
- Kyle Martin: PID control & Calibration
- David Kirk: Sensors & Actuation
- Ayush Upneja: Socket Connection & LIDAR setup

Relevant Files:
- quest-3/code/socket-tester: used to send messages to the ESP32
- crawler-control/main/crawler-control.c: main ESP file
