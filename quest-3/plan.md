api endpoints
- step on/off (post)
- battery monitor on/off (post)
- temp on/off (post)
- water frequency (post)
- find device (post)
- all sensor data (get)
- publish html (get)
- most recent data (get)

example socket line outbound (esp -> node, sent at regular intervals):
* '{"steps": 123, "battery": 1234, "thermistor": 25}'

example socket line inbound (node -> esp, sent when user requests change):
* '{"steps": true, "battery": true, "thermistor": false, "wateralarm": true, "finddevice": true}'

or 

* '1 1 0 1 1' where each digit is a flag -- would be easier to parse
