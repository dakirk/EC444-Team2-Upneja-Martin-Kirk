# Quest 5 Plan
Feel free to edit this as necessary. I'm just writing this as guidelines.

## Tasks
### Done
* Database basics
* Raspberry Pi setup
* IR RX/TX with UART
### To do
* Build multiple IR transmitter/receiver devices
* Write fob firmware which transmits its ID and an unlocking code and receives unlocking signals over IR
* Write hub firmware which receives a fob's ID and unlocking code over IR and transmits this data with its hub ID to the server over UDP
* Set up a database suitable for storing multiple hubs and fobs on the Raspberry Pi
* Determine a mechanism for assigning unique, permanent IDs to multiple hubs and fobs (hard-code?)
### Nice to have
* Test the system with multiple fobs and hubs

## Interfaces

#### Any data with an ESP32 as its destination should have a simple format, like values separated by spaces. In the following interfaces, "fob_id", "code", and "unlock_msg" are variables representing data values.

Fob -> Hub (IR):
```
fob_id code //both data elements are separated by spaces for easy parsing
```
Hub -> Fob (IR):
```
unlock_msg
```
Server -> Hub (UDP):
```
fob_id unlock_msg
```

#### Client- or server-bound data can be formatted with JSON.

Hub -> Server (UDP):
```
{fob_id, hub_id, code}
```
Server -> Client (TCP):

TBD

Client -> Server (TCP):

TBD


####Database queries should contain fob_id, hub_id, and code. Not yet sure what format they need to be in, or if the database is controlled directly by the Node server.
