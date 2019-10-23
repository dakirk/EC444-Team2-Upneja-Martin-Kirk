//code adapted from https://medium.com/@machadogj/arduino-and-node-js-via-serial-port-bcf9691fab6a and the serialport documentation

//IMPORTS AND SETUP//////////////////////////////////////////////////////////////////

const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline');
const http = require('http');
const fs = require('fs');
const express = require('express')
const app = express()
const bodyParser = require('body-parser');
const csvtojson = require("csvtojson");
const XMLHttpRequest = require('xmlhttprequest').XMLHttpRequest;

//set up serial port
const port = new SerialPort('/dev/cu.SLAB_USBtoUART', { 
  autoOpen: false,
  baudRate: 115200
});

//useful constants
const path = 'sensor_data.csv' //temporary CSV file
const csvheader = "timestamp, battery, temperature, ultrasonic, infrared\n" //CSV header

//working variables
var parser = port.pipe(new Readline({ delimiter: '\n' }));
var serialline = "not yet set"
var secondsSinceStartup = 0;

// set up express
app.use(express.static('public'))
app.use(bodyParser.urlencoded({ extended: true }));

//EXPRESS ROUTING (GET REQUEST RESPONSES)//////////////////////////////////////////////////////////////////


// send html to front end
app.get('/', function (req, res) {

  fs.readFile('charts.html', function(err, data) {
      console.log("sending data");
      res.writeHead(200, {'Content-Type': 'text/html'});
      res.write(data);
      res.end();
  }); 

});

// send sensor data formatted as json to front end
app.get('/data', function (req, res) {

  csvtojson({
    checkType:true
  })
    .fromFile(path)
    .then(function(jsonArrayObj){ //when parse finished, result will be emitted here.
      //console.log(jsonArrayObj); 

      res.send(jsonArrayObj);
    })

})

// send latest sensor reading formatted as json to front end
app.get('/serialline', function (req, res) {

  csvtojson({
    checkType:true
  })
  .fromString(csvheader + serialline)
  .then((jsonArrayObj)=>{ 
      res.send(jsonArrayObj)
  })

})

//SERIAL READING//////////////////////////////////////////////////////////////////

//start reading from serial port
port.open(function (err) {
  if (err) {
    return console.log('Error opening port: ', err.message);
  }
 
  // Because there's no callback to write, write errors will be emitted on the port:
  port.write('main screen turn on');
});
 
// Read data that is available but keep the stream from entering "flowing mode"
port.on('readable', function () {
  port.read();
});

console.log("Timestamp (s), Battery voltage (mV), temperature (C), ultrasonic distance (cm), infrared distance (cm)");

// Get sensor data in csv format and save it to a file
parser.on('data', data =>{

  serialline = secondsSinceStartup + ", " + data;


  console.log(serialline);

  //write data to file, one dataset per line
  fs.appendFile(path, (serialline + '\n'), function (err) {
    if (err) throw err;
    //console.log('Saved!');
  });

  secondsSinceStartup++;

});

//STARTUP AND SHUTDOWN//////////////////////////////////////////////////////////////////

//handle startup
app.listen(3000, function () {
  console.log("connected");

  //create a temporary CSV file
  fs.appendFile(path, csvheader, function (err) {
    if (err) throw err;
  });
})

//handle shutdown
process.on( 'SIGINT', function() {

  console.log( "\nGracefully shutting down from SIGINT (Ctrl-C)" );

  //delete the temporary CSV file
  fs.unlinkSync(path)

  // some other closing procedures go here
  process.exit( );
})
