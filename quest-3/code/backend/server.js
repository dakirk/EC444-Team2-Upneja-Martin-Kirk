// Modules /////////////////////////////////////////////////////////////////////////////////////////////
var express = require('express');
var app = express();
var http = require('http').createServer(app);
var io = require('socket.io')(http);
var dgram = require('dgram');
var bodyParser = require('body-parser');

// create application/json parser
var jsonParser = bodyParser.json();

// create application/x-www-form-urlencoded parser
var urlencodedParser = bodyParser.urlencoded({ extended: false });

// Port and IP
var PORT = 3333; //internal is 1236, external is 3333
var HOST = "192.168.1.102"; //Kyle's Laptop is .102, Pi is .122

// Create socket
var server = dgram.createSocket('udp4');

//file read
const fs = require('fs');

//Global state variable [vibration, temperature, battery, water, ping, time]
var state = "111111800";

//API ENDPOINTS/////////////////////////////////////////////////////////////////////////////////////////

// Publish HTML file
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});

//Obtain all stored sensor readings
app.get('/alldata', function(req, res){
  fs.readFile('data.json', (err, data) => {
    if (err) throw err;
    let all = JSON.parse(data);
    res.send(all);
  });
});

// Update vibration state
app.post('/vibration', jsonParser, function(req, res){
    var num = req.body.state;
    state = num.toString(10) + state.slice(1, 9);
    server.send(state,remote.port,remote.address,function(error){
      if(error){
        console.log('Found device');
      }
      else{
        console.log('Failed to update state');
      }
    });
    res.send(state)
});

// Update temperature state
app.post('/temperature', jsonParser, function(req, res){
    var num = req.body.state;
    state = state[0] + num.toString(10) + state.slice(2, 9);
    server.send(state,remote.port,remote.address,function(error){
      if(error){
        console.log('Found device');
      }
      else{
        console.log('Failed to update state');
      }
    });
    res.send(state)
});

// Update battery state
app.post('/battery', jsonParser, function(req, res){
    var num = req.body.state;
    state = state.slice(0, 2) + num.toString(10) + state.slice(3, 9);
    server.send(state,remote.port,remote.address,function(error){
      if(error){
        console.log('Found device');
      }
      else{
        console.log('Failed to update state');
      }
    });
    res.send(state)
});

// Update water state
app.post('/water', jsonParser, function(req, res){
    var num = req.body.state;
    state = state.slice(0, 3) + num.toString(10) + state.slice(4, 9);
    server.send(state,remote.port,remote.address,function(error){
      if(error){
        console.log('Found device');
      }
      else{
        console.log('Failed to update state');
      }
    });
    res.send(state)
});

// Update ping state
app.post('/ping', jsonParser, function(req, res){
    var num = req.body.state;
    state = state.slice(0,4) + num.toString(10) + state.slice(5, 9);
    server.send(state,remote.port,remote.address,function(error){
      if(error){
        console.log('Found device');
      }
      else{
        console.log('Failed to update state');
      }
    });
    res.send(state)
});

// Update time interval
app.post('/time', jsonParser, function(req, res){
    var num = req.body.state;
    state = state.slice(0,5) + num.toString(10);
    server.send(state,remote.port,remote.address,function(error){
      if(error){
        console.log('Found device');
      }
      else{
        console.log('Failed to update state');
      }
    });
    res.send(state)
});

//SOCKET MESSAGE READER/////////////////////////////////////////////////////////////////////////////////

// Send sensor readings to frontend
server.on('message', function (message, remote) {
    io.emit('message', message);
    // Send "Got data" acknowledgement
    server.send("Got data",remote.port,remote.address,function(error){
      if(error){
        console.log('Did not send: Got data');
      }
      else{
        console.log('Sent: Got data');
      }
    });
});

//HOST, SOCKET, AND EXPRESS INITIALIZATIONS/////////////////////////////////////////////////////////////

// User socket connection
io.on('connection', function(socket){
  console.log('a user connected');
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

// Listening on localhost:3000
http.listen(3000, function() {
  console.log('listening on *:3000');
});

// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// Bind server to port and IP
server.bind(PORT, HOST);













