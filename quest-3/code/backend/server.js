// Modules /////////////////////////////////////////////////////////////////////////////////////////////
var express = require('express');
var app = require('express')();
var http = require('http').createServer(app);
var io = require('socket.io')(http);
var dgram = require('dgram');
var bodyParser = require('body-parser');

// Port and IP
var PORT = 3333; //internal is 1236, external is 3333
var HOST = "192.168.1.102"; //Kyle's Laptop is .102, Pi is .122

// Create socket
var server = dgram.createSocket('udp4');

const fs = require('fs');

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

// Update device state
app.post('/updatedevice', function(req, res){
    var str = req.body.state;
    server.send(str,remote.port,remote.address,function(error){
      if(error){
        console.log('Found device: ' + str);
      }
      else{
        console.log('Failed to update state');
      }
    });
});

// Find device
app.post('/finddevice', function(req, res){
  var str = req.body.state;
    server.send(str,remote.port,remote.address,function(error){
      if(error){
        console.log('Found device: ' + str);
      }
      else{
        console.log('Failed to find device');
      }
    });
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

// Link JSON parser to express
app.use( bodyParser.json() ); 














