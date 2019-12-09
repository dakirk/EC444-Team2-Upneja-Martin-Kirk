//By Kyle Martin, David Kirk, Ayush Upneja

//Socket and server
var express = require('express');
var app = express();
var http = require('http').createServer(app);
var io = require('socket.io')(http);
var dgram = require('dgram');
var server = dgram.createSocket('udp4');

// File reader
const fs = require('fs');
const readline = require('readline');
var count = 0;
var header;

// Port and IP of Host
var PORT = 3333; // external is 3333
var HOST = "192.168.1.101"; // David's laptop is .101, Kyle's Laptop is .102, Pi is .122, Ayush's Laptop is .142
const code = "smartkey";

//Port and IP of Device
var devPORT = 3333;
var devHOST = "192.168.1.129"

// TingoDB
var Engine = require('tingodb')(),
    assert = require('assert');

// Create DB
var db = new Engine.Db('.', {});
var users = db.collection("users_v4");
var logs = db.collection("logs");

var dirs = [0, 0, 1];

// Publish HTML file
app.get('/', function(req, res){
  res.sendFile(__dirname + '/main.html');
});

app.get('/logs', function(req, res){
   logs.find().toArray(function(err, result) {
     if (err) throw err;
     res.send(result);
   });
});

app.post('/speed', function(req, res) {
  var speed = req.body.speed;
  dirs[0] += speed;


  console.log("speed: " + speed);
  server.send(dirs, devPORT, devHOST, function(error){});
});

app.post('/steer', function(req, res) {
  var steer = req.body.steer;
  dirs[1] += steer;

  console.log("steer: " + steer);
  server.send(dirs, devPORT, devHOST, function(error){});
});

app.post('/status', function(req, res) {
  var status = req.body.status;
  dirs[2] = status;

  console.log("status: " + status);
  server.send(dirs, devPORT, devHOST, function(error){});
});

// Send sensor readings to frontend and write JSON to local file
server.on('message', function (message, remote) {

  var splitTime = parseFloat(message);

  console.log(devHOST + ":" + devPORT + " time: " + splitTime);

  /*
  // Parse message into JSON object
  var request = JSON.parse(message.toString());
  // Update device port and host
  devPORT = remote.port;
  devHOST = remote.address;
  // Query the fobID in the user database
  users.findOne({"fobID": request["fob_id"]}, function(err, item) {
    // if fobID and code are valid
    if (request["code"] == code && item != null) {
      server.send("granted",devPORT,devHOST,function(error){});
      var d = new Date();
      var info = {"name": item["name"], "fobID": request["fob_id"], "hubID": request["hub_id"], "timestamp": d.toLocaleString(), "access": "granted"};
      // log new key access request
      logs.insert(info);
      console.log(info);
      io.emit("message", info);
    // if fobID and code are invalid
    } else {
      server.send("denied",devPORT,devHOST,function(error){});
      var d = new Date();
      if (item == null) {
        var info = {"name": "unknown", "fobID": request["fob_id"], "hubID": request["hub_id"], "timestamp": d.toLocaleString(), "access": "denied"};
      } else {
        var info = {"name": item["name"], "fobID": request["fob_id"], "hubID": request["hub_id"], "timestamp": d.toLocaleString(), "access": "denied"};
      }
      // log new key access request
      logs.insert(info);
      console.log(info);
      io.emit("message", info);
    }
  })
  */
});

// User socket connection
io.on('connection', function(socket){
  console.log('a user connected');
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

// Listening on port 3000
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
