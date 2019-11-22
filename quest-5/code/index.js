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
var HOST = "192.168.1.122"; // Kyle's Laptop is .102, Pi is .122
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

// Create readstream
//const readInterface = readline.createInterface({
//    input: fs.createReadStream('users.txt'),
    //output: process.stdout,
//    console: false
//});

// Read each line
//readInterface.on('line', function(line) {
//    if (count == 0) {
//      header = line.split("\t");
//      count = count + 1;
//    } else {
//      var vals = line.split("\t");
//      var data = {};
//      data[header[0]] = vals[0];
//      data[header[1]] = vals[1];
      // Insert data
//      console.log(data);
//      users.insert(data);
//    }
//});
users.findOne({"fobID": "hjackson"}, function(err, item) {
  console.log(item);
})
users.findOne({"fobID": "kmartin"}, function(err, item) {
  console.log(item["name"]);
})
users.findOne({"fobID": "aupneja"}, function(err, item) {
  console.log(item["name"]);
})

// Send sensor readings to frontend and write JSON to local file
server.on('message', function (message, remote) {
    // Send sensor data through TCP socket
    var request = JSON.parse(message.toString());
    // Update device port and host
    devPORT = remote.port;
    devHOST = remote.address;
    // check if creds are valid,log the request, and send unlock message if creds are valid
    //users.findOne({"fobID": "dkirk"}, function(err, item) {
    //  if (item != undefined && code == item["code"]) {
	//console.log(item);
        //server.send("unlock",devPORT,devHOST,function(error){});
        //var d = new Date(); 
        //logs.insert({"name": item["name"], "fobID": request["fob_id"], "hubID": request["hub_id"], "timestamp": d.toLocaleString(), "access": "granted"});
        //io.emit("message", logs);
      //} else {
	//console.log("undefined");
        //var d = new Date(); 
        //logs.insert({"name": "unknown", "fobID": request["fob_id"], "hubID": request["hub_id"], "timestamp": d.toLocaleString(), "access": "denied"});
        //io.emit("message", logs);
      //}
    //})
  users.findOne({"fobID": request["fob_id"]}, function(err, item) {
    if (request["code"] == code && item != null) { 
      server.send("unlock",devPORT,devHOST,function(error){});
      var d = new Date();
      var info = {"name": item["name"], "fobID": request["fob_id"], "hubID": request["hub_id"], "timestamp": d.toLocaleString(), "access": "granted"}; 
      logs.insert(info);
      io.emit("message", info);
      console.log(info);
    } else {
      var d = new Date();
      if (item == null) { 
        var info = {"name": "unknown", "fobID": request["fob_id"], "hubID": request["hub_id"], "timestamp": d.toLocaleString(), "access": "denied"};
      } else {
        var info = {"name": item["name"], "fobID": request["fob_id"], "hubID": request["hub_id"], "timestamp": d.toLocaleString(), "access": "denied"};
      }
      logs.insert(info);
      console.log(info);
    }
  })
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
