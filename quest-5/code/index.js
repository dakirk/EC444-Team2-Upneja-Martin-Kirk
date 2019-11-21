//By Kyle Martin, David Kirk, Ayush Upneja

//Socket and server
var express = require('express');
var app = express();
var http = require('http').createServer(app);
var io = require('socket.io')(http);
var dgram = require('dgram');
var server = dgram.createSocket('udp4');

// Port and IP of Host
var PORT = 3333; // external is 3333
var HOST = "192.168.1.102"; // Kyle's Laptop is .102, Pi is .122

//Port and IP of Device
var devPORT = 3333;
var devHOST = "192.168.1.129"

// File reader
const fs = require('fs');
const readline = require('readline');
var count = 0;
var header;

// TingoDB
var Engine = require('tingodb')(),
    assert = require('assert');

// Create DB
var db = new Engine.Db('.', {});
var users = db.collection("users");
var logs = db.collection("logs");

// Create readstream
const readInterface = readline.createInterface({
    input: fs.createReadStream('users.txt'),
    //output: process.stdout,
    console: false
});

// Read each line
readInterface.on('line', function(line) {
    if (count == 0) {
      header = line.split("\t");
      count = count + 1;
    } else {
      var vals = line.split("\t");
      var data = {};
      data[header[0]] = vals[0];
      data[header[1]] = vals[1];
      // Insert data
      users.insert(data);
    }
});

// Upon close, read the database
readInterface.on('close', function(line) {
  users.findOne({}, function(err, item) {
      assert.equal(null, err);
      console.log(item);
  })
});

// Send sensor readings to frontend and write JSON to local file
server.on('message', function (message, remote) {
    // Send sensor data through TCP socket
    var request = JSON.parse(message.toString());
    // Update device port and host
    devPORT = remote.port;
    devHOST = remote.address;
    // check if creds are valid,log the request, and send unlock message if creds are valid
    users.findOne({"fobID": request["fob_id"], "code": request["code"]}, function(err, item) {
      assert.equal(null, err);
      if (item == null) {
        server.send("unlock",devPORT,devHOST,function(error){});
        var d = new Date(); 
        logs.insert({"name": request["name"], "fobID": request["fob_id"], "hubID": request["hub_id"], "timestamp": d.toLocaleString(), "location": request["location"]}, "access": "granted");
        io.emit("message", logs)
      } else {
        var d = new Date(); 
        logs.insert({"name": request["name"], "fobID": request["fob_id"], "hubID": request["hub_id"], "timestamp": d.toLocaleString(), "location": request["location"]}, "access": "denied");
        io.emit("message", logs)
      }
    })
});


