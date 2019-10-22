/* This is an instantiation of a server that we designed to:

-- Run on a local machine (laptop)
-- Iterate and loop from 0 to 10 on the server-side
-- Serve up a webpage to clients accessing the server
-- Communication between server (backend) and HTML client (frontend) is done using socket.io module

-- server side

October 2018 -- Emily Lam
Updated September 2019
*/


// Modules
var express = require('express');
var app = require('express')();
var http = require('http').createServer(app);
var io = require('socket.io')(http);

// Iterate from 0 to 10 (then loop) at 1 second intervals
var i = 0;
setInterval( function() {
  console.log('Read:', i);    // Log to console
  io.emit('message', i);      // Emit to message event
  i++;
  if (i > 10) {i=0;}
}, 1000);

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});

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
