// Required module
var dgram = require('dgram');
var stdin = process.stdin;

// Port and IP
var PORT = 3333;
var HOST = '192.168.1.101'//'192.168.43.119';

var remoteport = 8080;
var remoteaddress = '123.456.7.890';

// Create socket
var server = dgram.createSocket('udp4');

//set up stdin
stdin.setEncoding('utf-8');

// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);

    process.stdout.write("Your message: ");
});

// On connection, print out received message
server.on('message', function (message, remote) {
	//process.stdout.clearLine();
	//process.stdout.cursorTo(0);
  //console.log(remote.address + ':' + remote.port +' - ' + message);

  remoteport = remote.port;
  remoteaddress = remote.address;
  //process.stdout.write("Your message: ");

    // Send Ok acknowledgement

    /*
	server.send("meep",remote.port,remote.address,function(error){
      if(error){
      	//throw error
        console.log('MEH!');
      }
      else{
        console.log('Sent: Ok!');
      }
    });*/
});

stdin.on('data', function(data) {

	server.send(data, remoteport, remoteaddress, function(error) {
	if(error){
      	//throw error
        console.log('MEH!');
      }
      else{
        //console.log('Sent: ' + data);
        process.stdout.write("Your message: ");

      }
	});

	//console.log("ip: " + remoteaddress + ":" + remoteport);

});

server.on('error', (err) => {
  console.log(`server error:\n${err.stack}`);
  server.close();
});



// Bind server to port and IP
server.bind(PORT, HOST);