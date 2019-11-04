// Modules /////////////////////////////////////////////////////////////////////////////////////////////
var express = require('express');
var app = express();
var bodyParser = require('body-parser');
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
    console.log("YOOOOOOOO: ", str);
});

app.listen(3000, () => {
 console.log("Server running on port 3000");
});

// Link JSON parser to express
app.use( bodyParser.json() ); 
app.use(bodyParser.urlencoded({ extended: false }));












