/** csv file
a,b,c
1,2,3
4,5,6
*/
const csvFilePath='sensor_data.csv'
const csv=require('csvtojson')
const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline');
const http = require('http');
const express = require('express')
const app = express()
const bodyParser = require('body-parser');

const fs = require('fs');

const FILE_NAME = 'data-write.json';
const NEW_DATA = [{ id: 2, name: 'Max' }];

const port = new SerialPort('/dev/cu.SLAB_USBtoUART', {
  autoOpen: false,
  baudRate: 115200
});


const path = 'sensor_data.csv'

var parser = port.pipe(new Readline({ delimiter: '\n' }));


const writeFileAsync = (newData) => {
  const stringifiedData = JSON.stringify(newData);

  fs.writeFile(FILE_NAME, stringifiedData, (error) => {
    if (error) {
      console.log('Async Write: NOT successful!');
      console.log(error);
    } else {
      console.log('Async Write: successful!');
      //console.log(stringifiedData);
    }
  });
};

////EXPRESS ROUTING////////////////////////////////////////////////////////////////

// set up express
app.use(express.static('public'))
app.use(bodyParser.urlencoded({ extended: true }));

// send html to front end
app.get('/', function (req, res) {

  console.log("loading!");

  fs.readFile('charts.html', function(err, data) {
      res.writeHead(200, {'Content-Type': 'text/html'});
      res.write(data);
      res.end();
  });

  //res.send();
});

// send sensor data formatted as json to front end
app.get('/data', function (req, res) {

  csv({
    checkType:true
  })
    .fromFile(path)
    .then(function(jsonArrayObj){ //when parse finished, result will be emitted here.
      //console.log(jsonArrayObj);

      res.send(jsonArrayObj);
    })

})

////SERIAL PORT READING////////////////////////////////////////////////////////////////

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

  var currTime = Math.floor(Date.now() / 1000)

  console.log(currTime + ", " + data);

  //add header if file missing or empty
  if (!fs.existsSync(path) || fs.statSync(path).size === 0) {
    fs.appendFile(path, "timestamp, battery, temperature, ultrasonic, infrared\n", function (err) {
      if (err) throw err;
      //console.log('Saved!');
    });
  }

  //write data to file, one dataset per line
  fs.appendFile(path, (currTime + ", " + data + '\n'), function (err) {
    if (err) throw err;
    //console.log('Saved!');
  });

});

////CSV CONVERSION////////////////////////////////////////////////////////////////


csv()
.fromFile(csvFilePath)
.then((jsonObj)=>{
  //  console.log(jsonObj);
  //console.log(jsonObj);
  writeFileAsync(jsonObj);

  /*  var chart1 = new CanvasJS.chart("chartContainer1"), {
      title: {
        text: "Live Data"
      }
      data: [{
        type: "line",
        name: "battery",
        datapoints : [
          { x: jsonObj[0][timestamp], y: jsonObj[0][battery] }
        ]
      }]
    }
    /**
     * [
     * 	{a:"1", b:"2", c:"3"},
     * 	{a:"4", b:"5". c:"6"}
     * ]
     */
})

// Async / await usage

const jsonArray= csv().fromFile(csvFilePath);


app.listen(3000, function () {
  console.log("connected");
})
