/** csv file
a,b,c
1,2,3
4,5,6
*/
const csvFilePath='sensor_data.csv'
const csv=require('csvtojson')

const fs = require('fs');

const FILE_NAME = 'data-write.json';
const NEW_DATA = [{ id: 2, name: 'Max' }];

const writeFileAsync = (newData) => {
  const stringifiedData = JSON.stringify(newData);

  fs.writeFile(FILE_NAME, stringifiedData, (error) => {
    if (error) {
      console.log('Async Write: NOT successful!');
      console.log(error);
    } else {
      console.log('Async Write: successful!');
      console.log(stringifiedData);
    }
  });
};



csv()
.fromFile(csvFilePath)
.then((jsonObj)=>{
  //  console.log(jsonObj);
  console.log(jsonObj);
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
