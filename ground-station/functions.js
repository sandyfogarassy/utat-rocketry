var fs = require("fs"); 
const electron = require('electron');
const {ipcRenderer} = electron;
var CanvasJS = require('./node_modules/canvasjs/dist/canvasjs.min');

var data_points = [];
var input_conditions = [];
var input_params = [];
var input_mass;
var tank_measured_mass;
var liquid_measured_mass;
var flight_systems_check = false;
var igniter_check = false;
var data_points = [];

// Changing sensors/transducers
// edits readout values that are displayed in the pre-launch requirements
function editReadouts (changed_HTML_id, changed_HTML_inner, changed_li_id, changed_li_val) {
  document.getElementById(changed_HTML_id).innerHTML = changed_HTML_inner;
  document.getElementById(changed_li_id).value = changed_li_val;
}

// function that is called from the inline styling mainWindow.html
function reply_click(clicked_id) {
  var changed_HTML = "dv" + clicked_id[2];
  var changed_inner = document.getElementById(clicked_id).innerHTML;
  var list_index = "li" + clicked_id[2];
  var list_index_value = clicked_id[7];
  
  if (clicked_id[2] > 4) {
    list_index_value = Number(list_index_value) + 4;
  }

  editReadouts(changed_HTML, changed_inner, list_index, list_index_value);
}

// function to edit pre-launch requirements and plotted variables through setup window
function setup(conditions, params) {
  var setup_conditions = conditions;
  var setup_params = params;
  var j;
  var count;
  for (j=0;j<3;j++) {
    count = j + 2;
    var checkval = "checkval" + count.toString();
    var checkres = "checkres" + count.toString();
    if (setup_conditions[j] == "between") {
      if (j == 0) {
        document.getElementById("required_ox_pres").innerHTML = "Ox. Tank Pres: " + setup_params[j][0] + ", " + setup_params[j][1] + " PSI";
      }
      if (j == 1) {
        document.getElementById("required_temp1").innerHTML = "Liquid Temp 1: " + setup_params[j][0] + ", " + setup_params[j][1] + " C";
      }
      if (j == 2) {
        document.getElementById("required_temp2").innerHTML = "Liquid Temp 2: " + setup_params[j][0] + ", " + setup_params[j][1] + " C";
      }
      if (((Number(document.getElementById(checkval).innerHTML)) > (Number(setup_params[j][0]))) && ((Number(document.getElementById(checkval).innerHTML)) < (Number(setup_params[j][1])))) {
        document.getElementById(checkres).innerHTML = "GO";
        document.getElementById(checkres).style.color = "green";
        document.getElementById(checkval).style.color = "green";
      }
      else {
        document.getElementById(checkres).innerHTML = "STOP";
        document.getElementById(checkres).style.color = "red";
        document.getElementById(checkval).style.color = "red";
      }
    }
    if(input_conditions[j] == 'equal') {
      console.log(checkval);
      if (j == 0) {
        document.getElementById("required_ox_pres").innerHTML = "Ox. Tank Pres = " + setup_params[j] + "PSI";
      }
      if (j == 1) {
        document.getElementById("required_temp1").innerHTML = "Liquid Temp 1 =  " + setup_params[j] + "C";
      }
      if (j == 2) {
        document.getElementById("required_temp2").innerHTML = "Liquid Temp 2 =  " + setup_params[j] + "C";
      }
      if ((Number(document.getElementById(checkval).innerHTML)) == (Number(setup_params[j]))) {
        document.getElementById(checkres).innerHTML = "GO";
        document.getElementById(checkres).style.color = "green";
        document.getElementById(checkval).style.color = "green";
      }
      else {
        document.getElementById(checkres).innerHTML = "STOP";
        document.getElementById(checkres).style.color = "red";
        document.getElementById(checkval).style.color = "red";
      }
    }
    if (setup_conditions[j] == 'greater') {
      if (j == 0) {
        document.getElementById("required_ox_pres").innerHTML = "Ox. Tank Pres >  " + setup_params[j] + "PSI";
      }
      if (j == 1) {
        document.getElementById("required_temp1").innerHTML = "Liquid Temp 1 >  " + setup_params[j] + "C";
      }
      if (j == 2) {
        document.getElementById("required_temp2").innerHTML = "Liquid Temp 2 >  " + setup_params[j] + "C";
      }
      if ((Number(document.getElementById(checkval).innerHTML)) > (Number(setup_params[j]))) {
        document.getElementById(checkres).innerHTML = "GO";
        document.getElementById(checkres).style.color = "green";
        document.getElementById(checkval).style.color = "green";
      }
      else {
        document.getElementById(checkres).innerHTML = "STOP";
        document.getElementById(checkres).style.color = "red";
        document.getElementById(checkval).style.color = "red";
      }
    }
    if (setup_conditions[j] == 'less') {
      if (j == 0) {
        document.getElementById("required_ox_pres").innerHTML = "Ox. Tank Pres <  " + setup_params[j] + "PSI";
      }
      if (j == 1) {
        document.getElementById("required_temp1").innerHTML = "Liquid Temp 1 <  " + setup_params[j] + "C";
      }
      if (j == 2) {
        document.getElementById("required_temp2").innerHTML = "Liquid Temp 2 <  " + setup_params[j]; + "C";
      }
      if ((Number(document.getElementById(checkval).innerHTML)) < (Number(setup_params[j]))) {
        document.getElementById(checkres).innerHTML = "GO";
        document.getElementById(checkres).style.color = "green";
        document.getElementById(checkval).style.color = "green";
      }
      else {
        document.getElementById(checkres).innerHTML = "STOP";
        document.getElementById(checkres).style.color = "red";
        document.getElementById(checkval).style.color = "red";
      }
    }
  }
}

function avg(contents, length) {
  var accum = 0;
  for (var i in contents) {
    accum = accum + contents[i];
  }
  var result = accum/length;
  return (result);
}

function assigningReadouts(timestamp) {

  //read from JSON file
  var readFile = "./DAQ/test" + timestamp +".json";
  var contents = fs.readFileSync(readFile);
  var jsonContents = JSON.parse(contents);
  var daqContents = jsonContents.DAQ;
  var arduinoContents = jsonContents.Arduino; 

  //assigning individual readout values to sensor/transducer
  var pt1 = avg(daqContents.PT1, daqContents.PT1.length);
  document.getElementById("read5").innerHTML = pt1.toFixed(5);

  var pt2 = avg(daqContents.PT2, daqContents.PT3.length);
  document.getElementById("read6").innerHTML = pt2.toFixed(5);

  var pt3 = avg(daqContents.PT3, daqContents.PT3.length);
  document.getElementById("read7").innerHTML = pt3.toFixed(5);
  
  var thermo1 = avg(arduinoContents.Thermocouple1, arduinoContents.Thermocouple1.length);
  document.getElementById("read1").innerHTML = thermo1.toFixed(5);

  var thermo2 = avg(arduinoContents.Thermocouple2, arduinoContents.Thermocouple2.length);
  document.getElementById("read2").innerHTML = thermo2.toFixed(5);

  var thermo3 = avg(arduinoContents.Thermocouple3, arduinoContents.Thermocouple3.length);
  document.getElementById("read3").innerHTML = thermo3.toFixed(5);

  var thermo4 = avg(arduinoContents.Thermocouple4, arduinoContents.Thermocouple4.length);
  document.getElementById("read4").innerHTML = thermo4.toFixed(5);

}

// when the window loads, loop through this
window.onload = function () {

  //mass submission button
  document.getElementById("btn_submit").addEventListener('click', function submitMass(e) {
    e.preventDefault();
    var masstarget = [];
    masstarget.push('masstarget');
    masstarget.push(document.getElementById('mass_target').value);
    document.getElementById("mass_target").value = "";
    ipcRenderer.send('main', masstarget);
  });

  // pressing buttons

  document.getElementById("btn_dump").addEventListener('click', function statusDump(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Dumping...";
    document.getElementById("status").style.color = "yellow";
  });

  document.getElementById("btn_fill").addEventListener('click', function statusFill(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Filling...";
    document.getElementById("status").style.color = "yellow";
  });

  document.getElementById("btn_vent").addEventListener('click', function statusVent(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Venting...";
    document.getElementById("status").style.color = "yellow";
  });

  document.getElementById("btn_launch").addEventListener('click', function statusLaunch(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Arming Igniter...";
    document.getElementById("status").style.color = "yellow";
  });

  document.getElementById("btn_abort").addEventListener('click', function statusAbort(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Aborting...";
    document.getElementById("status").style.color = "yellow";
    window.open("abortWindow.html");
  });

  document.getElementById("btn_arm_umbilical").addEventListener('click', function statusArm(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Arming Umbilical...";
    document.getElementById("status").style.color = "yellow";
  });

  document.getElementById("btn_disconnect_umbilical").addEventListener('click', function statusDisconnect(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Disconnecting Umbilical...";
    document.getElementById("status").style.color = "yellow";
  });

  document.getElementById("btn_submit").addEventListener('click', function statusSubmit(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Submitting Target Mass...";
    document.getElementById("status").style.color = "yellow";
    functionDelay();
  });

  document.getElementById("check_igniter").addEventListener('click', function() {
    armIgniter();
  });

  val4 = "Input";
  val5 = "Input";
  val6 = "Input";
  val7 = "Input";

  //
  ipcRenderer.on('saved_setup', function (e, arg) {
    if (arg[0] == 'setup') {
      input_params = [];
      input_conditions = [];
      for (i=1;i<4;i++){
        input_conditions.push(arg[i][0]);
        input_params.push(arg[i][1]);
      }
      for (i=4;i<8;i++) {
        if (arg[i][1] == "temperature") {
          arg[i][1] = "Temperature (C)";
        }
        if (arg[i][1] == "pressure") {
          arg[i][1] = "Pressure (PSI)";
        }
        if (arg[i][1] == "time") {
          arg[i][1] = "Time (s)";
        }
        if (arg[i][1] == "mass rate") {
          arg[i][1] = "Mass Rate of Change (kg/s)";
        }
        if (arg[i][1] == "mass") {
          arg[i][1] == "Mass (kg)";
        }
      }
      console.log(arg);
      val4 = arg[4][1];
      val5 = arg[5][1];
      val6 = arg[6][1];
      val7 = arg[7][1];
      //document.getElementById("topxtitle").innerHTML = val4;
      //document.getElementById("topytitle").innerHTML = val5;
      //document.getElementById("botxtitle").innerHTML = val6;
      //document.getElementById("botytitle").innerHTML = val7;
    }

    else if (arg[0] == 'masstarget') {
      input_mass = Number(arg[1]);
      document.getElementById("required_mass").innerHTML = "Liquid Mass >= " + input_mass + "kg";
    }

    else if (arg == "abort_confirmed") {
      document.getElementById("status").innerHTML = "Abort Confirmed";
      document.getElementById("status").style.color = "red"
    }

    else if (arg == "abort_cancelled") {
      document.getElementById("status").innerHTML = "Abort Cancelled";
      document.getElementById("status").style.color = "green";
    }
  });

  function reload(n) {
    assignValuesToCheck();
    assigningReadouts(n);
    console.log(n);
  }

  //plots

  var chart1 = new CanvasJS.Chart("chartContainer1", {

    backgroundColor: null,
  	axisY: {
      gridThickness: 0,
      lineColor: "#00AF8B",
      labelFontColor: "#00AF8B",
      tickColor: "#00AF8B",
  		includeZero: false
  	},
    axisX: {
      lineColor: "#00AF8B",
      labelFontColor: "#00AF8B",
      tickColor: "#00AF8B"
    },
  	data: [{
  		type: "line",
      color: "#00AF8B",
  		dataPoints: data_points
  	}]
  });

  var chart2 = new CanvasJS.Chart("chartContainer2", {

    backgroundColor: null,
  	axisY: {
      gridThickness: 0,
      lineColor: "#00AF8B",
      labelFontColor: "#00AF8B",
      tickColor: "#00AF8B",
  		includeZero: false
  	},
    axisX: {
      lineColor: "#00AF8B",
      labelFontColor: "#00AF8B",
      tickColor: "#00AF8B"
    },
  	data: [{
  		type: "line",
      color: "#00AF8B",
  		dataPoints: data_points
  	}]
  });

  var xVal = 0;
  var yVal = 100;
  var updateInterval = 10000;
  var dataLength = 20; // number of dataPoints visible at any point

  var updateChart = function (count) {

  	count = count || 1;

  	for (var j = 0; j < count; j++) {
  		yVal = yVal +  Math.round(5 + Math.random() *(-5-5));
  		data_points.push({
  			x: xVal,
  			y: yVal
  		});
  		xVal++;
  	}

  	if (data_points.length > dataLength) {
  		data_points.shift();
  	}

  	chart1.render();
    chart2.render();

  };

  updateChart(dataLength);
  setInterval(function(){updateChart()}, updateInterval);

  //reloading the readout values based on the timestamped JSON files
  //var n represents the timestamp
  var n = 1;
  setInterval(function () {
    if (n > 3) {n = 3;}
    reload(n);
    n++;}, 
    1000
    );

}

function assignValuesToCheck() {

  var checklist = [];
  var i;
  var n;

  for (n=0;n<3;n++) {

    if (input_conditions[n] == 'between') {
      var str = input_params[n];
      var res = str.toString().split(",");
      input_params[n] = [res[0], res[1]];
    }
  }

  li1 = document.getElementById("li1").value;
  li2 = document.getElementById("li2").value;
  li3 = document.getElementById("li3").value;
  li4 = document.getElementById("li4").value;
  li5 = document.getElementById("li5").value;
  li6 = document.getElementById("li6").value;
  li7 = document.getElementById("li7").value;

  read1 = document.getElementById("read1").innerHTML;
  read2 = document.getElementById("read2").innerHTML;
  read3 = document.getElementById("read3").innerHTML;
  read4 = document.getElementById("read4").innerHTML;
  read5 = document.getElementById("read5").innerHTML;
  read6 = document.getElementById("read6").innerHTML;
  read7 = document.getElementById("read7").innerHTML;

  checklist.push([li1, read1], [li2, read2], [li3, read3], [li4, read4], [li5, read5], [li6, read6], [li7, read7]);

  for (i=0;i<7;i++) {
    if (checklist[i][0] == "5") {
      document.getElementById("checkval2").innerHTML = checklist[i][1];
    }
    if (checklist[i][0] == "1") {
      document.getElementById("checkval3").innerHTML = checklist[i][1];
    }
    if (checklist[i][0] == "2") {
      document.getElementById("checkval4").innerHTML = checklist[i][1];
    }
  }
  setup(input_conditions, input_params);
  liquid_measured_mass = 1000;
  if (input_mass != undefined) {
    document.getElementById("checkval5").innerHTML = Number(liquid_measured_mass);
    if (Number(liquid_measured_mass) >= Number(input_mass)) {
      document.getElementById("checkres5").innerHTML = "GO";
      document.getElementById("checkres5").style.color = "green";
      document.getElementById("checkval5").style.color = "green";
    }
    else {
      document.getElementById("checkres5").innerHTML = "STOP";
      document.getElementById("checkres5").style.color = "red";
      document.getElementById("checkval5").style.color = "red";
    }
  }
  flight_systems_check = true;
  igniter_check = true;
  if (flight_systems_check != undefined) {
    if (flight_systems_check == true) {
      document.getElementById("checkval6").innerHTML = "Continuous";
      document.getElementById("checkval6").style.color = "green";
      document.getElementById("checkres6").innerHTML = "GO";
      document.getElementById("checkres6").style.color = "green";
    }

    if (flight_systems_check != true) {
      document.getElementById("checkval6").innerHTML = "No Continuity";
      document.getElementById("checkval6").style.color = "red";
      document.getElementById("checkres6").innerHTML = "STOP";
      document.getElementById("checkres6").style.color = "red";
    }
  }

  if (igniter_check != undefined) {
    if (igniter_check == true) {
      document.getElementById("checkval7").innerHTML = "Continuous";
      document.getElementById("checkval7").style.color = "green";
      document.getElementById("checkres7").innerHTML = "GO";
      document.getElementById("checkres7").style.color = "green";
    }

    if (igniter_check != true) {
      document.getElementById("checkval7").innerHTML = "No Continuity";
      document.getElementById("checkval7").style.color = "red";
      document.getElementById("checkres7").innerHTML = "STOP";
      document.getElementById("checkres7").style.color = "red";
    }
  }
  tank_measured_mass = 1000;

  if (tank_measured_mass != undefined) {
    document.getElementById("checkval1").innerHTML = tank_measured_mass;
  }

  if ((document.getElementById("checkval2").style.color != "green") ||
  (document.getElementById("checkval3").style.color != "green") ||
  (document.getElementById("checkval4").style.color != "green") ||
  (document.getElementById("checkval5").style.color != "green") ||
  (document.getElementById("checkval6").style.color != "green") ||
  (document.getElementById("checkval7").style.color != "green")) {
    if (document.getElementById("btn_launch").classList.contains("btn_launch_enabled")) {
      alert("Igniter has been disarmed");
    }
    document.getElementById("btn_launch").className = "btn_launch_disabled";
  }
}

function functionDelay() {
  setTimeout(function() {
    document.getElementById("status").innerHTML = "";
  }, 2000);
}

function armIgniter() {
  if ((document.getElementById("checkval2").style.color == "green") &&
  (document.getElementById("checkval3").style.color == "green") &&
  (document.getElementById("checkval4").style.color == "green") &&
  (document.getElementById("checkval5").style.color == "green") &&
  (document.getElementById("checkval6").style.color == "green") &&
  (document.getElementById("checkval7").style.color == "green")) {
    document.getElementById("btn_launch").className = "btn_launch_enabled";
    document.getElementById("btn_launch").disabled = false;
  }

  else {
    document.getElementById("btn_launch").className = "btn_launch_disabled";
  }
}

