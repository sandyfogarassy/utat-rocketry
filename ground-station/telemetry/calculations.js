

var initialPressure = 1013.25 //hPa
var pressure = 1038465 //some abstract number that represents the bytes pulled from the pressure sensor
var pressure_scale = 1 //some constant that relates the abstract pressure bytes to pressure
var pressure_timestamp = 60000 //ms
var gravity = 9.81 //m/s

function duration(initialTime, finalTime) { //units of s
	return ((finalTime - initialTime) / 1000)
}

function bundle(offset = 0) {
	//return dictionary of all most recent values
}

function altitude(pressure) {}

function velocity(initialTime, initialAltitude, finalTime, finalAltitude) { //units of m/s
	return ((finalAltitude - initialAltitude) / duration(initialTime, finalTime))
}

function accelerationFromPressure(initialTime, initialVelocity, finalTime, finalVelocity) { //units of m/s/s
	return ((finalVelocity - initialVelocity) / duration(initialTime, finalTime))
}

function force(acceleration) { //units of G's
	return ((acceleration / gravity) + 1)
}
