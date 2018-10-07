#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <C:\Users\sandy\OneDrive\Desktop\utat-rocketry\ground-station\DAQ\jsoncpp\dist\jsoncpp.cpp>
#include <C:\Users\sandy\OneDrive\Desktop\utat-rocketry\ground-station\DAQ\jsoncpp\dist\json\json.h>
#include <C:\Users\sandy\OneDrive\Desktop\utat-rocketry\ground-station\DAQ\jsoncpp\dist\json\json-forwards.h>

//#include "Mccdaq.h"

void write_to_JSON() {

	//initializing overarching JSON object
	Json::Value data;   

	//initializing JSON vector
    Json::Value vec(Json::arrayValue);

	//testing output of vector
    vec.append(Json::Value(1));
    vec.append(Json::Value(2));
    vec.append(Json::Value(3));

	//JSON will have two branches underneath data, one for the DAQ, and one for the arduino
    data["DAQ"]["PT"] = "test";
	data["DAQ"]["Load Cell"] = "test";
    data["Arduino"]["Thermocouple"] = "test";
    data["Arduino"]["Load Cell"]=vec;

    std::cout << "SUCCESS" << std::endl;

	std::ofstream ofs ("test.json", std::ofstream::app);
    ofs << data << std::flush;

}

int main() {

	write_to_JSON();
	return 0;
	
}