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

	//initializing JSON vectors
    Json::Value tc1(Json::arrayValue);
	Json::Value tc2(Json::arrayValue);
	Json::Value tc3(Json::arrayValue);
	Json::Value tc4(Json::arrayValue);

	Json::Value pt1(Json::arrayValue);
	Json::Value pt2(Json::arrayValue);
	Json::Value pt3(Json::arrayValue);

	//testing output of vectors
    tc1.append(Json::Value(52));
    tc1.append(Json::Value(55));
    tc1.append(Json::Value(50));
	tc1.append(Json::Value(51));
    tc1.append(Json::Value(51));
    tc1.append(Json::Value(50));

	tc2.append(Json::Value(49));
    tc2.append(Json::Value(50));
    tc2.append(Json::Value(49));
	tc2.append(Json::Value(49));
    tc2.append(Json::Value(49));
    tc2.append(Json::Value(49));

	tc3.append(Json::Value(48));
    tc3.append(Json::Value(50));
    tc3.append(Json::Value(51));
	tc3.append(Json::Value(51));
    tc3.append(Json::Value(51));
    tc3.append(Json::Value(50));

	tc4.append(Json::Value(52));
    tc4.append(Json::Value(50));
    tc4.append(Json::Value(51));
	tc4.append(Json::Value(51));
    tc4.append(Json::Value(51));
    tc4.append(Json::Value(50));

	pt1.append(Json::Value(600));
    pt1.append(Json::Value(611));
    pt1.append(Json::Value(602));
	pt1.append(Json::Value(604));
    pt1.append(Json::Value(605));
    pt1.append(Json::Value(606));

	pt2.append(Json::Value(600));
    pt2.append(Json::Value(609));
    pt2.append(Json::Value(600));
	pt2.append(Json::Value(602));
    pt2.append(Json::Value(603));
    pt2.append(Json::Value(601));

	pt3.append(Json::Value(599));
    pt3.append(Json::Value(602));
    pt3.append(Json::Value(602));
	pt3.append(Json::Value(604));
    pt3.append(Json::Value(605));
    pt3.append(Json::Value(601));

	//JSON will have two branches underneath data, one for the DAQ, and one for the arduino
    data["DAQ"]["PT1"] = pt1;
	data["DAQ"]["PT2"] = pt2;
	data["DAQ"]["PT3"] = pt3;
	data["DAQ"]["Load Cell"] = "test";
    data["Arduino"]["Thermocouple1"] = tc1;
	data["Arduino"]["Thermocouple2"] = tc2;
	data["Arduino"]["Thermocouple3"] = tc3;
	data["Arduino"]["Thermocouple4"] = tc4;
    data["Arduino"]["Load Cell"] = "test";

    std::cout << "SUCCESS" << std::endl;

	std::ofstream ofs ("test.json", std::ofstream::app);
    ofs << data << std::flush;

}

int main() {

	write_to_JSON();
	return 0;
	
}