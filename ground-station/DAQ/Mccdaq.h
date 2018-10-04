#ifndef MCCDAQ_H
#define MCCDAQ_H

#include <C:\Users\Public\Documents\measurement_computing\DAQ\C\cbw.h>
#include <C:\Users\Public\Documents\measurement_computing\DAQ\C\cbw32.lib>
#include <json/writer.h>
#include <iostream>
#include <fstream> 
#include <json.h> 

class Daq {
	//Parent class for the entire program
	public: 

		

	private: 
	protected: 
}

using namespace std;

void save_file() {

}

void write_to_JSON() {

    Json::Value event;
    Json::Value vec(Json::arrayValue);

    //Putting DAQ feedback into JSON objects
    event["DAQ"]["PT"]["code"] = "Ox. Temp 1";
    event["DAQ"]["PT"]["value"] = "600psi";

    //Putting arduino feedback into JSON objects
    event["arduino"]["Load Cell"]["code"] = "Thrust Load Cell";
    event["arduino"]["Load Cell"]["value"] = "2000N";

    //Writing data to JSON file
    std::ofstream file;
    file.open("data_00001.json");
    Json::StyledWriter styledWriter;
    file << styledWriter.write(event);
    file.close();

}

void main() {


}
