/*
 * The purpose of this code is to take readings from the GPS Sensor (in this case Ublox NEO-6M),
 * process them into readable data (optional, as can be done later on the ground), and compress the data so as to
 * remove all the useless crap and be able to store the data as efficiently as possible without losing much precision.
 * 
 * Processing, compressing is needed as the gps sends the data as long ass strings where data is sent as ascii strings + a bunch of useless boilerplate code
 * which is not optimal for storing or transmitting the data.
 * 
 * Data is from the GPS is sent with NMEA protocol.
 */
// atm, readings that will be used are the time, altitude, and coordinates
// maybe speed?
#include <SoftwareSerial.h>
#define baud_rate 9600
// serial pins for gps
#define RX_pin 3
#define TX_pin 5

SoftwareSerial gps_st(RX_pin, TX_pin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  gps_st.begin(baud_rate);
}

// Crap for reading our gps is below

// current task function pointer - assigned to a specific function to handle each type
struct GPS_Data {
  char time[20];
  
  char long_coords[20];
  char lat_coords[20];
  char sealvl_alt[20];
} gps_data;
void clear_data() {
  memset(&gps_data, 0, sizeof(gps_data));
  gps_data.time[0] = (char)(-1);
  gps_data.long_coords[0] = (char)(-1);
  gps_data.lat_coords[0] = (char)(-1);
  gps_data.sealvl_alt[0] = (char)(-1);
}

void (*type_handler)(char data) = NULL; // function addressed here is a sentence specific handler to get the data we need from the string and commit appropriately

char field_data[20]; // current field data
int fieldLen = 0; // number of chars read from current field
int field_no = 0; // current field starting from 1 in the current sentence
void clear_field() { // reset data stored from current field (make it empty)
  memset(field_data,0,fieldLen);
  fieldLen = 0;
}
bool checksum_en = false; // used by type handlers
byte parity = 0; // used for checksum at end of sentence
bool run_checksum() { // at end of sentences for decision whether to commit
  // loop through all chars in field and convert to hex
  byte result;
  for (int i = 0; i < fieldLen; i++) {
    // shift current number in result to next hex "column"
    result *= 16;
    char data = field_data[i];
    if ('0' <= data && data <= '9') {
      result += data - '0';
    } else if ('A' <= data && data <= 'F') {
      result += data - 'A' + 10;
    }
  }
  // result depends on whether parity and checksum value received are the same
  return parity == result;
}

void GGA_handler(char data) { // UTC, coordinates, quality, number of satellites tracked, altitude
    if (data == '\r' || data == '\n') { // signifies end of sentence - time to run checksum
      // however, if final checksum process was not started, this sentence must be garbage
      type_handler = NULL; // end of sentence in any checksum scenario
      if (!checksum_en) {
        // don't even bother with any checksums
        return;
      }
      if (run_checksum()) { // if this part is ever entered it means the checksum was successful and we're committing data
        Serial.println("Successful checksum!");
        // check if a value for the following data fields has been recorded
        if (gps_data.time[0] != -1) {
          Serial.print("Time: "); Serial.println(gps_data.time);
        }
        if (gps_data.long_coords[0] != -1) {
          Serial.print("Longitude: "); Serial.println(gps_data.long_coords);
        }
        if (gps_data.lat_coords[0] != -1) {
          Serial.print("Latitude: "); Serial.println(gps_data.lat_coords);
        }
        if (gps_data.sealvl_alt[0] != -1) {
          Serial.print("Altitude above sea level: "); Serial.println(gps_data.sealvl_alt);
        }
      } else {
        Serial.println("Houston, we've got a problem!");
      }
      return;
    } else if (data == '*') { // start of checksum, end of current field
      field_no++;
      checksum_en = true;
      return; // no point in running rest of function
    } else if (data == ',') { // indicates end of field
      // store data accordingly based on current field number
      if (field_no == 2) { // second field number is time
        memcpy(gps_data.time, field_data, fieldLen);
      } else if (field_no ==  3) { // longitude field
        memcpy(gps_data.long_coords, field_data, fieldLen);
      } else if (field_no == 5) { // latitude field
        memcpy(gps_data.lat_coords, field_data, fieldLen);
      } else if (field_no == 4) { // indicates whether the longitude coords are north or south
        gps_data.long_coords[strlen(gps_data.long_coords)] = field_data[0]; // place the char at the end of the string to indicate coord direction (N or S)
      } else if (field_no == 6) { // whether latitude coords are west or east
        gps_data.lat_coords[strlen(gps_data.lat_coords)] = field_data[0]; // char at end to indicate coord direction (W or E)
      } else if (field_no == 10) { // altitude field
        memcpy(gps_data.sealvl_alt, field_data, fieldLen);
      }
      field_no++;
      clear_field();
      parity ^= data;
    } else { // just add the data to the field if none of above criteria apply
      // also run parity check if final checksum stage is not enabled
      field_data[fieldLen] = data;
      fieldLen++;
      if (!checksum_en) {
        parity ^= (uint8_t)data;
      }
    }
}

void discover_type(char data) { // assigned at beginning of sentence reading cycle - reads first few chars to get sentence type
  field_data[fieldLen] = data;
  fieldLen++;
  // parity xor, always enabled for beginning stage
  parity ^= (uint8_t)data;
  // this is the first check - make sure it's a proper gps message
  if (fieldLen == 2) {
    if (strcmp("GP", field_data)) {
      type_handler = NULL;
    } /*else {
      Serial.println("Proper gps message detected!");
    }*/
  } // wait for next 3 chars to get the sentence type
  else if (fieldLen == 5) {
    // new string location is located 2 from beginning to read last 3 chars (since 5 - 3 is 2)
    char* typeField = field_data + 2;
    if (strcmp("GGA", typeField) == 0) {
      Serial.println("GGA type sentence detected!");
      type_handler = GGA_handler;
    } else if (strcmp("RMC", typeField) == 0) {
      Serial.println("RMC type sentence detected!");
      // assign handler for RMC string type
    } else { // default case, means cannot recognize and we must disable string handler til next sentence start
      type_handler = NULL;
    }
  }
}

void loop() { // loop that interprets one character received at a time
  while (gps_st.available() > 0) {
    char data = gps_st.read();
    //Serial.print(data);
    // '$' signifies the start of a new sentence
    if (data == '$') {
      // start new sentence reading cycle
      checksum_en = false;
      parity = 0;
      field_no = 1;
      clear_data();
      clear_field();
      type_handler = discover_type;
      return;
    }
    // check if current sentence is already being interpretted as a specific type
    if (type_handler != NULL) {
      type_handler(data);
    }
  }
}

void loop_debug() { // debugging loop, print out whatever is there from gps stream
  while (gps_st.available() > 0 ) {
    Serial.print((char)gps_st.read());
  }
}
