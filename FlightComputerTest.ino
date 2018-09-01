/*
  Multiple Serial test

  Receives from the main serial port, sends to the others.
  Receives from serial port 1, sends to the main serial (Serial 0).

  This example works only with boards with more than one serial like Arduino Mega, Due, Zero etc.

  The circuit:
  - any serial device attached to Serial port 1
  - Serial Monitor open on Serial port 0

  created 30 Dec 2008
  modified 20 May 2012
  by Tom Igoe & Jed Roach
  modified 27 Nov 2015
  by Arturo Guadalupi

  This example code is in the public domain.
*/

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  
}
 
void loop() {
  int i=0;
  int sum=
  while(i<10){
    if (Serial2.available()) {
       rawAltitude1=Serial2.parseint();
    }

    if (Serial3.available()) {
        rawAltitude2=Serial3.parseint();
    }

    if(Serial3.available()) {
       Serial.println(Serial3.parseInt());
    }
  
    i++;
    delay(250);
  }
}


