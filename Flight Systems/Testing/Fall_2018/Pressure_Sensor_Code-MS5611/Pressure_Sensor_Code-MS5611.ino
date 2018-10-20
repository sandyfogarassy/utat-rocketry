#include <Wire.h>
/*
 * MS5611 pressure sensor code
 * Uses I2C for data output
 * Used as altimeter
 */

// address of the sensor - depends on the value of the CSB pin
const byte CSB_State = 0;
const byte ADDR = B1110110;

int readAmount(byte* buffer, int amount) {
  int receivedAmount = Wire.requestFrom(ADDR, amount); // read amount of bytes specified from sensor

  // load the recorded bytes into the array specified
  int i = 0;
  while (Wire.available()) {
    buffer[i] = Wire.read();
    i++;
  }
  return receivedAmount;
}
int reset() {
  Wire.beginTransmission(ADDR);
    Wire.write(0b00011110); // this is the reset command
  byte result = Wire.endTransmission();
  return result;
}
uint32_t readRawTemp() {
  Wire.beginTransmission(ADDR);
    Wire.write(0b01011000); // command for d2 conversion when adc is called
  Wire.endTransmission();

  // call adc
  Wire.beginTransmission(ADDR);
    Wire.write(0b00000000);
  Wire.endTransmission();

  uint32_t data;
  readAmount((byte*) &data, 3);
  return data;
}
uint32_t readRawPressure() {
  Wire.beginTransmission(ADDR);
    Wire.write(0b01001000); // command for d1 conversion with OSR at 4096 when adc is called
  if(Wire.endTransmission()) {
    Serial.println("Failure to transmit pressure conversion bytes!");
  }

  // call adc
  Wire.beginTransmission(ADDR);
    Wire.write(0b00000000);
  if(Wire.endTransmission()) {
    Serial.println("Failure to transmit bytes for pressure read!");
  }

  uint32_t data;
  readAmount((byte*) &data, 3);
  return data;
}
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);

  _delay_ms(50);
  // send reset command to load prom or something i forgot
  if(int i = reset()) {
    Serial.println("Failure to reset!");
    Serial.print("Error number: "); Serial.println(i);
  }
  _delay_ms(100);
/*
  // send a bunch of useless sckls and then reset
  Wire.beginTransmission(ADDR);
  Wire.write(0);Wire.write(0);
  Wire.endTransmission();
  _delay_ms(50);
  reset();`
  _delay_ms(50);*/
}

void loop() {
  // first print raw temp then raw pressure data
  //Serial.print("Raw Temperature Data: "); Serial.print(readRawTemp()); Serial.print("\n");
  
  Serial.print("Raw Pressure Data: "); Serial.print(readRawPressure()); Serial.print("\n");
  _delay_ms(800); /* */
}
