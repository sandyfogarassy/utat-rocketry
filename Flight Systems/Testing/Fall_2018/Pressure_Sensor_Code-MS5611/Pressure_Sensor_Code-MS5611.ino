#include <Wire.h>
/*
 * MS5611 pressure sensor code
 * Uses I2C for data output
 * Used as altimeter and temperature sensor
 */

struct Ms5611_Cmd {
  static const byte reset = 0b00011110;
  static const byte d1_4096 = 0b01001000; // pressure conversion request - osr is 4096
  static const byte d2_4096 = 0b01011000; // temperature conversion request - osr is 4096
  static const byte adc_read = 0x00; // run adc and next reading will give the last converted value
  static const byte coefficient_req = 0b10100000; // combined with another number to get the right cofficients of conversion - between from c1 to c6
} ms5611_cmd;

// address of the sensor - depends on the value of the CSB pin
const byte CSB_State = 0;
const byte ADDR = 0b1110110 | (1 & ~CSB_State);

int write(byte data, byte addr) {
  Wire.beginTransmission(addr);
  Wire.write(data);
  return Wire.endTransmission();
}
int readAmount(byte* data, int amount, byte addr) { // data is the array to store the data
  int readAmount = Wire.requestFrom(addr, amount);
  for (int i = 0; i < amount; i++) {
    data[i] = Wire.read();
  }
  return readAmount;
}
void reset() {
  // just sends the reset command
  if (int i = write(ms5611_cmd.reset, ADDR)) {
    Serial.println("Failure to send reset!");
    Serial.println(i);
  } else {
    Serial.println("Successful reset!");
  }
}

// coefficients for conversion to real values
uint16_t c[6];

void setup() {
  Serial.begin(9600);
  Wire.begin();
  reset();

  _delay_ms(10); // wait a bit after resetting

  // retrieve prom coefficients for conversion
  for (int i = 0; i < 6; i++) {
    byte data[2];
    write(ms5611_cmd.coefficient_req | ((i + 1) << 1), ADDR);
    if (readAmount(data, 2, ADDR)) {
      Serial.println("Coefficient successfully read!");
      c[i] = data[0] << 8 | data[1];
      Serial.println(c[i]);
    } else {
      Serial.println("Failure to read coefficients!");
    }
  }
}
// makes coefficient access easier to understand
// ex.: p_coef(1) will be ptr to coefficient 1
uint16_t coef(int num) {
  return c[num - 1];
}
// these variables are used in function below
// they are declared here to stop the rampant variable destruction and creation
int32_t dT, TEMP;
int64_t OFF, SENS, P;
void calcTempandPressure(float* temp, float* pressure, uint32_t* d1, uint32_t* d2) {
  // temperature calculations - will be used to compensate for sensor behavior changes 
  // at different temperatures
  dT = *d2 - ((uint32_t)coef(5) << 8);
  
  TEMP = 2000 + ((int64_t)dT * (int64_t)coef(6) >> 23);
  *temp = (float)TEMP / 100.0f; 

  // pressure calculations
  OFF  = ((int64_t)coef(2) << 16) + ((dT * coef(4)) >> 7);
  SENS = ((int32_t)coef(1) << 15) + ((dT * coef(3)) >> 8);
  P = (((int64_t)*d1 * SENS >> 21) - OFF) >> 15;
  *pressure = (float)P / 100.0f;
}
void loop() {
  // send request for pressure and do adc
  write(ms5611_cmd.d1_4096, ADDR);
  _delay_ms(10); // conversion takes a while
  write(ms5611_cmd.adc_read, ADDR);
  
  byte raw_data[3]; // 24 bit output!
  
 uint32_t d1; // raw pressure value
  if (readAmount(raw_data, 3, ADDR)) {
    for (int i = 0; i < 3; i++) { // arrange the bytes received into a 32 bit location, from high to low in order
      d1 <<= 8;
      d1 |= raw_data[i];
    }
  }

  write(ms5611_cmd.d2_4096, ADDR); // temperature request
  _delay_ms(10); // conversion takes a while
  write(ms5611_cmd.adc_read, ADDR);
  
 uint32_t d2; // raw temp value
  if (readAmount(raw_data, 3, ADDR)) {
    for (int i = 0; i < 3; i++) { // arrange the bytes received into a 32 bit location, from high to low in order
      d2 <<= 8;
      d2 |= raw_data[i];
    }
  }
  float temp, pressure; // store final calculated values
  
  calcTempandPressure(&temp, &pressure, &d1, &d2);
  Serial.print("Temp: "); Serial.println(temp);  
  Serial.print("Pressure: "); Serial.println(pressure);
}
