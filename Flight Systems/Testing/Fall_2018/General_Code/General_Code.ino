#include <SD.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define sd_chipSelect 48 // which pin is connected to CS pin of sd module pin
#define sd_fileName "RECOVER.txt"

//#define UNO

/*

   Main code to run on the avionics bay.

*/

// adapt code for Arduino Uno target
#ifdef UNO
 #include <SoftwareSerial.h>
 #define RX 3
 #define TX 5
 SoftwareSerial gps_st(RX, TX);
 SoftwareSerial& Serial3 = gps_st;
#endif

union FloatBytes {
  float f;
  char b[4];
}; // extremely useful for sending messages which are made of chars

char data2commit[100]; // data that we are going to add to sd card
int commitLen; // size of above string - amount currently set to commit

int writeAmount(byte* data, int amount, byte addr) { // write an array of bytes to i2c device
  Wire.beginTransmission(addr);
  for (int i = 0; i < amount; i++) { // send all bytes in the pointer sent
    Wire.write(data[i]);
  }
  return Wire.endTransmission();
}
int readAmount(byte* data, int amount, byte addr) { // read amount of bytes to an array from an i2c device
  Wire.beginTransmission(addr);
  int readAmount = Wire.requestFrom(addr, amount);
  for (int i = 0; i < amount; i++) {
    data[i] = Wire.read();
  }
  Wire.endTransmission();
  return readAmount;
}
int writeToRegister(byte rgstr, byte* data, int size, byte addr) { // i2c write to address register
  Wire.beginTransmission(addr);
  Wire.write(rgstr);
  for (int i = 0; i < size; i++) {
    Wire.write(data[i]);
  }
  return Wire.endTransmission();
}
int readFromRegister(byte rgstr, byte* buffer, int amount, byte addr) { // i2c read from address register
  // first send in register to read from
  Wire.beginTransmission(addr);
  Wire.write(rgstr);
  if (Wire.endTransmission()) {
    return 0;
  }

  int readAmount = Wire.requestFrom(addr, amount);
  // load read bytes into given buffer
  for (int i = 0; i < readAmount; i++) {
    buffer[i] = Wire.read();
  }
  return readAmount;
}
void commitData(char* msg) { // when a function has data to submit (for now it will just be onto an sd card)
  // add data to be committed
  int msgLen = strlen(msg);
  memcpy(data2commit + commitLen, msg, msgLen);
  commitLen += msgLen;
}
void commitData(char* msg, int msgLen) { // when a function has data to submit (for now it will just be onto an sd card)
  // add data to be committed
  memcpy(data2commit + commitLen, msg, msgLen);
  commitLen += msgLen;
}

void finishCommit() { // what we will do with the data - for now its just transferring to sd card
  File commitFile;
  commitFile = SD.open(sd_fileName, FILE_WRITE);
  if (commitFile) {
    for (int i = 0; i < commitLen; i++) {
      Serial.print(data2commit[i]);
      commitFile.print(data2commit[i]);
    }
    commitFile.close();
  }
  // clear the commit buffer
  memset(data2commit, 0, commitLen);
  commitLen = 0;
}



// ms5611 stuff

struct Ms5611_Cmd { // commands to be sent to be pressure sensor which is ms5611
  const byte reset = 0b00011110;
  const byte d1_4096 = 0b01001000; // pressure conversion request - osr is 4096
  const byte d2_4096 = 0b01011000; // temperature conversion request - osr is 4096
  const byte adc_read = 0x00; // run adc and next reading will give the last converted value
  const byte coefficient_req = 0b10100000; // combined with another number to get the right cofficients of conversion - between from c1 to c6
} ms5611_cmd;
// predefined variables to prevent madness from creating and destroying variables every loop
struct Ms5611_Vars {
  int64_t OFF, SENS, P, dT, TEMP;
  char msg_temp[7]; // 7 bytes - initial +  4 bytes float + comma terminator + null terminator
  char msg_pressure[7]; // same size as above for same reasons
} ms5611_vars;

// address of the sensor - depends on the value of the CSB pin
const byte MS5611_CSB_State = 0;
const byte MS5611_addr = 0b1110110 | (1 & ~MS5611_CSB_State);

void resetMS5611() {
  // just send the reset command
  byte data = ms5611_cmd.reset;
  writeAmount(&data, 1, MS5611_addr);
}
// coefficients for pressure and temp conversion
uint16_t ms5611_c[6];
uint16_t ms5611_coef(int num) { // makes the coefficient access much more intuitive as index 0 is actually of coefficient 1
  return ms5611_c[num - 1];
}
void calcTempandPressure(float* temp, float* pressure, uint32_t* d1, uint32_t* d2) { // d1, d2 is raw pressure, temp respectively after combining the data from 3 registers to an uint32
  // all calculations here are from the ms5611 datasheet
  // temperature calculations
  ms5611_vars.dT = *d2 - ((uint32_t)ms5611_coef(5) << 8);

  ms5611_vars.TEMP = 2000 + ((int64_t)ms5611_vars.dT * (int64_t)ms5611_coef(6) >> 23);
  *temp = (float)ms5611_vars.TEMP / 100.0f;

  // pressure calculations
  ms5611_vars.OFF = ((int64_t)ms5611_coef(2) << 16) + ((ms5611_vars.dT * ms5611_coef(4)) >> 7);
  ms5611_vars.SENS = ((int32_t)ms5611_coef(1) << 15) + ((ms5611_vars.dT * ms5611_coef(3)) >> 8);
  ms5611_vars.P = (((int64_t) * d1 * ms5611_vars.SENS >> 21) - ms5611_vars.OFF) >> 15;
  *pressure = (float)ms5611_vars.P / 100.0f;
}
void setupMS5611() {
  Serial.println("I need help");
  resetMS5611();
  _delay_ms(10); // wait a bit after restting
  Serial.println("Houston we've got a problem!");

  // retrieve coefficients from the device's prom
  for (int i = 0; i < 6; i++) {
    byte data[2];
    data[0] = ms5611_cmd.coefficient_req | ((i + 1) << 1); // command to request the (i + 1)th coefficient
    writeAmount(data, 1, MS5611_addr);
    if (readAmount(data, 2, MS5611_addr)) {
      ms5611_c[i] = data[0] << 8 | data[1];
      //Serial.println("Received coefficients!"); // debug
    } else {
      Serial.println("Failure to retrieve coefficient!");
    }
  }

  // setup the message strings' intiator and terminator chars
  ms5611_vars.msg_temp[0] = 'T'; ms5611_vars.msg_temp[5] = ',';
  ms5611_vars.msg_pressure[0] = 'P'; ms5611_vars.msg_pressure[5] = ',';
}
// loop procedure for ms5611
void ms5611_loop() {
  // request for pressure
  writeAmount(&ms5611_cmd.d1_4096, 1, MS5611_addr);
  _delay_ms(10); // conversion from analog takes a while
  writeAmount(&ms5611_cmd.adc_read, 1, MS5611_addr);

  byte raw_data[3];
  uint32_t d1; // raw pressure value after combining registers for pressure data
  if (readAmount(raw_data, 3, MS5611_addr) == 3) { // only proceed if all 3 bytes are gotten
    for (int i = 0; i < 3; i++) { // arrange the bytes received into a 32 bit location, from high to low in order
      d1 <<= 8;
      d1 |= raw_data[i];
    }
  }

  // request for temperature
  writeAmount(&ms5611_cmd.d2_4096, 1, MS5611_addr);
  _delay_ms(10); // conversion from analog takes a while
  writeAmount(&ms5611_cmd.adc_read, 1, MS5611_addr);

  uint32_t d2; // raw temperature value after combining bytes
  if (readAmount(raw_data, 3, MS5611_addr) == 3) {
    for (int i = 0; i < 3; i++) { // arrange the bytes received into a 32 bit location, from high to low in order
      d2 <<= 8;
      d2 |= raw_data[i];
    }
  }
  FloatBytes temp, pressure; // final calculated values

  calcTempandPressure(&temp.f, &pressure.f, &d1, &d2);

  // debug info
  Serial.print("Temperature data: "); Serial.println(temp.f);
  Serial.print("Pressure data: "); Serial.println(pressure.f);

  // update the message template variables with the new data
  for (int i = 0; i < 4; i++) {
    ms5611_vars.msg_temp[i+1] = temp.b[i]; // the data in the string starts one after the first character as the first character tells what type of data the string contains
    ms5611_vars.msg_pressure[i+1] = pressure.b[i];
  }
  //ftoa(temp.f, ms5611_vars.msg_temp, 1);
  dtostrf(temp.f, 1, 3, ms5611_vars.msg_temp);
  //ftoa(pressure.f, ms5611_vars.msg_pressure, 1);
  dtostrf(pressure.f, 1, 3, ms5611_vars.msg_temp);

  commitData(ms5611_vars.msg_temp, 6);
  commitData(ms5611_vars.msg_pressure, 6);
}


// crap for the IMU - MPU-9250
#define imu_int_pin 7 // interrupt pin number for imu data ready
#define sample_rate 4 // sample rate has to be between 4 hz and 1000 hz

void calcAccel(FloatBytes* data, int fsr) { // accepts 3 float values which store raw data
  int convDiv = 1 << 14; // conversion divisor for 2g
  // for each double in FSR from 2, our conversion divisor must get divided by two to accomodate for bigger FSR during conversion
  // we can just divide our given FSR by 2 to find out what to divide our conversion divisor by
  fsr /= 2;
  convDiv /= fsr;

  for (int i = 0; i < 3; i++) {
    data[i].f /= float(convDiv);
  }
}
void calcGyro(FloatBytes* data, int fsr) {
  // uses a conversion factor specified in data sheet for an fsr of 250 LSB/(degree per second)
  for (int i = 0; i < 3; i++) {
    data[i].f *= (float(fsr) / 250.0f) / 131.0f; // the conversion works by getting the value you need to divide by for 250DPS fsr (131 according to datasheet) and multiplying by how many times bigger the actual fsr is
  }
  // we will probably need to test for the 250 dps fsr as its not perfect
}
// sensity values for conversion from raw measurement data to values in micro Tesla
struct Mag_Sens {
  static byte x;
  static byte y;
  static byte z;
} mag_sens;
static byte Mag_Sens::x = -1;
static byte Mag_Sens::y = -1;
static byte Mag_Sens::z = -1;

void calcMag(FloatBytes* data, int output_mode) { // output mode is either 14 or 16 bit - will be used in conversion
  // use equation given in register map to calculate the magnetism
  // it is assumed the given array has measurement values in order: x y z each measurement being a float
  // we need to ensure we're accomodating properly for the bit range - the bit difference will be used as an exponent of 2 which we will divide our number by to accomodate for the extra bits
  float convDiv = 1 << (output_mode - 14);
  data[0].f *= (1.0f + (float(mag_sens.x) - 128.0f) * 0.5f / 128.0f) / convDiv;
  data[1].f *= (1.0f + (float(mag_sens.y) - 128.0f) * 0.5f / 128.0f) / convDiv;
  data[2].f *= (1.0f + (float(mag_sens.z) - 128.0f) * 0.5f / 128.0f) / convDiv;
}
struct IMU_Reg { // stores register values to for the mpu 9250
  static const byte pwr_mgmt_1 = 0x6B;
  static const byte pwr_mgmt_2 = 0x6C;
  static const byte config = 0x1A;
  static const byte smplrt_div = 0x19;
  static const byte accel_config = 0x1C;
  static const byte accel_config_2 = 0x1D;
  static const byte gyro_config = 0x1B;
  static const byte int_bypass_cfg = 0x37;
  static const byte int_enable = 0x38;
  static const byte i2c_mst_ctrl = 0x24;
  static const byte user_ctrl = 0x6A;

  // registers for slave 0 handler
  static const byte slv0_addr = 0x25; // aka I2C_SLV0_ADDR - used for read/write mode and address set
  static const byte slv0_reg = 0x26; // aka I2C_SLV0_REG - address to start data transfer from in the slave device
  static const byte slv0_ctrl = 0x27; // aka I2C_SLV0_CTRL - does a bunch of crap im too lazy to write it all
  static const byte slv0_do = 0x63; // aka I2C_SLV0_DO - data to be sent to slave device

  static const byte ext_sens_data_00 = 0x49; // starting address for i2c slave readings - up to 23 bytes

  static const byte int_status = 0x3A;

  static const byte accel_x_H = 0x3B;
  /*static const byte accel_x_L = 0x3C;
    static const byte accel_y_H = 0x3D;
    static const byte accel_y_L = 0x3E;
    static const byte accel_z_H = 0x3F;
    static const byte accel_z_L = 0x40;*/

  static const byte gyro_x_H = 0x43;
  /*static const byte gyro_x_L = 0x44;
    static const byte gyro_y_H = 0x45;
    static const byte gyro_y_L = 0x46;
    static const byte gyro_z_H = 0x47;
    static const byte gyro_z_L = 0x48;*/
} IMU_reg;
struct IMU_bits { // bit values for the imu
  // pwr mgmt 1 bits
  static const byte H_RESET = 0x80;
  static const byte AUTO_CLCK_SEL = 1;
  static const byte BIT_SLEEP = 0x40;

  // int status bits
  static const byte RAW_DATA_RDY_INT = 0x01;

  // int enable bits
  static const byte RAW_DATA_EN = 1; // interrupt when raw data is available

  // int and bypass configuration bits
  static const byte ACTL = 1 << 7; // active low set
  static const byte OPEN_POP = 1 << 6;  // open drain or push or pull configuration
  static const byte LATCH_INT_EN = 1 << 5; // whether to hold interrupt until int_status is read or if to do a 50 us interrupt
  static const byte INT_STATUS_CLEAR = 1 <<  4;
  static const byte BYPASS_EN = 1 << 1; // bypass mpu for magnetometer

  // i2c master control bits
  static const byte WAIT_FOR_ES = 1 << 6; // if set, interrupt only triggers when external sensor data is loaded
  static const byte MULT_MST_EN = 1 << 7; // allow multiple masters to act on the line

  // slave handler control bits below (same bits for all slaves)

  // slv0_addr bits
  static const byte SLV_READ = 1 << 7; // read mode for slave
  // for write mode you set that bit to 0

  // slv0_ctrl bits
  static const byte SLV_EN = 1 << 7; // enable slave handler
  static const byte SLV_BYTE_SWP = 1 << 6; // swap groups of bytes after reading them in the external sensor data registers (byte group definition are based on whether SLV_GRP bit is set) (groups are 2 bytes in size)
  static const byte SLV_REG_DIS = 1 << 5; // don't send register address before reading or writing data
  static const byte SLV_GRP = 1 << 4; // 0 means 0 and 1 define first group, 1 means 1 and 2 define a group

  // user ctrl bits
  static const byte I2C_MST_EN = 1 << 5; // enable master mode for devices connected to imu's own i2c bus (in particular the magnetometer - AK8963)
} IMU_bits;
struct Mag_Reg { // register addresses for the magnetometer - AK8963
  static const byte cntl1 = 0x0A; // control 1 register
  static const byte cntl2 = 0x0B; // control 2 register

  // following registers give data in each direction
  // each direction measurement is 2 bytes long and is deposited in memory starting at the low byte
  static const byte x_L = 0x03;
  /*static const byte x_H = 0x04;
    static const byte y_L = 0x05;
    static const byte y_H = 0x06;
    static const byte z_L = 0x07;
    static const byte z_H = 0x08;*/

  // status 2 register - read this at the end of measurement to have this register refresh
  static const byte status_2 = 0x09;

  // sensitivity values for each dimension
  static const byte sens_x = 0x10;
  static const byte sens_y = 0x11;
  static const byte sens_z = 0x12;
} mag_reg;
struct Mag_Bits { // bit values for magnetometer
  // CNTL2 bits
  static const byte SOFT_RES = 1 << 0; // soft reset

  // CNTL1 bits
  static const byte OUTPUT_16 = 1 << 4; // if set, 16 bit output; not set means 14 bit
  static const byte SINGLE_MEASUREMENT = 1 << 0; // single measurement mode - only read once before no more updates to measurement registers
  static const byte CONT_MEASUREMENT_1 = 1 << 1; // 8Hz measurement update
  static const byte CONT_MEASUREMENT_2 = 0b0110; // 100Hz measurement update

} mag_bits;
// vars to prevent the madness of construction and destruction of variables
struct IMU_Vars {
  char msg_accel[17]; // initial char + 3 accel comps * 4 bytes + 2 spacers + comma terminator + null terminator
  char msg_gyro[17]; // same size for same reasons as above
  char msg_mag[17]; // same size for same reasons as above
} IMU_vars;

const byte IMU_addr = 0b1101000; // imu address
const byte MAG_addr = 0x0C; // magnetometer address - if i2c bypass on imu is on

void resetIMU() { // send reset command to device
  // this will be done by setting the H_RESET bit in power management register 1
  byte data = IMU_bits.H_RESET;
  if ( writeToRegister(IMU_reg.pwr_mgmt_1, &data, 1, IMU_addr) ) {
    Serial.println("Failure to send reset command!");
  }
}
// lpf_vals and set_lpf are all for setting low pass filter to change data sampling rate
enum lpf_vals {
  FILTER_188 = 1,
  FILTER_98,
  FILTER_42,
  FILTER_20,
  FILTER_10,
  FILTER_5
};
void set_lpf(int fsr) { // this changes the dlpf_cfg portion of the config register
  byte data;
  if (fsr >= 188) {
    data = FILTER_188;
  } else if (fsr >= 98) {
    data = FILTER_98;
  } else if (fsr >= 42) {
    data = FILTER_42;
  } else if (fsr >= 20) {
    data = FILTER_20;
  } else if (fsr >= 10) {
    data = FILTER_10;
  } else {
    data = FILTER_5;
  }
  if (writeToRegister(IMU_reg.config, &data, 1, IMU_addr)) {
    // how will we respnod to this FAILURE!?
    Serial.println("Failure to set lpf!");
  }
}
void setupIMUInterrupt() {
  byte data = 0;
  data |= IMU_bits.OPEN_POP; // open drain configuration as the arduino has it's own pull up resistor
  data |= IMU_bits.LATCH_INT_EN; // interrupt status remains until int_status is cleared
  //data |= bits.INT_STATUS_CLEAR; // will make int status get cleared with any read instead of clearing when int_status register is read
  data |= IMU_bits.ACTL; // active low interrupts

  if (writeToRegister(IMU_reg.int_bypass_cfg, &data, 1, IMU_addr)) {
    Serial.println("Failure to write to int_bypass_cfg register!");
  } /* */

  data = IMU_bits.RAW_DATA_EN;
  if (writeToRegister(IMU_reg.int_enable, &data, 1, IMU_addr)) {
    Serial.println("Failure to send int enable bits!");
  }
}
int set_bypass(int state) { // bypass the mpu to be able to access the magnetometer directly
  byte data;
  if (state) {
    // only change the bypass value of the int/bypass register
    // so we must know the original state of the register
    if (readFromRegister(IMU_reg.int_bypass_cfg, &data, 1, IMU_addr)) { // readFromRegister is non-zero if success as it returns bytes read
      data |= IMU_bits.BYPASS_EN;
      if (writeToRegister(IMU_reg.int_bypass_cfg, &data, 1, IMU_addr)) {
        return 0;
      }
    }
  } else {
    if (readFromRegister(IMU_reg.int_bypass_cfg, &data, 1, IMU_addr)) { // readFromRegister is non-zero if success as it returns bytes read
      data &= ~IMU_bits.BYPASS_EN;
      if (writeToRegister(IMU_reg.int_bypass_cfg, &data, 1, IMU_addr)) {
        return 0;
      }
    }
  }
  return -1; // if function reached this point its a failure
}
void setupMagnetometer() {
  byte data;
  // first we setup magnetometer by communicating with it directly
  set_bypass(1);
  // first do a soft reset of the magnetometer
  //data = mag_bits.SOFT_RES;
  if (writeToRegister(mag_reg.cntl2, &data, 1, MAG_addr)) {
    Serial.println("Failure to send reset bits!");
  }
  // wait a while for soft reset to occur
  _delay_ms(50);
  // set 100Hz output mode with 16 bits
  data = mag_bits.CONT_MEASUREMENT_2 | mag_bits.OUTPUT_16;
  writeToRegister(mag_reg.cntl1, &data, 1, MAG_addr);

  // get sensitivity values for conversion from raw measurement to micro Tesla
  // sensitivity values are stored in order x y z
  byte sens[3];
  readFromRegister(mag_reg.sens_x, sens, 3, MAG_addr);
  mag_sens.x = sens[0]; mag_sens.y = sens[1]; mag_sens.z = sens[2];

  Serial.print("Sensitivity Values Magnetometer: "); Serial.print(mag_sens.x); Serial.print(" "); Serial.print(mag_sens.y); Serial.print(" "); Serial.println(mag_sens.z);

  // we are done interacting directly with magnetometer at this point
  set_bypass(0);

  // interrupts only trigger when external sensor data is loaded. also multiple masters can operate on lines
  data = IMU_bits.WAIT_FOR_ES | IMU_bits.MULT_MST_EN;
  writeToRegister(IMU_reg.i2c_mst_ctrl, &data, 1, IMU_addr);

  // setup the slave 0 handler to read from magnetometer
  data = IMU_bits.SLV_READ;
  data |= MAG_addr; // in the same register, we put the address
  writeToRegister(IMU_reg.slv0_addr, &data, 1, IMU_addr);
  // register to start reading from - we'll do x measurement low register since that is first in full measurement sequence which is 6 bytes long - two bytes for each dimension
  data = mag_reg.x_L;
  writeToRegister(IMU_reg.slv0_reg, &data, 1, IMU_addr);

  // enable slave 0 handler
  data = IMU_bits.SLV_EN;
  data |= 7; // finally, tell that 7 bytes total in measurement data (two bytes for each dimension) must be read from the starting register address; extra byte is to read status_2 register which will make the sensor take another measurement
  writeToRegister(IMU_reg.slv0_ctrl, &data, 1, IMU_addr);

  // enable i2c master
  data = IMU_bits.I2C_MST_EN;
  writeToRegister(IMU_reg.user_ctrl, &data, 1, IMU_addr);
}
void setupIMU() {
  pinMode(imu_int_pin, INPUT_PULLUP); // interrupt pin will be active low

  resetIMU();
  _delay_ms(200);
  byte data = 0x00;
  writeToRegister(IMU_reg.pwr_mgmt_1, &data, 1, IMU_addr);/* */

  // put the damn thing to sleep for startup
  data = IMU_bits.BIT_SLEEP;
  writeToRegister(IMU_reg.pwr_mgmt_1, &data, 1, IMU_addr);
  _delay_ms(100);
  data = 0x00;
  writeToRegister(IMU_reg.pwr_mgmt_1, &data, 1, IMU_addr);

  // choose best clock to use timing
  data = IMU_bits.AUTO_CLCK_SEL;
  writeToRegister(IMU_reg.pwr_mgmt_1, &data, 1, IMU_addr);

  // suggested code for setting sample rate:
  data = 1000 / sample_rate - 1;
  writeToRegister(IMU_reg.smplrt_div, &data, 1, IMU_addr);
  set_lpf(sample_rate >> 1);
  // we have to set the low pass filter as well

  // set up our own full scale range
  // b10 means 8 gs in the correct register
  data = 0b10;
  data <<= 3; // put in correct place to set the accel fsr
  writeToRegister(IMU_reg.accel_config, &data, 1, IMU_addr);
  // gyro fsr
  // b00 means 250 fsr
  // b11 means 2000 fsr
  data = 0b00;
  data <<= 3; // put in correct place for gyro fsr
  writeToRegister(IMU_reg.gyro_config, &data, 1, IMU_addr);

  setupIMUInterrupt();
  setupMagnetometer();

  // setup the message strings - put in the characters that will never change: initial character, spacers, comma terminator
  IMU_vars.msg_accel[0] = 'A'; IMU_vars.msg_accel[15] = ','; IMU_vars.msg_accel[5] = ' '; IMU_vars.msg_accel[10] = ' ';
  IMU_vars.msg_gyro[0] = 'G'; IMU_vars.msg_gyro[15] = ','; IMU_vars.msg_gyro[5] = ' '; IMU_vars.msg_gyro[10] = ' ';
  IMU_vars.msg_mag[0] = 'M'; IMU_vars.msg_mag[15] = ','; IMU_vars.msg_mag[5] = ' '; IMU_vars.msg_mag[10] = ' ';
}
void imu_loop() {
  // accel, gyro readings are all 16 bit values stored in two registers that are each one byte long
  // and each MSB byte register has it's corresponding LSB register after
  // magnetometer readings are also 16 bit values but LSB comes before the MSB
  // we'll store these in float values which are 32 bits long (enough to store the value)

  if (true) { // only try reading if the interrupt on the device is triggered
    byte raw_data[6]; // raw data for each dimension for accel, gyro, mag each being 2 bytes long so in total 6 bytes
    if (readFromRegister(IMU_reg.int_status, raw_data, 1, IMU_addr) == 0) { // read int status register
      Serial.println("Failure to read from int status register!");
    }
    if ((IMU_bits.RAW_DATA_RDY_INT & raw_data[0]) == 0) { // check if interrupt occured because raw data was ready
      return;
    }

    // Accel Reading Section
    FloatBytes accel[3]; // axies values will be stored in order x,y,z
    readFromRegister(IMU_reg.accel_x_H, raw_data, 6, IMU_addr);
    for (int i = 0; i < 3; i++) {
      accel[i].f = raw_data[2 * i] << 8 | raw_data[2 * i + 1]; // convert the 2 byte values for each dimension into a single float value for conversion (while swapping low and high bytes)
    }
    calcAccel(accel, 8); // 8g full scale range

    // Gyro calculation section (this method for getting the values is better)
    // fsr for gyro currently is 250 DPS
    FloatBytes gyro[3]; // gyro values stored in order x y z
    readFromRegister(IMU_reg.gyro_x_H, raw_data, 6, IMU_addr);
    for (int i = 0; i < 3; i++) {
      gyro[i].f = raw_data[2 * i] << 8 | raw_data[2 * i + 1]; // convert the 2 byte values for each dimension into a single float value for conversion (while swapping low and high bytes)
    }
    calcGyro(gyro, 250); // 250 dps full scale range

    // magnetometer readings
    FloatBytes mag[3];
    readFromRegister(IMU_reg.ext_sens_data_00, raw_data, 6, IMU_addr); // magnetometer is a sensor being read from by the IMU in master mode so we're reading from a register that got its data from the magnetometer
    for (int i = 0; i < 3; i ++) { // convert the 2 byte values for each dimension into a single float value for conversion
      mag[i].f = raw_data[2 * i + 1] << 8 | raw_data[2 * i];
    }
    calcMag(mag, 16);

    Serial.print("Accel X Data!:"); Serial.println(accel[0].f);
    Serial.print("Accel Y Data!:"); Serial.println(accel[1].f);
    Serial.print("Accel Z Data!:"); Serial.println(accel[2].f);
    Serial.print("Gyro X Data!:"); Serial.println(gyro[0].f);
    Serial.print("Gyro Y Data!:"); Serial.println(gyro[1].f);
    Serial.print("Gyro Z Data!:"); Serial.println(gyro[2].f);
    Serial.print("Magnetometer X Data!:"); Serial.println(mag[0].f);
    Serial.print("Magnetometer Y Data!:"); Serial.println(mag[1].f);
    Serial.print("Magnetometer Z Data!:"); Serial.println(mag[2].f);

    // create messages for accel, gyro, mag for commit
    for (int comp = 0; comp < 3; comp++) {
      // starting position of each component in the message
      int pos = 1 + (comp * 5);
      for (int i = 0; i < 4; i++) { // loop through 4 bytes of each component
        IMU_vars.msg_accel[pos + i] = accel[comp].b[i];
        IMU_vars.msg_gyro[pos + i] = gyro[comp].b[i];
        IMU_vars.msg_mag[pos + i] = mag[comp].b[i];
      }
    }
  }
  //_delay_ms(100);
  commitData(IMU_vars.msg_accel, 16);
  commitData(IMU_vars.msg_gyro, 16);
  commitData(IMU_vars.msg_mag, 16);
}

// GPS stuff
#define GPS_RX_pin 3
#define GPS_TX_pin 5
//#define GPS_baud_rate 38400
#define GPS_baud_rate 9600

// these bytes being sent can control speed of gps read rate
const unsigned char UBLOX_INIT[] PROGMEM = {
  // Rate (pick one)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)

  // Disable NMEA
  //  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  //  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off
};


void setupGPS() {
  Serial3.begin(9600); // initial baud rate
  //_delay_ms(100);
  //Serial.flush(); Serial.begin(19200);
  _delay_ms(100);
  // setup the gps module according to the data stored in UBLOX_INIT
  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    Serial3.write( pgm_read_byte(UBLOX_INIT + i) );
  }
  _delay_ms(100);
  // reset serial for the new baud rate
  //Serial3.flush();
  //Serial3.begin(GPS_baud_rate);
}

// current task function pointer - assigned to a specific function to handle each type
struct GPS_Data {
  char time[20];
  bool gps_str_read = false;
  bool gga_str_read = false;
  char long_coords[20];
  char lat_coords[20];
  char sealvl_alt[20];
  char track_made_goodN[20]; // direction of speed relative to true north - the magnetic north track made good will be ignored
  char grnd_speed[20];
} gps_data;
void clear_data() {
  memset(&gps_data, 0, sizeof(gps_data));
  gps_data.time[0] = (char)(-1);
  gps_data.long_coords[0] = (char)(-1);
  gps_data.lat_coords[0] = (char)(-1);
  gps_data.sealvl_alt[0] = (char)(-1);
  gps_data.track_made_goodN[0] = (char)(-1);
  gps_data.grnd_speed[0] == (char)(-1);
}

void (*type_handler)(char data) = NULL; // function addressed here is a sentence specific handler to get the data we need from the string and commit appropriately

char field_data[20]; // current field data
int fieldLen = 0; // number of chars read from current field
int field_no = 0; // current field starting from 1 in the current sentence
void clear_field() { // reset data stored from current field (make it empty)
  memset(field_data, 0, fieldLen);
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
      //Serial.println("Successful checksum!");
      // check if a value for the following data fields has been recorded
      char msg_location[12]; msg_location[0] = 'L'; msg_location[5] = ' '; msg_location[10] = ',';
      if (gps_data.time[0] != -1) {
        Serial.print("Time: "); Serial.println(gps_data.time);
        FloatBytes time; time.f = atof(gps_data.time);
        char msg_time[7]; msg_time[0] = 'U'; msg_time[5] = ','; // message designator for time
        for (int i = 0; i < 4; i++) {
          msg_time[i + 1] = time.b[i];
        }
        commitData(msg_time, 6);
      } else {
        Serial.println("Invalid time data!");
      }
      if (gps_data.long_coords[0] != -1) {
        Serial.print("Longitude: "); Serial.println(gps_data.long_coords);
        FloatBytes longitude; longitude.f = 1;
        if (gps_data.long_coords[strlen(gps_data.long_coords)] == 'E') { // data will be negative if its east
          longitude.f = -1;
        }
        gps_data.long_coords[strlen(gps_data.long_coords)] = '0';
        longitude.f *= atof(gps_data.long_coords);
        for (int i = 0; i < 4; i++) {
          msg_location[6 + i] = longitude.b[i];
        }
      }
      if (gps_data.lat_coords[0] != -1) {
        Serial.print("Latitude: "); Serial.println(gps_data.lat_coords);
        FloatBytes latitude; latitude.f = 1;
        if (gps_data.lat_coords[strlen(gps_data.lat_coords)] == 'E') { // data will be negative if its east
          latitude.f = -1;
        }
        gps_data.lat_coords[strlen(gps_data.lat_coords)] = '0';
        latitude.f *= atof(gps_data.long_coords);
        for (int i = 0; i < 4; i++) {
          msg_location[1 + i] = latitude.b[i];
        }
      }
      if (gps_data.sealvl_alt[0] != -1) {
        Serial.print("Altitude above sea level: "); Serial.println(gps_data.sealvl_alt);
      }
      commitData(msg_location, 11);

      // tell if no location data at all is being recorded
      if (gps_data.long_coords[0] == -1 && gps_data.lat_coords[0] == -1) {
        Serial.println("Invalid longitude and latitude data!");
      }
    } else {
      Serial.println("Houston, we've got a problem!");
      // clear all the fields
      clear_data();
    }
    // indicate gga been read
    gps_data.gga_str_read = true;
    // indicate that a gps string has been finished reading
    gps_data.gps_str_read = true;
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
void VTG_handler(char data) {
  if (data == '\r' || data == '\n') { // signifies end of sentence - time to run checksum
    // however, if final checksum process was not started, this sentence must be garbage
    type_handler = NULL; // end of sentence in any checksum scenario
    if (!checksum_en) {
      // don't even bother with any checksums
      return;
    }
    if (run_checksum()) { // if this part is ever entered it means the checksum was successful and we're committing data
      if (gps_data.track_made_goodN[0] != -1) {
        Serial.print("Track made good: "); Serial.println(gps_data.track_made_goodN);
      }
    } else {
      Serial.println("Houston, we've got a problem!");
      // clear all the fields
      clear_data();
    }
    // indicate a gps string has been finished reading
    gps_data.gps_str_read = true;
    return;
  } else if (data == '*') { // start of checksum, end of current field
    field_no++;
    checksum_en = true;
    return; // no point in running rest of function
  } else if (data == ',') { // indicates end of field
    if (field_no == 2) { // track made good for true north
      memcpy(gps_data.track_made_goodN, field_data, fieldLen); // temporarily copy track made good and we will check after if this is the track made good relative to true north
    } else if (field_no == 3) { // indicates if track made good is relative to magnetic or true north
      if (strcmp('T', field_data)) { // if they are different, the track made good is wrong we must reset the track made good field
        memset(gps_data.track_made_goodN, 0, strlen(gps_data.track_made_goodN));
        gps_data.track_made_goodN[0] = (char)(-1);
      }
    } else if (field_no == 8) { // ground speed measurement, following field tells units
      memcpy(gps_data.grnd_speed, field_data, fieldLen); // copy temporarily, check later
    } else if (field_no == 9) {
      if (strcmp('K', field_data)) {
        memset(gps_data.grnd_speed, 0, strlen(gps_data.grnd_speed));
        gps_data.grnd_speed[0] = (char)(-1);
      }
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
    }
  } // wait for next 3 chars to get the sentence type
  else if (fieldLen == 5) {
    // new string location is located 2 from beginning to read last 3 chars (since 5 - 3 is 2)
    char* typeField = field_data + 2;
    if (strcmp("GGA", typeField) == 0) {
      //Serial.println("GGA type sentence detected!");
      type_handler = GGA_handler;
    } else if (strcmp("VTG", typeField) == 0) {
      //Serial.println("VTG type sentence detected!");
      type_handler = VTG_handler;
    } else { // default case, means cannot recognize and we must disable string handler til next sentence start
      type_handler = NULL;
    }
  }
}
void gps_loop() {
  //Serial.println("GPS LOOPPP");
  while (Serial3.available() == 0); // wait until the gps has something to tell us
  do { // first check if a string is available to be read
    while (!gps_data.gga_str_read) {
      while (Serial3.available() > 0 && !gps_data.gga_str_read) {
      char data = Serial3.read();
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
        //return;
        break;
      }
      // check if current sentence is already being interpretted as a specific type
      if (type_handler != NULL) {
        type_handler(data);
      }
    }
    }
    gps_data.gga_str_read = false;
  } while (!gps_data.gps_str_read); // keep looking for data until something useful to us is printed by the gps
}


void setup() {
  Serial.begin(GPS_baud_rate);
  Wire.begin();
  if (!SD.begin(sd_chipSelect)) {
    Serial.println("Failure to initialize SD card!");
  }
  Serial.println("AHHH");

  setupMS5611();
  setupIMU();
  setupGPS();
  //_delay_ms(100);
}

long prev;
void loop() {
  gps_loop();
  if (gps_data.gps_str_read) { // only read ms5611 if we gained data useful to us from the gps
    ms5611_loop();
    gps_data.gps_str_read = false; // only read this once per each useful read from gps!
  }
  //gps_data.gps_str_read = false;
  //_delay_ms(100);
  imu_loop();
  //_delay_ms(500);
  //_delay_ms(500);

  finishCommit();
  long cur = millis();
  Serial.println(cur - prev);
  prev = cur;
  //_delay_ms(100);
}
