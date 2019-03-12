/*
 * Code aiming to move away from libraries for use with MPU 9250
 * Note: MPU 9250 contains a seperate magnetometer (AK8963) that can either be accessed by bypassing mpu 9250 or
 * using the mpu 9250 in master mode for the magnetometer
 * 
 * The IMU gives accelerometer, gyro, and magnetometer readings.
 * There is also some other crap that you can do with the IMU, but that stuff is pretty advanced and looking at previous avionics code, we won't be needing it right now.
 */
#include <Wire.h>

#define int_pin 7

void calcAccel(float* data, int fsr) { // accepts 3 float values which store raw data 
  int convDiv = 1 << 14; // conversion divisor for 2g
  // for each double in FSR from 2, our conversion divisor must get divided by two to accomodate for bigger FSR during conversion
  // we can just divide our given FSR by 2 to find out what to divide our conversion divisor by
  fsr /= 2;
  convDiv /= fsr;
  
  for (int i = 0; i < 3; i++) {
    data[i] /= float(convDiv);
  }
  //return float(raw_data) / float(convDiv);
}
void calcGyro(float* data, int fsr) {
  // uses a conversion factor specified in data sheet for an fsr of 250 LSB/(degree per second)
  for (int i = 0; i < 3; i++) {
    data[i] *= (float(fsr) / 250.0f) / 131.0f; // the conversion works by getting the value you need to divide by for 250DPS fsr (131 according to datasheet) and multiplying by how many times bigger the actual fsr is
  }
  //return float(raw_data) * (float(fsr) / 250.0f) / 131.0f; 
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

void calcMag(float* data, int output_mode) { // output mode is either 14 or 16 bit - will be used in conversion
  // use equation given in register map to calculate the magnetism
  // it is assumed the given array has measurement values in order: x y z each measurement being a float
  // we need to ensure we're accomodating properly for the bit range - the bit difference will be used as an exponent of 2 which we will divide our number by to accomodate for the extra bits
  float convDiv = 1 << (output_mode - 14);
  data[0] *= (1.0f + (float(mag_sens.x) - 128.0f) * 0.5f / 128.0f) / convDiv;
  data[1] *= (1.0f + (float(mag_sens.y) - 128.0f) * 0.5f / 128.0f) / convDiv;
  data[2] *= (1.0f + (float(mag_sens.z) - 128.0f) * 0.5f / 128.0f) / convDiv;
  /*for (int i = 0; i < 3; i++) {
    data[i] *= 0.15f;
  }*/
}
struct Reg { // stores register values to for the mpu 9250
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
  static const byte accel_x_L = 0x3C;
  static const byte accel_y_H = 0x3D;
  static const byte accel_y_L = 0x3E;
  static const byte accel_z_H = 0x3F;
  static const byte accel_z_L = 0x40;
  
  static const byte gyro_x_H = 0x43;
  static const byte gyro_x_L = 0x44;
  static const byte gyro_y_H = 0x45;
  static const byte gyro_y_L = 0x46;
  static const byte gyro_z_H = 0x47;
  static const byte gyro_z_L = 0x48;
} r; // change back to reg in the real code

struct Bits { // bit values for the imu
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
} bits;

struct Mag_Reg { // register addresses for the magnetometer - AK8963
  static const byte cntl1 = 0x0A; // control 1 register
  static const byte cntl2 = 0x0B; // control 2 register

  // following registers give data in each direction
  // each direction measurement is 2 bytes long and is deposited in memory starting at the low byte
  static const byte x_L = 0x03;
  static const byte x_H = 0x04;
  static const byte y_L = 0x05;
  static const byte y_H = 0x06;
  static const byte z_L = 0x07;
  static const byte z_H = 0x08;

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

const byte addr = 0b1101000; // imu address
const byte mag_addr = 0x0C; // magnetometer address - if i2c bypass on imu is on

int writeToRegister(byte rgstr, byte* data, int size) {
  Wire.beginTransmission(addr);
  Wire.write(rgstr);
  for (int i = 0; i < size; i++) {
    Wire.write(data[i]);
  }
  return Wire.endTransmission();
}
int writeToRegister(byte rgstr, byte* data, int size, byte addr) { // write to different address
  Wire.beginTransmission(addr);
  Wire.write(rgstr);
  for (int i = 0; i < size; i++) {
    Wire.write(data[i]);
  }
  return Wire.endTransmission();
}
int readFromRegister(byte rgstr, byte* buffer, int amount) { // returns how many bytes read
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
int readFromRegister(byte rgstr, byte* buffer, int amount, byte addr) { // read from different address
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
void reset() { // send reset command to device
  // this will be done by setting the H_RESET bit in power management register 1
  byte data = bits.H_RESET;
  if ( writeToRegister(r.pwr_mgmt_1, &data, 1) ) {
    Serial.println("Failure to send reset command!");
  } else {
    Serial.println("No sign of failure for reset!");
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
  if (writeToRegister(r.config, &data, 1)) {
    // how will we respnod to this FAILURE!?
    Serial.println("Failure to set lpf!");
  }
}
void setupIMUInterrupt() {
  byte data = 0;
  data |= bits.OPEN_POP; // open drain configuration as the arduino has it's own pull up resistor
  data |= bits.LATCH_INT_EN; // interrupt status remains until int_status is cleared
  //data |= bits.INT_STATUS_CLEAR; // will make int status get cleared with any read instead of clearing when int_status register is read
  data |= bits.ACTL; // active low interrupts
  
  if (writeToRegister(r.int_bypass_cfg, &data, 1)) {
    Serial.println("Failure to write to int_bypass_cfg register!");
  } /* */
  
  data = bits.RAW_DATA_EN;
  if (writeToRegister(r.int_enable, &data, 1)) {
    Serial.println("Failure to send int enable bits!");
  }
}
int set_bypass(int state) { // bypass the mpu to be able to access the magnetometer directly
  byte data;
  if (state) {
    // only change the bypass value of the int/bypass register
    // so we must know the original state of the register
    if (readFromRegister(r.int_bypass_cfg, &data, 1)) { // readFromRegister is non-zero if success as it returns bytes read
      data |= bits.BYPASS_EN;
      if (writeToRegister(r.int_bypass_cfg, &data, 1)) {
        return 0;
      }
    }
  } else {
    if (readFromRegister(r.int_bypass_cfg, &data, 1)) { // readFromRegister is non-zero if success as it returns bytes read
      data &= ~bits.BYPASS_EN;
      if (writeToRegister(r.int_bypass_cfg, &data, 1)) {
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
  if (writeToRegister(mag_reg.cntl2, &data, 1, mag_addr)) {
    Serial.println("Failure to send reset bits!");
  }
  // wait a while for soft reset to occur
  _delay_ms(50);
  // set 100Hz output mode with 16 bits
  data = mag_bits.CONT_MEASUREMENT_2 | mag_bits.OUTPUT_16;
  writeToRegister(mag_reg.cntl1, &data, 1, mag_addr);

  // get sensitivity values for conversion from raw measurement to micro Tesla
  // sensitivity values are stored in order x y z
  byte sens[3];
  readFromRegister(mag_reg.sens_x, sens, 3, mag_addr);
  mag_sens.x = sens[0]; mag_sens.y = sens[1]; mag_sens.z = sens[2];

  Serial.print("Sensitivity Values Magnetometer: "); Serial.print(mag_sens.x); Serial.print(" "); Serial.print(mag_sens.y); Serial.print(" "); Serial.println(mag_sens.z);
  
  // we are done interacting directly with magnetometer at this point
  set_bypass(0);
  
  // interrupts only trigger when external sensor data is loaded. also multiple masters can operate on lines
  data = bits.WAIT_FOR_ES | bits.MULT_MST_EN;
  writeToRegister(r.i2c_mst_ctrl, &data, 1);

  // setup the slave 0 handler to read from magnetometer
  data = bits.SLV_READ;
  data |= mag_addr; // in the same register, we put the address
  writeToRegister(r.slv0_addr, &data, 1);
  // register to start reading from - we'll do x measurement low register since that is first in full measurement sequence which is 6 bytes long - two bytes for each dimension
  data = mag_reg.x_L;  
  writeToRegister(r.slv0_reg, &data, 1);
  
  // enable slave 0 handler
  data = bits.SLV_EN;
  data |= 7; // finally, tell that 7 bytes total in measurement data (two bytes for each dimension) must be read from the starting register address; extra byte is to read status_2 register which will make the sensor take another measurement
  writeToRegister(r.slv0_ctrl, &data, 1);

  // enable i2c master
  data = bits.I2C_MST_EN;
  writeToRegister(r.user_ctrl, &data, 1);
}
void setup() {
  pinMode(int_pin, INPUT_PULLUP);
  Wire.begin();
  Serial.begin(9600);

  reset();
  _delay_ms(200);
  byte data = 0x00;
  writeToRegister(r.pwr_mgmt_1, &data, 1);/* */

  // put the damn thing to sleep for startup
  data = bits.BIT_SLEEP;
  writeToRegister(r.pwr_mgmt_1, &data, 1);
  _delay_ms(100);
  data = 0x00;
  writeToRegister(r.pwr_mgmt_1, &data, 1);
  
  // choose best clock to use timing
  data = bits.AUTO_CLCK_SEL;
  writeToRegister(r.pwr_mgmt_1, &data, 1);

  // suggested code for setting sample rate:
  #define smpl_rate 4 // sample rate has to be between 4 hz and 1000 hz
  data = 1000 / smpl_rate - 1;
  writeToRegister(r.smplrt_div, &data, 1);
  set_lpf(smpl_rate >> 1);
  // we have to set the low pass filter as well 

  // set up our own full scale range
  // b10 means 8 gs in the correct register
  data = 0b10;
  data <<= 3; // put in correct place to set the accel fsr
  writeToRegister(r.accel_config, &data, 1);
  // gyro fsr
  // b00 means 250 fsr
  // b11 means 2000 fsr
  data = 0b00;
  data <<= 3; // put in correct place for gyro fsr
  writeToRegister(r.gyro_config, &data, 1);
  
  setupIMUInterrupt();
  setupMagnetometer();
}

void loop() {
  _delay_ms(10);

  // accel, gyro readings are all 16 bit values stored in two registers that are each one byte long
  // and each MSB byte register has it's corresponding LSB register after
  // magnetometer readings are also 16 bit values but LSB comes before the MSB
  // we'll store these in float values which are 32 bits long (enough to store the value)
 
  if ( digitalRead(int_pin) == LOW ) { // only if new data has been loaded from the sensor 
    byte raw_data[6]; // raw data for each dimension for accel, gyro, mag each being 2 bytes long so in total 6 bytes
    //byte data = 0; // store current readings from sensor
    if (readFromRegister(r.int_status, raw_data, 1) == 0) { // read int status register
      Serial.println("Failure to read from int status register!");
    }
    if ((bits.RAW_DATA_RDY_INT & raw_data[0]) == 0) { // check if interrupt occured because raw data was ready
      Serial.println("Data not ready!");
      return;
    }

    // Accel Reading Section
    float accel[3]; // axies values will be stored in order x,y,z
    readFromRegister(r.accel_x_H, raw_data, 6);
    for (int i = 0; i < 3; i++) {
      accel[i] = raw_data[2*i] << 8 | raw_data[2*i + 1]; // convert the 2 byte values for each dimension into a single float value for conversion (while swapping low and high bytes)
    }
    calcAccel(accel, 8); // 8g full scale range
    
    // Gyro calculation section (this method for getting the values is better)
    // fsr for gyro currently is 250 DPS
    float gyro[3]; // gyro values stored in order x y z
    readFromRegister(r.gyro_x_H, raw_data, 6);
    for (int i = 0; i < 3; i++) {
      gyro[i] = raw_data[2*i] << 8 | raw_data[2*i+1]; // convert the 2 byte values for each dimension into a single float value for conversion (while swapping low and high bytes)
    }
    calcGyro(gyro, 250); // 250 dps full scale range

    // magnetometer readings
    float mag[3];
    readFromRegister(r.ext_sens_data_00, raw_data, 6); // magnetometer is a sensor being read from by the IMU in master mode so we're reading from a register that got its data from the magnetometer
    for (int i = 0; i < 3; i ++) { // convert the 2 byte values for each dimension into a single float value for conversion
      mag[i] = raw_data[2*i + 1] << 8 | raw_data[2*i];
    }
    calcMag(mag, 16);
    
    Serial.print("Accel X Data!:"); Serial.println(accel[0]);
    Serial.print("Accel Y Data!:"); Serial.println(accel[1]);
    Serial.print("Accel Z Data!:"); Serial.println(accel[2]);
    Serial.print("Gyro X Data!:"); Serial.println(gyro[0]);
    Serial.print("Gyro Y Data!:"); Serial.println(gyro[1]);
    Serial.print("Gyro Z Data!:"); Serial.println(gyro[2]);
    Serial.print("Magnetometer X Data!:"); Serial.println(mag[0]);
    Serial.print("Magnetometer Y Data!:"); Serial.println(mag[1]);
    Serial.print("Magnetometer Z Data!:"); Serial.println(mag[2]);
  }
}
