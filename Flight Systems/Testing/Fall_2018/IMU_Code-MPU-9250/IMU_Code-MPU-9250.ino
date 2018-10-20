#include <Wire.h> // for i2c communication

struct Reg { // stores register values to for the mpu 9250
  static const byte pwr_mgmt_1 = 0x6B;
  static const byte pwr_mgmt_2 = 0x6C;
  static const byte config = 0x1A;
  static const byte smplrt_div = 0x19;
  static const byte accel_config = 0x1C;
  static const byte accel_config_2 = 0x1D;
  static const byte int_pin_cfg = 0x37;
  static const byte int_enable = 0x38;
  static const byte i2c_mst_ctrl = 0x24;
  
  static const byte int_status = 0x3A;
  // right now we'll only be reading the z output
  static const byte accel_xout_H = 0x3B;
  static const byte accel_xout_L = 0x3C;
  static const byte accel_yout_H = 0x3D;
  static const byte accel_yout_L = 0x3E;
  static const byte accel_zout_H = 0x3F;
  static const byte accel_zout_L = 0x40;
} reg;

struct Bits {
  // pwr mgmt 1 bits
  static const byte H_RESET = 1 << 7;
  static const byte AUTO_CLCK_SEL = 1;

  // int status bits
  static const byte RAW_DATA_RDY_INT = 0x01;

  // int enable bits
  static const byte RAW_DATA_EN = 1; // interrupt when raw data is available

  // int pin configuration bits
  static const byte ACTL = 1 << 7; // active low set
  static const byte OPEN_POP = 1 << 6;  // open drain or push or pull configuration
  static const byte LATCH_INT_EN = 1 << 5; // whether to hold interrupt until int_status is read or if to do a 50 us interrupt
  static const byte INT_STATUS_CLEAR = 1 <<  4;
  

  // i2c master control bits
  static const byte WAIT_FOR_DATA_READY = 1 << 6; // if set, interrupt only triggers when data is completely loaded into registers
} bits;

const int ad0 = 0; // state of the ad0/sd0 pin - used to determine sensor address

const byte addr = 0b1101000 + ad0; // address of the mpu 9250

bool failure = false; // if for some reason the device cannot be used anymore, the loop will start spamming "failure" into serial

int writeToRegister(byte rgstr, byte* data, int size) {
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

void reset() { // send reset command to device
  // this will be done by setting the H_RESET bit in power management register 1
  byte data = bits.H_RESET;
  if ( writeToRegister(reg.pwr_mgmt_1, &data, 1) ) {
    Serial.println("Failure to send reset command!");
    failure = true;
  } else {
    Serial.println("No sign of failure for reset!");
  }
}

void setArduinoInterrupt() {
  // we'll make the arduino get triggered on active low on digital pin 2
  // we have to make INT0 (digital pin 2) an input pin
  DDRD |= (1 << DDD2);

  PORTD |= (1 << PORTD2); // enable pull up resistors on digital pin 2
  
  // enable active low interrupt on pin INT0 which is digital pin 2
  EICRA &= ~( (1 << ISC00) | (1 << ISC01) );
  EIMSK |= 1 << INT0; // enable INT0 interrupts

  sei(); // enable interrupts to occur
}

volatile bool newData = false;

// interrupt service routine for INT0 interrupts
ISR (INT0_vect) {
  EIFR |= (1 << INTF0); // clear interrupt flag
  newData = true;
}

void setupInterrupt() {
  // first we'll modify the interrupt configuration to suite our needs
  byte data = 0;
  data |= bits.ACTL; // active low interrupts
  data |= bits.OPEN_POP; // open drain configuration as the arduino has it's own pull up resistor
  data |= bits.LATCH_INT_EN; // interrupt status remains until int_status is cleared
  //data |= bits.INT_STATUS_CLEAR; // will make int status get cleared with any read instead of clearing when int_status register is read

  if (writeToRegister(reg.int_pin_cfg, &data, 1)) {
    Serial.println("Failure to write to int_pin_cfg register!");
    failure = true;
  } /* */

  // now we can set the int enable register to enable interrupts when raw data is available

  data = 0;
  data |= bits.RAW_DATA_EN; // bit to enable raw data interrupt

  if (writeToRegister(reg.int_enable, &data, 1)) {
    Serial.println("Failure to write to int_enable register!");
    failure = true;
  }
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);

  // sensor setup
  
  reset();
  Serial.println("HALP!");
  
  _delay_ms(10); // wait a while for reset before clearing power management register
  // clear pwr managment by simply sending 0
  byte data = 0x00;
  if(writeToRegister(reg.pwr_mgmt_1, &data, 1)) {
    Serial.println("Failure to clear power managment register!");
    failure = true;
  }
  
  _delay_ms(10);
  // let clock be selected automatically
  data |= bits.AUTO_CLCK_SEL;
  if (writeToRegister(reg.pwr_mgmt_1, &data, 1)) {
    Serial.println("Failure to set clck pin!");
    failure = true;
  }
  // going minimalistic - reset configuration register
  data = 0x00;
  writeToRegister(reg.config, &data, 1);

  // set full scale range for accel to be 8g
  byte fsr = 0b10; // means 8g
  data = fsr << 3;
  if (writeToRegister(reg.accel_config, &data, 1)) {
    Serial.println("Failure to set full scale range for acceleration!");
    failure = true;
  }
  _delay_ms(50);


  // interrupts testing
  setArduinoInterrupt();
  setupInterrupt();
}

float calcAccel(int raw_data, int fsr) {
  int convDiv = 1 << 14; // conversion divisor for 2g
  // for each double in FSR from 2, our conversion divisor must get divided by two to accomodate for bigger FSR during conversion
  // we can just divide our given FSR by 2 to find out what to divide our conversion divisor by
  fsr /= 2;
  convDiv /= fsr;

  return (float) raw_data / (float) convDiv;
}

void loop() {
  _delay_ms(800);

  /*
  if (digitalRead(2) == 25) {
    Serial.println("Digital pin 2 gone low!");
  } else {
    Serial.println("Digital pin 2 is high!");
  }       /* This comment makes commenting easier */

 
  if (1) { // only if new data has been loaded from the sensor
    newData = false;
    byte data = 0; // store current readings from sensor
    if (readFromRegister(reg.int_status, &data, 1) == 0) { // read int status register
      failure = true;
    }
    if ((bits.RAW_DATA_RDY_INT & data) == 0) { // check if interrupt occured because raw data was ready
      Serial.println("Data not ready!");
      return;
    }
    float accel[3]; // axies values will be stored in order x,y,z
    byte currentReg = reg.accel_xout_H; // acceleration output is stored in a block starting with accel_xout_H, in order x,y,z
    // in order to access the other registers we'll just add 1
    // and each axis (x,y,z) is stored in two registers to get the high accuracy 16 bit numbers

    for (int i = 0; i < 3; i++) { // 3 since 3 axies to read from
      byte reg_H; // 8 MSBs of accel data
      byte reg_L; // 8 LSBs of accel z data

      readFromRegister(currentReg, &reg_H, 1); // read most significant bits of accel z data
      currentReg++;
      if (readFromRegister(currentReg, &reg_L, 1) == 0) { // read least significant bits of accel z data
        Serial.println("Failure to read accel bits!");
        failure = true;
      }
      currentReg++;
      
      // put em together to create raw z accel value
      accel[i] = (float) ((reg_H << 8) + reg_L);
      // tell us the value (after conversion)!
      accel[i] = calcAccel(accel[i], 8); // 8 for 8g FSR
    }
    Serial.print("Accel X Raw Data!:"); Serial.println(accel[0]);
    Serial.print("Accel Y Raw Data!:"); Serial.println(accel[1]);
    Serial.print("Accel Z Raw Data!:"); Serial.println(accel[2]);
  }
}

void loop_old() {
  // put your main code here, to run repeatedly:
  // check status of sensor
  byte data = 0; // by reading int status register
  if (readFromRegister(reg.int_status, &data, 1) == 0) {
    Serial.println("Failure to read from int_status register!");
    failure = true;
  }
  // if the first bit is set, which is RAW_DATA_RDY_INT, we have new data to be read!
  if ((bits.RAW_DATA_RDY_INT & data)) {
    float accel[3]; // axies values will be stored in order x,y,z
    byte currentReg = reg.accel_xout_H; // acceleration output is stored in a block starting with accel_xout_H, in order x,y,z
    // in order to access the other registers we'll just add 1
    // and each axis (x,y,z) is stored in two registers to get the high accuracy 16 bit numbers

    for (int i = 0; i < 3; i++) { // 3 since 3 axies to read from
      byte reg_H; // 8 MSBs of accel data
      byte reg_L; // 8 LSBs of accel z data

      readFromRegister(currentReg, &reg_H, 1); // read most significant bits of accel z data
      currentReg++;
      if (readFromRegister(currentReg, &reg_L, 1) == 0) { // read least significant bits of accel z data
        Serial.println("Failure to read accel bits!");
        failure = true;
      }
      currentReg++;
      
      // put em together to create raw z accel value
      accel[i] = (float) ((reg_H << 8) + reg_L);
      // tell us the value (after conversion)!
      accel[i] = calcAccel(accel[i], 8); // 8 for 8g FSR
    }
    Serial.print("Accel X Raw Data!:"); Serial.println(accel[0]);
    Serial.print("Accel Y Raw Data!:"); Serial.println(accel[1]);
    Serial.print("Accel Z Raw Data!:"); Serial.println(accel[2]);
  } else {
    // if not we'll just wait a bit
    _delay_ms(50);
  }
  //in case that device cannot be read from
  while (failure) {
    Serial.println("Failure!");
    _delay_ms(3000);
  }
}
