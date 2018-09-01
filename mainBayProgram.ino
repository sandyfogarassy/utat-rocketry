#include <SPI.h>
#include <SD.h>
#include <MPU9250.h>
File mainFile;
MPU9250 IMU(Wire, 0x68);
int status;
int timeCounter = 0;
float recData[10];

void setup() {
  Serial.begin(9600);
  while (!Serial){
    ;
  }
  
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  mainFile = SD.open("flight.txt", FILE_WRITE);
  if(mainFile){
    Serial.println("Initialization Successful");
    mainFile.print("Time");
    mainFile.close();
  }
  else{
    Serial.println("Error");
  }

  status = IMU.begin();
  if (status<0){
    Serial.println("IMU initialization failed");
    while(1);
  }
}

void readIMU(float recData[10]){
  IMU.readSensor();
  recData[0] = IMU.getAccelX_mss();
  recData[1] = IMU.getAccelY_mss();
  recData[2] = IMU.getAccelZ_mss();
  recData[3] = IMU.getGyroX_rads();
  recData[4] = IMU.getGyroY_rads();
  recData[5] = (IMU.getGyroZ_rads(),6);
  recData[6] = IMU.getMagX_uT();
  recData[7] = IMU.getMagY_uT();
  recData[8] = IMU.getMagZ_uT();
  recData[9] = IMU.getTemperature_C();
}

void writeMem(float recData[10], int timeCounter){
  mainFile = SD.open("flight.txt", FILE_WRITE);
  mainFile.print(timeCounter);
  for (int i = 1; i < 10; i++){
    mainFile.print(recData[i]);
    mainFile.print("\t");
  }
  mainFile.println("");
  mainFile.close();  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Writing to memory...");
  timeCounter += 1;
  readIMU(recData);
  writeMem(recData, timeCounter);
}
