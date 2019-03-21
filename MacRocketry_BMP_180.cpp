/*
This BMP library is based on MacRocketry_BMP_180.h by Mike Grusin, SparkFun Electronics
This library is rewritten to suit the need and design for McMaster Rocketry Team
*/

#include <Arduino.h>                //include Arduino library
#include <MacRocketry_BMP_180.h>    //include header file
#include <Wire.h>                   //Wire library needed for I2c


//constructor --------------------
MacRocketry_BMP_180::MacRocketry_BMP_180(void){ //constructor
  #ifdef BMP180_NO_DELAY_READ
  state = BMP_Init;
  #endif
  
  oss = 0;                  //default oversampling
  setSeaLevel_hPa(1013.25); //default sea level
  
  //connectBMP = begin(); //there is a bug where Wire.begin() cannot be called within a class contructor
  //therefore, we have to call MacRocketry_BMP_180::begin() inside setup() function
  //so this line above is commented out, and connectBMP is evaluated inside MacRocketry_BMP_180::begin() instead

}

//getters and settes --------------------
bool MacRocketry_BMP_180::getConnectBMP(void){ return connectBMP; }
float MacRocketry_BMP_180::getTemperature(void){ return temperature; }
float MacRocketry_BMP_180::getPressure(void){ return pressure; }
float MacRocketry_BMP_180::getAltitude(void){ return altitude; }
uint32_t MacRocketry_BMP_180::getTime(void){ return time; }

void MacRocketry_BMP_180::setSeaLevel_hPa(int p){ seaPressure = p; }
void MacRocketry_BMP_180::setSeaLevel_kPa(int p){ seaPressure = p * 10; }
void MacRocketry_BMP_180::setOversampling(char oversampling){ oss = oversampling; }

//initialize BMP and calibration --------------------
bool MacRocketry_BMP_180::begin(){
  //https://github.com/esp8266/Arduino/issues/3570
  //there is a bug where Wire.begin() cannot be called within a class contructor
  //therefore, we have to call MacRocketry_BMP_180::begin() inside setup()
  
  Wire.begin(); //start up the Arduino's "wire" (I2C) library

  //retrieve calibration data stored from device
  if (
    readInt(0xAA,AC1) &&
    readInt(0xAC,AC2) &&
    readInt(0xAE,AC3) &&
    
    readUInt(0xB0,AC4) &&
    readUInt(0xB2,AC5) &&
    readUInt(0xB4,AC6) &&
    
    readInt(0xB6,VB1) &&
    readInt(0xB8,VB2) &&
    readInt(0xBA,MB) &&
    readInt(0xBC,MC) &&
    readInt(0xBE,MD)
  ){
    
    //compute floating-point polynominals http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf

    //first three fp values are used in computing final constants below and is discarded afterward
    float c3, c4, b1;
    c3 = 160.0 * pow(2,-15) * AC3;
    c4 = pow(10,-3) * pow(2,-15) * AC4;
    b1 = pow(160,2) * pow(2,-30) * VB1;
    
    //next four constant are used in temperature computation
    c5 = (pow(2,-15) / 160) * AC5;
    c6 = AC6;
    mc = (pow(2,11) / pow(160,2)) * MC;
    md = MD / 160.0;
    
    //three second order polynomials are used to compute pressure
    //they require another nine constants (three each per polynomial)
    x0 = AC1;
    x1 = 160.0 * pow(2,-13) * AC2;
    x2 = pow(160,2) * pow(2,-25) * VB2;
    
    y0 = c4 * pow(2,15);
    y1 = c4 * c3;
    y2 = c4 * b1;
    
    p0 = (3791.0 - 8.0) / 1600.0;
    p1 = 1.0 - 7357.0 * pow(2,-20);
    p2 = 3038.0 * 100.0 * pow(2,-36);
    
    connectBMP = true; //success!
  } else {
    connectBMP = false; //error reading calibration data; bad component or connection
  }
  return connectBMP;
}


//read BMP data state machine --------------------
bool MacRocketry_BMP_180::readData(){
  delay(startTemperature());  //delay_block for BMP
  readTemperature();          //read temperature
  delay(startPressure());     //delay_block for BMP
  readPressure();             //read pressure
  time = millis();            //set time right away
  altitude = calcAltitude(pressure, seaPressure);
  return true;
}

//functions for reading and writing I2C data from BMP --------------------

bool MacRocketry_BMP_180::readBytes(unsigned char *data, char dataLength){
  
  Wire.beginTransmission(BMP180_ADDR); //start I2C communication
  Wire.write(data[0]);
  _error = Wire.endTransmission(); //flush buffer
  
  if (_error == 0){
    Wire.requestFrom(BMP180_ADDR, dataLength); //request data
    while(Wire.available() < dataLength); // wait until bytes are ready
    
    for (char i = 0; i < dataLength; i++){ //start reading
      data[i] = Wire.read();
    }
    return(1);
  }
  return(0);
}

bool MacRocketry_BMP_180::readInt(char regAddress, int16_t &value){
  unsigned char data[2];

  data[0] = regAddress;
  if (readBytes(data, 2)){ //read signed integer (2 bytes)
    value = (int16_t)((data[0]<<8)|data[1]); //store result in value
    return(1);
  }
  value = 0;
  return(0);
}


bool MacRocketry_BMP_180::readUInt(char regAddress, uint16_t &value){
  unsigned char data[2];

  data[0] = regAddress;
  if (readBytes(data, 2)){ //read unsigned integer (2 bytes)
    value = (((uint16_t)data[0]<<8)|(uint16_t)data[1]); //store result in value
    return(1);
  }
  value = 0;
  return(0);
}


bool MacRocketry_BMP_180::writeBytes(unsigned char *data, char dataLength){
  
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(data, dataLength); //write data array
  _error = Wire.endTransmission(); //flush buffer
  
  if (_error == 0) return(1);
  else return(0);
}

//functions for reading measurement data from BMP --------------------

char MacRocketry_BMP_180::startTemperature(void){ //return delay in ms to wait, or 0 if I2C error
  unsigned char data[2];

  data[0] = BMP180_REG_CONTROL;
  data[1] = BMP180_COMMAND_TEMPERATURE;
  
  if (writeBytes(data, 2)) //good write?
    return(5); //return the delay in ms (rounded up) to wait before retrieving data
  else
    return(0); // or return 0 if there was a problem communicating with the BMP
}


bool MacRocketry_BMP_180::readTemperature(void){
  //requires startTemperature() to have been called prior and sufficient time elapsed
  //retrieve a previously-started temperature reading

  unsigned char data[2];
  float tu, a;
  
  data[0] = BMP180_REG_RESULT;
  if (readBytes(data, 2)){ //send read command
    
    //good read, calculate temperature based on http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
    tu = (data[0] * 256.0) + data[1];
    
    a = c5 * (tu - c6);
    temperature = a + (mc / (a + md));
    return(1);
  }
  return(0);
}


char MacRocketry_BMP_180::startPressure(){
  //oversampling: 0 to 3, higher numbers are slower, higher-res outputs
  //will return delay in ms to wait, or 0 if I2C error
  
  unsigned char data[2], delay;
  data[0] = BMP180_REG_CONTROL;

  switch (oss){
    case 0:
      data[1] = BMP180_COMMAND_PRESSURE_0;
      delay = 5;
    break;
    case 1:
      data[1] = BMP180_COMMAND_PRESSURE_1;
      delay = 8;
    break;
    case 2:
      data[1] = BMP180_COMMAND_PRESSURE_2;
      delay = 14;
    break;
    case 3:
      data[1] = BMP180_COMMAND_PRESSURE_3;
      delay = 26;
    break;
    default:
      data[1] = BMP180_COMMAND_PRESSURE_0;
      delay = 5;
    break;
  }
  
  if (writeBytes(data, 2)) //good write?
    return(delay); // return the delay in ms (rounded up) to wait before retrieving data
  else
    return(0); //return 0 if there was a problem communicating with the BMP
}


bool MacRocketry_BMP_180::readPressure(void){
  //retrieve a previously started pressure reading, calculate abolute pressure in hPa / mbar
  //requires startPressure() to have been called prior and sufficient time elapsed
  //requires recent temperature reading to accurately calculate pressure
  
  //note that calculated pressure value is absolute mbars, to compensate for altitude call calcSeaLevel()
  unsigned char data[3];
  float pu,s,x,y,z;
  
  data[0] = BMP180_REG_RESULT;
  
  if (readBytes(data, 3)){ //send read command

    //good read, calculate pressure based on http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
    pu = (data[0] * 256.0) + data[1] + (data[2] / 256.0);
    
    s = temperature - 25.0;
    x = (x2 * pow(s,2)) + (x1 * s) + x0;
    y = (y2 * pow(s,2)) + (y1 * s) + y0;
    z = (pu - x) / y;
    pressure = (p2 * pow(z,2)) + (p1 * z) + p0;
    
    return(1);
  }
  return(0);
}


float MacRocketry_BMP_180::calcSeaLevel(float P, float A){
  //given a pressure P (hPa / mbar) taken at a specific altitude (meters)
  //return the equivalent pressure (hPa / mbar) at sea level
  return(P / pow(1 - (A / 44330.0), 5.255));
}


float MacRocketry_BMP_180::calcAltitude(float P, float P0){
  //given a pressure measurement P (mb) and the pressure at a baseline P0 (mb)
  //return altitude (meters) above baseline
  return(44330.0 * (1 - pow(P/P0, 1/5.255)));
}

bool MacRocketry_BMP_180::getError(void){
  //if any library command fails, you can retrieve an extended
  //error code using this command. Errors are from the wire library: 
  // 0 = Success
  // 1 = Data too long to fit in transmit buffer
  // 2 = Received NACK on transmit of address
  // 3 = Received NACK on transmit of data
  // 4 = Other error
  return(_error);
}
