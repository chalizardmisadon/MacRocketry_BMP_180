/*
This BMP library is based on SFE_BMP180.h by Mike Grusin, SparkFun Electronics
This library is rewritten to suit the need and design for McMaster Rocketry Team
*/

#ifndef MacRocketry_BMP_180_h
#define MacRocketry_BMP_180_h

//I2C addresses
#define BMP180_ADDR 0x77 // default 7-bit address

#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6

#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE_0 0x34
#define BMP180_COMMAND_PRESSURE_1 0x74
#define BMP180_COMMAND_PRESSURE_2 0xB4
#define BMP180_COMMAND_PRESSURE_3 0xF4

enum BMPState {
  BMP_Init,
  BMP_ReadPressure_StartTemperature,
  BMP_ReadTemperature_StartPressure
};

class MacRocketry_BMP_180
{
  public:
    MacRocketry_BMP_180(void);
    MacRocketry_BMP_180(char oversampling); //set oss

    bool begin(void);
    bool readData(void);
    
    char startTemperature(void);
    bool readTemperature(void); //temperature in hPa / mbars
    char startPressure(void);
    bool readPressure(void);
    
    float calcSeaLevel(float P, float A);
    float calcAltitude(float P, float P0);
    bool getError(void);

    //getters and setters
    bool getConnectBMP(void);
    float getTemperature(void);
    float getPressure(void);
    float getAltitude(void);
    
    void setSeaLevel_hPa(int p);
    void setSeaLevel_kPa(int p);
    void setOversampling(char oversampling);

    
  private:
  
    bool readBytes(unsigned char *values, char length);
    bool writeBytes(unsigned char *values, char length);
    
    bool readInt(char regAddress, int16_t &value);
    bool readUInt(char regAddress, uint16_t &value);
    
    //calibration variables --------------------
    int16_t AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
    uint16_t AC4,AC5,AC6; 
    float c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;
    char _error;

    //calculation variables --------------------
    bool connectBMP;
    char oss; //oversampling setting
    
    unsigned long waitTimer;
    BMPState state;
    
    float temperature, pressure, seaPressure, altitude;
};


#endif