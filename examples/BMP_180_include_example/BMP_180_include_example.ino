#include <MacRocketry_GPS_Shield.h> //include GPS
#include <MacRocketry_BMP_180.h>    //include BMP

MacRocketry_BMP_180 bmp;
MacRocketry_GPS_Shield gps;

void setup() {
  Serial.begin(9600);
  bmp.begin(); //must call this in setup
}

void loop() {
  if(bmp.readData()){
    Serial.print("Temperature: "); Serial.println(bmp.getTemperature());
    Serial.print("Pressure: "); Serial.println(bmp.getPressure());
    Serial.print("Altitude: "); Serial.println(bmp.getAltitude());
  }
}
