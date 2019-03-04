#include <MacRocketry_BMP_180.h>   //include header file

MacRocketry_BMP_180 bmp;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if(bmp.readData()){
    Serial.print("Temperature: "); Serial.println(bmp.getTemperature());
    Serial.print("Pressure: "); Serial.println(bmp.getPressure());
    Serial.print("Altitude: "); Serial.println(bmp.getAltitude());
  }
}
