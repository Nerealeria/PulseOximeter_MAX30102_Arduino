

#include <Wire.h>    // This library allows I2C communication
#include "MAX30102.h" // Library 
MAX30102 bioSensor;


int devicesConnected = 0;

void setup() {
  
  Serial.begin(115200); // WHY THIS VALUE
  delay(3000);
  Serial.println("ESP32 ready.");

  Wire.begin();
  Serial.println("I2C initialized.");

  while ( !bioSensor.begin(Wire)){
    Serial.println("MAX30102 not found.");
    delay(1000);
  }
  Serial.println("MAX30102 initialized.");

  byte LED_Brightness = 70;

}
 
void loop() {

  devicesConnected = 0;
  countDevices();

  if (devicesConnected == 0){
    Serial.println("No I2C devices found.");
  }else{
    Serial.println("Search of devices finished.");
  }
  delay(3000);
}

int countDevices(){ 
  for (int i = 1; i<127; i++){   
    // As I2C uses 7-bit addresses, it is possible to scan all those addresses to detect 
    // the devices. However, 0x00 is excluded as it is reserved as the general call.
    Wire.beginTransmission(i);
    byte error = Wire.endTransmission();
    // error = 0 means device has been found
    // error != 0 means NO device has been found 
    if (error == 0){
      Serial.print("I2C device found at 0x");
      if (i < 16){
        Serial.print("0");
      }
      Serial.println(i, HEX);
      devicesConnected++;
    }
  }
  return devicesConnected;
}
