

#include <Wire.h>    // This library allows I2C communication
#include "MAX30105.h" // Library 
MAX30105 bioSensor;


int devicesConnected = 0;
long samples_Taken = 0;
bool PLOT_MODE = true;


void setup() {
  
  Serial.begin(115200); // WHY THIS VALUE
  delay(3000);
  Wire.begin();

  if(!PLOT_MODE) {
    Serial.println("ESP32 ready.");
    Serial.println("I2C initialized.");
    devicesConnected = 0;
    countDevices();
    if (devicesConnected == 0){
    Serial.println("No I2C devices found.");
    }else{
    Serial.println("Search of devices finished.");
    }
    delay(2000);
  }
   

  while ( !bioSensor.begin(Wire)){
    if (!PLOT_MODE) Serial.println("MAX30102 not found.");
    delay(1000);
  }
  if (!PLOT_MODE) Serial.println("MAX30102 initialized.");

  byte LED_Brightness = 70;  // Range between 0 - 255 
  byte LED_Mode = 2;
  byte sample_Average = 4; // Possible 1,2,4,8,16,32
  int sample_Rate = 100;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulse_Width = 411;  // Possible 69,118,215,411
  int ADC_Range = 16384; // Options: 2048, 4096, 8192, 16384

  bioSensor.setup(LED_Brightness, sample_Average, LED_Mode, sample_Rate, pulse_Width, ADC_Range);


}
 
void loop() {

  bioSensor.check();
  while(bioSensor.available()){
    samples_Taken++;
    //Serial.print("Red = ");
    uint32_t red = bioSensor.getFIFORed();
    uint32_t ir = bioSensor.getFIFOIR();
    Serial.print("RED:");
    Serial.print(red);
    Serial.print("  ");
    Serial.print("IR:");
    Serial.println(ir);


    bioSensor.nextSample();  // Sample finished, move to the next sample
  }
  


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
