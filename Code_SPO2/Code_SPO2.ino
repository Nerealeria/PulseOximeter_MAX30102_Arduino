

#include <Wire.h>    // This library allows I2C communication
#include "MAX30105.h" // Library 
MAX30105 bioSensor;


int devicesConnected = 0;
long samples_Taken = 0;
bool PLOT_MODE = true;

int countDevices();
int32_t removeDC(uint32_t IR_raw);


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

  const byte LED_Brightness = 70;  // Range between 0 - 255 
  const byte LED_Mode = 2;
  const byte sample_Average = 4; // Possible 1,2,4,8,16,32
  const int sample_Rate = 100;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  const int pulse_Width = 411;  // Possible 69,118,215,411
  const int ADC_Range = 16384; // Options: 2048, 4096, 8192, 16384

  bioSensor.setup(LED_Brightness, sample_Average, LED_Mode, sample_Rate, pulse_Width, ADC_Range);


}
 
void loop() {

  bioSensor.check();
  while(bioSensor.available()){
    samples_Taken++;
    //Serial.print("Red = ");
    uint32_t red = bioSensor.getFIFORed();
    uint32_t ir_raw = bioSensor.getFIFOIR();
    int32_t ir_ac = removeDC(ir_raw);
    int32_t ir_filtered = smoothSignal(ir_ac);

    //Serial.print("RED:");
    //Serial.print(red);
    //Serial.print("  ");
    //Serial.print("IR_raw:");
    //Serial.print(ir_raw);
    //Serial.print(" ");
    Serial.print("IR_ac:");
    Serial.print(ir_ac * 50);
    Serial.print(" "); 
    Serial.print("IR_filtered:");
    Serial.println(ir_filtered * 50);

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

int32_t removeDC(uint32_t ir_raw){
  const int N = 100; // DC baseline window lenght, 1s window at 100 Hz
  static uint32_t buffer[N]; // Stores last N samples, circular buffer
  static uint64_t sum = 0;
  static int i = 0; 
  static int count = 0;
  
  sum -= buffer[i]; // This removes the oldest value from sum
  buffer[i] = ir_raw;
  sum += ir_raw; 
  i = (i+1) % N; // Update index in a circular way
  if (count < N) count++;
  int32_t dc = sum/count;
  int32_t ac = ir_raw - dc;
  return ac;
}

int32_t smoothSignal(int32_t ir_ac){

  const int M = 5; 
  static uint32_t buffer[M]; 
  static uint64_t sum = 0;
  static int i = 0; 
  static int count = 0;
  
  sum -= buffer[i]; // This removes the oldest value from sum
  buffer[i] = ir_ac;
  sum += ir_ac; 
  i = (i+1) % M; // Update index in a circular way
  if (count < M) count++;
  int32_t ir_filtered = sum/count;
  return ir_filtered;
}
