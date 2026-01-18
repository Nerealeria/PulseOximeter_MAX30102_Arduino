

#include <Wire.h>    // This library allows I2C communication
#include "MAX30105.h" // Library 
MAX30105 bioSensor;


int devicesConnected = 0;
long samples_Taken = 0;
bool PLOT_MODE = true;

int32_t prev_prev = 0;  // Corresponds to sample n-2
int32_t prev = 0;       // Corresponds to sample n-1

static float amp = 0; 
static int32_t threshold_High = 60;  // To detect peaks
static int32_t threshold_Low = 20;  // To re-arm 
static bool armed = true;
bool fingerPresent = false;

unsigned long startTime = 0;


int countDevices();
int32_t removeDC(uint32_t ir_raw);
int32_t smoothSignal(int32_t ir_ac);
int calculateBPM(bool peakDetected, bool fingerPresent);


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
  const byte sample_Average = 4; // Options: 1,2,4,8,16,32
  const int sample_Rate = 100;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  const int pulse_Width = 411;  // Options: 69,118,215,411
  const int ADC_Range = 16384; // Options: 2048, 4096, 8192, 16384

  bioSensor.setup(LED_Brightness, sample_Average, LED_Mode, sample_Rate, pulse_Width, ADC_Range);

  startTime = millis();  // Initializes the start time just once

}
 
void loop() {

  bioSensor.check();
  while(bioSensor.available()){
    samples_Taken++;
    //Serial.print("Red = ");
    uint32_t red = bioSensor.getFIFORed();
    uint32_t ir_raw = bioSensor.getFIFOIR();
    fingerPresent = (ir_raw > 20000);
    int32_t ir_ac = removeDC(ir_raw);
    int32_t ir_filtered = smoothSignal(ir_ac);
    // The value ir_filtered corresponds to the current value

    bool peakDetected = false;

    if (abs(ir_filtered) < threshold_Low){
      armed = true;
    }
    // Detect if "prev" was a peak only if armed
    if (armed){
      if ((prev > prev_prev) && (prev > ir_filtered) && (abs(prev) > threshold_High)){
        peakDetected = true;
        armed = false; // block until signal goes low again
      }
    }
    int BPM = 0;
    if(fingerPresent){
      BPM = calculationBPM(peakDetected, fingerPresent);
    }else{
      BPM = 0; //Show 0 when there is no finger over sensor
      armed = true; // Resets peak detector
      prev = 0; 
      prev_prev = 0;
    }

    prev_prev = prev;  // This shifts samples for next iteration
    prev = ir_filtered; 

    //Serial.print("RED:");
    //Serial.print(red);
    //Serial.print("  ");
    
    //Serial.print(" ");
    //Serial.print("IR_ac:");
    //Serial.print(ir_ac * 50);
    //Serial.print(" "); 
    Serial.print("IR_raw:");
    Serial.print(ir_raw);
    Serial.print(" Finger:");
    Serial.print(fingerPresent);
    Serial.print(" IR_filtered:");
    Serial.print(ir_filtered);
    Serial.print(" Peak:"); 
    Serial.print(peakDetected);
    Serial.print(" BPM:"); 
    Serial.println(BPM);

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
  uint32_t dc = sum/count;
  int32_t ac = (int32_t)ir_raw - (int32_t)dc;
  return ac;
}

int32_t smoothSignal(int32_t ir_ac){
  const int M = 5; 
  static int32_t buffer[M];  // They must be signed
  static int64_t sum = 0;
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

int calculationBPM(bool peakDetected, bool fingerPresent){
  const int N = 5;  // 5 beats to obtain average
  const unsigned long setup_ms = 2000; 
  const unsigned long refract_ms = 350; 
  static unsigned long IBI_buffer[N] = {0};
  static int IBI_i = 0; 
  static int IBI_count = 0;
  unsigned long IBI_sum = 0;
  static unsigned long lastBeatTime = 0; 
  static int BPM = 0;
  unsigned long now = millis();
  
  if(!fingerPresent){
    lastBeatTime = 0; 
    BPM = 0; 
    IBI_i = 0;
    IBI_count = 0;
    for(int i = 0; i < N; i++){
      IBI_buffer[i] = 0; 
      return 0;
    }
  }

  // To calibrate the sensor, a setup phase is needed
  if (now - startTime < setup_ms){
    if (peakDetected) lastBeatTime = now; 
    return 0;
  }
  if (!peakDetected){
    return BPM; // If no peak has been detected BPM is still 0
  } 
  // Refractory period, it ignores peaks too close to each other
  if (lastBeatTime != 0 && (now - lastBeatTime) < refract_ms){
    return BPM; 
  }
  // First valid beat
  if (lastBeatTime == 0){
    lastBeatTime = now; 
    return BPM;
  }
  unsigned long IBI = now - lastBeatTime; // Interbeat Interval, is the exact time period between consecutive heartbeats
  lastBeatTime = now;
  if(IBI < 300 || IBI > 2000) return BPM;
  IBI_buffer[IBI_i] = IBI; 
  IBI_i = (IBI_i +1) % N; 
  if(IBI_count < N) IBI_count++;

  // Necessary to calculate average IBI
  for(int i = 0; i < IBI_count; i++){
    IBI_sum += IBI_buffer[i];
  }
  unsigned long IBI_average = IBI_sum/IBI_count;
  BPM = 60000/IBI_average;
  return BPM;

}


