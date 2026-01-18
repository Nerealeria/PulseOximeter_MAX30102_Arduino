

#include <Wire.h>    // This library allows I2C communication
#include "MAX30105.h" // Library 
MAX30105 bioSensor;

int devicesConnected = 0;
long samples_Taken = 0;
bool PLOT_MODE = true;

 // Adaptive threshold behaviour 
static float ampEMA = 0; 
static const float ampAlpha = 0.05f; 
// Peak detector state
static int32_t prev_prev = 0;  // Corresponds to sample n-2
static int32_t prev = 0;       // Corresponds to sample n-1
static bool armed = true;
static int32_t recentMax = 0;
// Finger state 
static bool fingerPresent = false;
static bool fingerPresent_Prev = false; 
// Setup timing after finger is placed
static unsigned long startTime = 0;
// Reset flag for filters 
static bool resetFilterState = false;

// PROTOTYPES
int countDevices();
int32_t removeDC(uint32_t ir_raw);
int32_t smoothSignal(int32_t ir_ac);
void resetFilters();
int calculationBPM(bool peakDetected, bool fingerPresent);


void setup() {
  
  Serial.begin(115200); 
  delay(1500);
  Wire.begin();

  if(!PLOT_MODE) {
    Serial.println("ESP32 ready.");
    Serial.println("I2C initialized.");
    int devicesConnected = 0;
    countDevices();
    if (devicesConnected == 0){
    Serial.println("No I2C devices found.");
    }else{
    Serial.println("Search of devices finished.");
    }
    delay(1000);
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
  resetFilters();

}
 
void loop() {

  bioSensor.check();
  while(bioSensor.available()){
    samples_Taken++;
    //Serial.print("Red = ");
    uint32_t red = bioSensor.getFIFORed();
    uint32_t ir_raw = bioSensor.getFIFOIR();

    // Finger detection
    if(!fingerPresent_Prev){
      fingerPresent = (ir_raw > 50000); 
      // This value obtained from observing the values of ir_raw while the finger was pressed on the sensor
    }else{
      fingerPresent = (ir_raw > 30000);
    } 
    
    if (fingerPresent != fingerPresent_Prev){ // Checks if finger state changes
      resetFilters(); 
      startTime = millis();
    }
    fingerPresent_Prev = fingerPresent;

    // Filtering 
    int32_t ir_ac = removeDC(ir_raw);
    int32_t ir_filtered = smoothSignal(ir_ac);
    // The value ir_filtered corresponds to the current value
    if (ir_filtered > recentMax){
      recentMax = ir_filtered; 
    }

    // Adaptive threshold 
    ampEMA = (1.0f - ampAlpha) * ampEMA + ampAlpha * (float)abs(ir_filtered);
    int32_t threshold_High = (int32_t)max(30.0f, 0.50f * ampEMA);
    int32_t threshold_Low = (int32_t)max(10.0f, 0.30f * ampEMA);

    // Peak detection 
    bool peakDetected = false;

    if (recentMax > 0 && ir_filtered < (int32_t)(0.6f * recentMax)){
      armed = true;
      recentMax = ir_filtered;
    }
    // Detect if "prev" was a peak only if armed
    if (armed){
      if ((prev > prev_prev) && (prev > ir_filtered) && (prev > threshold_High)){
        peakDetected = true;
        armed = false; // block until signal goes low again
        recentMax = prev;
      }
    }

    prev_prev = prev;  // This shifts samples for next iteration
    prev = ir_filtered; 

    // BPM calculation
    int BPM = 0;
    if(fingerPresent){
      BPM = calculationBPM(peakDetected, fingerPresent);
    }else{
      BPM = 0; //Show 0 when there is no finger over sensor
    }

    

    //Serial.print("RED:");
    //Serial.print(red);
    //Serial.print("  ");
    
    //Serial.print(" ");
    //Serial.print("IR_ac:");
    //Serial.print(ir_ac * 50);
    //Serial.print(" "); 
    //Serial.print("IR_raw:");
    //Serial.print(ir_raw);
    Serial.print(" Finger:");
    Serial.print(fingerPresent);
    Serial.print(" IR_filtered:");
    Serial.print(ir_filtered);
    Serial.print(" amp:"); 
    Serial.print((int)ampEMA); 
    Serial.print(" thHIGH:");
    Serial.print(threshold_High); 
    Serial.print(" thLOW:"); 
    Serial.print(threshold_Low);
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

void resetFilters(){
  resetFilterState = true; 
  ampEMA = 0; 
  armed = true; 
  prev = 0; 
  prev_prev = 0;
  recentMax = 0;
}

int32_t removeDC(uint32_t ir_raw){
  const int N = 100; // DC baseline window lenght, 1s window at 100 Hz
  static uint32_t buffer[N]; // Stores last N samples, circular buffer
  static uint64_t sum = 0;
  static int i = 0; 
  static int count = 0;
  
  if (resetFilterState){
    for (int a = 0; a < N; a++){
      buffer[a] = 0; 
    }
    sum = 0; 
    i = 0; 
    count = 0;
  }

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

  if (resetFilterState){
    for (int a = 0; a < M; a++){
      buffer[a] = 0; 
    }
    sum = 0; 
    i = 0; 
    count = 0;
    resetFilterState = false; // Cleared after both filters, DC removal and smoothing have seen the flag
  }
  
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
  const unsigned long setup_ms = 1500; 
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
    }
    return 0;
  }

  // Setup time after finger is placed 
  if (now - startTime < setup_ms){
    lastBeatTime = 0; 
    IBI_i = 0; 
    IBI_count = 0; 
    BPM = 0;
    return 0;
  }
  if (!peakDetected){
    return BPM; // If no peak has been detected it return the last BPM 
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

  if(IBI < 300 || IBI > 2000) return BPM; // Limits possible values of BPM (30...200 BPM)

  IBI_buffer[IBI_i] = IBI; 
  IBI_i = (IBI_i +1) % N; 
  if(IBI_count < N) IBI_count++;

  // Necessary to calculate average IBI
  if(IBI_count >= 3){
    for(int i = 0; i < IBI_count; i++){
    IBI_sum += IBI_buffer[i];
    }
    unsigned long IBI_average = IBI_sum/(unsigned long)IBI_count;
    BPM = (int)(60000UL/IBI_average);
  }
  
  return BPM;

}


