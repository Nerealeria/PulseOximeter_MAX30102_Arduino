

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

// Setup timing after finger is placed
static unsigned long startTime = 0;
// Reset flag for filters 
static bool resetFilterState = false;

// PROTOTYPES
int countDevices();
void resetFilters();
bool checkFingerPresence(uint32_t ir_raw);
int32_t IR_removeDC(uint32_t ir_raw);
int32_t IR_smoothSignal(int32_t ir_ac);
int32_t RED_removeDC(uint32_t red_raw);
int32_t RED_smoothSignal(int32_t red_ac);
int calculationBPM(bool peakDetected, bool fingerPresent);
int calculationSpO2(uint32_t ir_raw, int32_t ir_ac_filtered, uint32_t red_raw, int32_t red_ac_filtered, bool fingerPresent);


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
    
    uint32_t red_raw = bioSensor.getFIFORed();
    uint32_t ir_raw = bioSensor.getFIFOIR();
      
    fingerPresent = checkFingerPresence(ir_raw);
      
    // Filtering 
    int32_t ir_ac = IR_removeDC(ir_raw);
    int32_t ir_ac_filtered = IR_smoothSignal(ir_ac);
    int32_t red_ac = RED_removeDC(red_raw); 
    int32_t red_ac_filtered = RED_smoothSignal(red_ac);
    

    // The value ir_filtered corresponds to the current value
    if (abs(ir_ac_filtered) > abs(recentMax)){
      recentMax = ir_ac_filtered; 
    }

    // Adaptive threshold 
    ampEMA = (1.0f - ampAlpha) * ampEMA + ampAlpha * (float)abs(ir_ac_filtered);
    int32_t threshold_High = (int32_t)max(30.0f, 0.50f * ampEMA);
    int32_t threshold_Low = (int32_t)max(10.0f, 0.30f * ampEMA);

    // Peak detection 
    bool peakDetected = false;

    if (abs(ir_ac_filtered) < threshold_Low){
      armed = true;
    }
    // Detect if "prev" was a peak only if armed
    if (armed){
      if ((prev > prev_prev) && (prev > ir_ac_filtered) && (prev > threshold_High)){
        peakDetected = true;
        armed = false; // block until signal goes low again
        recentMax = prev;
      }
    }

    prev_prev = prev;  // This shifts samples for next iteration
    prev = ir_ac_filtered; 

    // BPM and SpO2 calculation 
    int BPM = 0;
    int SpO2 = 0; 
    if(fingerPresent){
      BPM = calculationBPM(peakDetected, fingerPresent);
      SpO2 = calculationSpO2(ir_raw, ir_ac_filtered, red_raw, red_ac_filtered, fingerPresent);
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
    //Serial.print(" IR_filtered:");
    //Serial.print(ir_ac_filtered);
    //Serial.print(peakDetected);
    Serial.print(" SpO2:"); 
    Serial.print(SpO2);
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

bool checkFingerPresence(uint32_t raw_signal){
  static bool fingerStable = false; 
  static bool fingerCandidate = false;
  static unsigned long fingerChange = 0; 
  const unsigned long fingerDebounce_ms = 250;

  bool candidate = fingerStable ? (raw_signal > 45000) : (raw_signal > 60000);
  // This value obtained from observing the values of raw_signal while the finger was pressed on the sensor
  if (candidate != fingerStable){
    if (candidate != fingerCandidate){
      fingerCandidate = candidate; 
      fingerChange = millis();
    }
    if (millis() - fingerChange >= fingerDebounce_ms){
      fingerStable = candidate; 
      resetFilters();
      startTime = millis();
    }
  }else{
    fingerCandidate = candidate;
  }
  return fingerStable;
}

int32_t IR_removeDC(uint32_t ir_raw){
  const int N = 100; // DC baseline window lenght, 1s window at 100 Hz
  static uint32_t buffer[N]; // Stores last N samples, circular buffer
  static uint64_t sum = 0;
  static int i = 0; 
  static int count = 0;
  
  if (resetFilterState){
    for (int a = 0; a < N; a++){
      buffer[a] = ir_raw;  // Buffer reset with first real sample
    }
    sum = (uint64_t)ir_raw* (uint64_t)N; 
    i = 0; 
    count = N;
  }

  sum -= buffer[i]; // This removes the oldest value from sum
  buffer[i] = ir_raw;
  sum += ir_raw; 
  i = (i+1) % N; // Update index in a circular way
  if (count < N) count++;
  uint32_t dc = sum/count;
  int32_t ir_ac = (int32_t)ir_raw - (int32_t)dc;
  return ir_ac;
}

int32_t IR_smoothSignal(int32_t ir_ac){
  const int M = 5; 
  static int32_t buffer[M];  // They must be signed
  static int64_t sum = 0;
  static int i = 0; 
  static int count = 0;

  if (resetFilterState){
    for (int a = 0; a < M; a++){
      buffer[a] = ir_ac; 
    }
    sum = (int64_t)ir_ac * (int64_t)M; 
    i = 0; 
    count = M;
  }
  
  sum -= buffer[i]; // This removes the oldest value from sum
  buffer[i] = ir_ac;
  sum += ir_ac; 
  i = (i+1) % M; // Update index in a circular way
  if (count < M) count++;
  int32_t ir_ac_filtered = sum/count;
  return ir_ac_filtered;
}

int32_t RED_removeDC(uint32_t red_raw){
  const int N = 100; // DC baseline window lenght, 1s window at 100 Hz
  static uint32_t buffer[N]; // Stores last N samples, circular buffer
  static uint64_t sum = 0;
  static int i = 0; 
  static int count = 0;
  
  if (resetFilterState){
    for (int a = 0; a < N; a++){
      buffer[a] = red_raw;  // Buffer reset with first real sample
    }
    sum = (uint64_t)red_raw* (uint64_t)N; 
    i = 0; 
    count = N;
  }

  sum -= buffer[i]; // This removes the oldest value from sum
  buffer[i] = red_raw;
  sum += red_raw; 
  i = (i+1) % N; // Update index in a circular way
  if (count < N) count++;
  uint32_t dc = sum/count;
  int32_t red_ac = (int32_t)red_raw - (int32_t)dc;
  return red_ac;
}
int32_t RED_smoothSignal(int32_t red_ac){
  const int M = 5; 
  static int32_t buffer[M];  // They must be signed
  static int64_t sum = 0;
  static int i = 0; 
  static int count = 0;

  if (resetFilterState){
    for (int a = 0; a < M; a++){
      buffer[a] = red_ac; 
    }
    sum = (int64_t)red_ac * (int64_t)M; 
    i = 0; 
    count = M;
    resetFilterState = false;
  }
  
  sum -= buffer[i]; // This removes the oldest value from sum
  buffer[i] = red_ac;
  sum += red_ac; 
  i = (i+1) % M; // Update index in a circular way
  if (count < M) count++;
  int32_t red_ac_filtered = sum/count;
  return red_ac_filtered;
}

int calculationBPM(bool peakDetected, bool fingerPresent){
  const int N = 5;  // 5 beats to obtain average
  const unsigned long setup_ms = 1500; 
  const unsigned long refract_ms = 300; 
  static unsigned long IBI_buffer[N] = {0};
  static int IBI_i = 0; 
  static int IBI_count = 0;
  unsigned long IBI_sum = 0;
  static unsigned long lastBeatTime = 0; 
  static int BPM = 0;
  unsigned long now = millis();
  static float BPM_ema = 0; 
  const float beta = 0.2f;
  static unsigned long lastGoodIBI = 0; 
  
  if(!fingerPresent){
    lastBeatTime = 0; 
    BPM = 0; 
    IBI_i = 0;
    IBI_count = 0;
    BPM_ema = 0;
    lastGoodIBI = 0;
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
    BPM_ema = 0;
    lastGoodIBI = 0;
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

  // This ignores outlier IBI values that occur because of missed or false peaks due to motion or noise.
  // It prevents pulling BPM down caused by a single bad interval
  if (lastGoodIBI != 0){
    // Applied an acceptance band that allows changes within +- 35% from one beat to the next
    if (IBI > (unsigned long)(1.35f * lastGoodIBI) || IBI < (unsigned long)(0.65f * lastGoodIBI)){
      return BPM;
    }
  }
  lastGoodIBI = IBI;

  IBI_buffer[IBI_i] = IBI; 
  IBI_i = (IBI_i +1) % N; 
  if(IBI_count < N) IBI_count++;

  // Necessary to calculate average IBI
  if(IBI_count >= 2){
    for(int i = 0; i < IBI_count; i++){
    IBI_sum += IBI_buffer[i];
    }
    unsigned long IBI_average = IBI_sum/(unsigned long)IBI_count;
    BPM = (int)(60000UL/IBI_average);
    if(BPM_ema == 0){
      BPM_ema = (float)BPM;
    }else{
      BPM_ema = (1.0f - beta) * BPM_ema + beta *(float)BPM;
    }
    BPM = (int)(BPM_ema + 0.5f);
  }
  
  return BPM;

}


int calculationSpO2(uint32_t ir_raw, int32_t ir_ac_filtered, uint32_t red_raw, int32_t red_ac_filtered, bool fingerPresent){
  const int N_dc = 100;  // DC window for 2s at 100 Hz
  const int N_ac = 100;  // AC amplitude window for 0.5s at 100 Hz

// Variables needed for obtaining IR_DC and RED_DC, this is the moving average of the moving raw signals
  static uint32_t ir_dc_buffer[N_dc];
  static uint32_t red_dc_buffer[N_dc];
  static uint64_t ir_dc_sum = 0;
  static uint64_t red_dc_sum = 0; 
  static int dc_i = 0; 
  static int dc_count = 0;

  // Variables needed for obtaining AC amplitude estimation
  static int ac_count = 0; 
  static int32_t ir_min = 0, ir_max = 0, red_min = 0, red_max = 0;

  static float SpO2_ema = 0.0f;
  static int lastSpO2 = 0;
  const float beta = 0.10f;

  if(!fingerPresent){
    ir_dc_sum = 0; 
    red_dc_sum = 0;
    dc_i = 0; dc_count = 0;
    ac_count = 0; 
    ir_min = ir_max = red_min = red_max = 0;
    SpO2_ema = 0.0f;
    lastSpO2 = 0;
    return 0;
  }

  // DC estimation 
  if (dc_count < N_dc){
    ir_dc_buffer[dc_i] = ir_raw;
    red_dc_buffer[dc_i] = red_raw;
    ir_dc_sum += ir_raw; 
    red_dc_sum += red_raw; 
    dc_count++;
  }else{
    ir_dc_sum -= ir_dc_buffer[dc_i];
    red_dc_sum -= red_dc_buffer[dc_i]; 
    ir_dc_buffer[dc_i] = ir_raw; 
    red_dc_buffer[dc_i] = red_raw;
    ir_dc_sum += ir_raw; 
    red_dc_sum += red_raw; 
  }
  dc_i = (dc_i + 1) % N_dc; 
  float IR_DC = (float) ir_dc_sum / (float)dc_count;
  float RED_DC = (float) red_dc_sum / (float)dc_count;

  // AC amplitude estimation, it analyses peak-to-peak of the filtered AC signal
  if(ac_count == 0){
    ir_min = ir_max = ir_ac_filtered;
    red_min = red_max = red_ac_filtered;
  }else{
    if(ir_ac_filtered < ir_min){ir_min = ir_ac_filtered;}
    if(ir_ac_filtered > ir_max){ir_max = ir_ac_filtered;}
    if(red_ac_filtered < red_min){red_min = red_ac_filtered;}
    if(red_ac_filtered > red_max){red_max = red_ac_filtered;}
  }
  ac_count++;

  if (ac_count < N_ac || dc_count < N_dc){
    return lastSpO2; // It waits until there is a full window completed
  }
  ac_count = 0; 

  float IR_AC = (float)(ir_max - ir_min);
  float RED_AC = (float)(red_max - red_min);
  
  // Value check
  if (IR_DC < 1.0f || RED_DC < 1.0f) return 0;
  if (IR_AC < 5.0f || RED_AC < 5.0f) return 0;
 
  // Calculates ratio of ratios 
  float R = ((RED_AC/RED_DC)/(IR_AC/IR_DC));
  float SpO2 = 110.0f - 25.0f *R;  // Generic empirical calibration

  // Smooth values of SpO2
  // EMA -> Exponential Moving Average 
  if (SpO2_ema == 0.0f){
    SpO2_ema = SpO2;
  }else{
    SpO2_ema = (1.0f - beta)* SpO2_ema + beta * SpO2; 
  }

  int SpO2_out = (int)(SpO2_ema + 0.5f); // For rounding 
  lastSpO2 = SpO2_out; // This value is updated once it is computed

  return  SpO2_out;

}




