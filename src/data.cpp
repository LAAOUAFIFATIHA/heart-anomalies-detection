#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#include "config.h"
#include <U8x8lib.h> 




// ECG Configuration
// const int ECG_PIN = 34;
const int SAMPLING_RATE = 100;  // Higher rate needed for ECG analysis
int BASELINE = 2048;

// Sensor
#define DPIN 4        
#define DTYPE DHT11   
Adafruit_MPU6050 mpu;

DHT dht(DPIN, DTYPE);

// ECG Analysis Parameters
const int BUFFER_SIZE = 500;  // 2 seconds of data at 250Hz
int ecgBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// other sensors
float prevAccelX = 0, prevAccelY = 0, prevAccelZ = 0;
unsigned long prevTime = 0;  
const unsigned long interval = 1000;  // 10 seconds ms

// identify if the person in notion
int notion(float first_ac_x, float secnde_ac_x, float first_ac_y, float secnde_ac_y, float first_ac_z, float secnde_ac_z) {
  const float threshold = 0.1;  
  if (abs(first_ac_x - secnde_ac_x) > threshold || 
      abs(first_ac_y - secnde_ac_y) > threshold || 
      abs(first_ac_z - secnde_ac_z) > threshold) {
    return 1; 
  } else {
    return 0;  
  }
}

// Peak Detection
int dynamicThreshold = 100;  // Minimum height for R-peak
const int MIN_RR_INTERVAL = 50;   // Minimum samples between R-peaks (200ms at 250Hz)
const int MAX_RR_INTERVAL = 500;  // Maximum samples between R-peaks (2s at 250Hz)

// Detected Peaks Storage
const int MAX_PEAKS = 10;
struct Peak {
  int position;
  int amplitude;
  unsigned long timestamp;
};

Peak rPeaks[MAX_PEAKS];
Peak pPeaks[MAX_PEAKS];
Peak tPeaks[MAX_PEAKS];
int rPeakCount = 3;
int pPeakCount = 3;
int tPeakCount = 3;
int motionDetected = 0;
float tc = 0;
float hu =1;


// Heart Rate Parameters
// float heartRate = NAN;
// float rrInterval = NAN;
// float ppInterval = NAN;
// float qtInterval = NAN;
// float qrsInterval = NAN;
// float prInterval = NAN;


float heartRate = 80;
float rrInterval = 800;
float ppInterval = 900;
float qtInterval = 424;
float qrsInterval = 83;
float prInterval = 200;



// float heartRate = 20;
// float rrInterval = 800;
// float ppInterval = 900;
// float qtInterval = 34;
// float qrsInterval = 83;
// float prInterval = 60;

int rawValue = 0;
const int NUM_SENSORS = 10;
float sensorReadings[NUM_SENSORS];

// WiFiClient espClient;
// PubSubClient client(espClient);

// Function declarations
void connectToWiFi();
void reconnect();
void calibrateBaseline();
void addToBuffer(int value);
void analyzeECG();
void detectRPeaks();
void detectPTWaves();
void calculateIntervals();
void sendECGParameters( float tc , float hu , int notion , int rawValue);
bool isValidECGSignal(int rawValue);
int movingAverage(int newValue);
int applyLowPassFilter(int value);
float* getSensorData();


// U8X8_ST7565_ERC12864_4W_SW_SPI u8x8(/* clock=*/18, /* data=*/23, /* cs=*/5, /* reset=*/19 , 21);


void setup_1() {
  u8x8.clearDisplay();
  u8x8.drawString(0, 0, "Starting ECG...");

  Serial.begin(9600);
  dht.begin();
  while (!Serial) delay(10); 

  Serial.println("Initializing MPU6050...");
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050!");
    while (1) delay(10);
  }
  
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);       
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    

  // Initialize previous acceleration with current values
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  prevAccelX = a.acceleration.x;
  prevAccelY = a.acceleration.y;
  prevAccelZ = a.acceleration.z;
  prevTime = millis();


  // ECG 
  pinMode(ECG_PIN, INPUT);
  analogReadResolution(12);
  
  connectToWiFi();
  client.setServer(mqtt_server, mqtt_port);
  
  calibrateBaseline();
  
  // Initialize buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    ecgBuffer[i] = 0;
  }
  
  Serial.println("ECG Analysis System Started");
  client.publish(mqtt_topic_status, "ECG Analysis System Started");
}

void loop_1() {



  unsigned long currentTime = millis();
  
  // Check if 10 seconds have passed
  if (currentTime - prevTime >= interval) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    motionDetected = notion(prevAccelX, a.acceleration.x, prevAccelY, a.acceleration.y, prevAccelZ, a.acceleration.z);
    // Serial.print("Motion Detected: ");
    // Serial.println(motionDetected);


    // Update previous values
    prevAccelX = a.acceleration.x;
    prevAccelY = a.acceleration.y;
    prevAccelZ = a.acceleration.z;
    prevTime = currentTime;  // Reset timer
  }

  // / / /  ---------------------  SENSOR 2 -----------------------
  
  tc = dht.readTemperature(false);  // Read temperature in C
  float tf = dht.readTemperature(true);   // Read Temperature in F
  hu = dht.readHumidity();          // Read Humidity

  // Serial.print("Temp: ");
  // Serial.print(tc);
  // Serial.print(" C, ");
  // Serial.print(tf);
  // Serial.print(" F, Hum: ");
  // Serial.print(hu);
  // Serial.println("%");

  //  ----------------------  sensor 3 ------------------- (-______-)
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static unsigned long lastSample = 0;
  
  if (millis() - lastSample >= 1000/SAMPLING_RATE) {
    lastSample = millis();
    
    rawValue = analogRead(ECG_PIN);
    
    if (isValidECGSignal(rawValue)) {
      int processedValue = rawValue - BASELINE;
      
      // Apply simple filtering
      int filteredValue = applyLowPassFilter(processedValue);
      
      // Add to circular buffer
      addToBuffer(filteredValue);
      
      // Analyze ECG when buffer is full
      if (bufferFull) {
        analyzeECG();
      }
    }
  }
  
  // Send parameters every 5 seconds
  static unsigned long lastParameterSend = 0;
  if (millis() - lastParameterSend >= 1000) {
    sendECGParameters(tc, hu, motionDetected ,rawValue );
    lastParameterSend = millis();
  }
}


void addToBuffer(int value) {
  ecgBuffer[bufferIndex] = value;
  bufferIndex++;
  
  if (bufferIndex >= BUFFER_SIZE) {
    bufferIndex = 0;
    bufferFull = true;
  }
}

void analyzeECG() {
  detectRPeaks();
  detectPTWaves();
  calculateIntervals();
}

void detectRPeaks() {
    int newPeakCount = 0;
    Peak newPeaks[MAX_PEAKS];

    for (int i = 2; i < BUFFER_SIZE - 2; i++) {
        int current = ecgBuffer[i];
        if (current > dynamicThreshold && 
            current > ecgBuffer[i-1] && 
            current > ecgBuffer[i+1] &&
            current > ecgBuffer[i-2] && // Require wider peak
            current > ecgBuffer[i+2]) {
            
            if (newPeakCount == 0 || 
                (i - newPeaks[newPeakCount-1].position) > MIN_RR_INTERVAL) {
                
                if (newPeakCount < MAX_PEAKS) {
                    newPeaks[newPeakCount++] = {i, current, millis()};
                }
            }
        }
    }

    //Update threshold based on detected peaks
    if (newPeakCount > 0) {
        dynamicThreshold = 0.6 * newPeaks[newPeakCount-1].amplitude;
    }
  
    // Copy to main array
    memcpy(rPeaks, newPeaks, sizeof(Peak)*newPeakCount);
    rPeakCount = newPeakCount;
}

void detectPTWaves() {
  pPeakCount = 0;
  tPeakCount = 0;
  
  for (int r = 0; r < rPeakCount; r++) {
    int rPos = rPeaks[r].position;
    
    // Look for P-wave before R-peak 
    int pSearchStart = rPos - 80;  // Look 80 samples before R
    int pSearchEnd = rPos - 20;    // Up to 20 samples before R
    
    if (pSearchStart < 0) pSearchStart += BUFFER_SIZE;
    if (pSearchEnd < 0) pSearchEnd += BUFFER_SIZE;
    
    int maxP = -1000;
    int pPos = -1;
    
    for (int i = pSearchStart; i != pSearchEnd; i = (i + 1) % BUFFER_SIZE) {
      if (ecgBuffer[i] > maxP && ecgBuffer[i] > 70) {
        maxP = ecgBuffer[i];
        pPos = i;
      }
    }
    
    if (pPos != -1 && pPeakCount < MAX_PEAKS) {
      pPeaks[pPeakCount].position = pPos;
      pPeaks[pPeakCount].amplitude = maxP;
      pPeaks[pPeakCount].timestamp = millis();
      pPeakCount++;
    }
    
    // Look for T-wave (after R-peak)
    int tSearchStart = rPos + 40;  // Look 40 samples after R
    int tSearchEnd = rPos + 120;   // Up to 120 samples after R
    
    if (tSearchStart >= BUFFER_SIZE) tSearchStart -= BUFFER_SIZE;
    if (tSearchEnd >= BUFFER_SIZE) tSearchEnd -= BUFFER_SIZE;
    
    int maxT = -1000;
    int tPos = -1;
    
    for (int i = tSearchStart; i != tSearchEnd; i = (i + 1) % BUFFER_SIZE) {
      if (ecgBuffer[i] > maxT && ecgBuffer[i] > 30) {
        maxT = ecgBuffer[i];
        tPos = i;
      }
    }
    
    if (tPos != -1 && tPeakCount < MAX_PEAKS) {
      tPeaks[tPeakCount].position = tPos;
      tPeaks[tPeakCount].amplitude = maxT;
      tPeaks[tPeakCount].timestamp = millis();
      tPeakCount++;
    }
  }
}

void calculateIntervals() {
  if (rPeakCount >= 2) {
    // Calculate RR interval (time between R-peaks)
    int rrSamples = rPeaks[rPeakCount-1].position - rPeaks[rPeakCount-2].position;
    if (rrSamples < 0) rrSamples += BUFFER_SIZE;
    
    rrInterval = (float)rrSamples / SAMPLING_RATE * 1000; // Convert to milliseconds
    heartRate = 60000.0 / rrInterval; // BPM
    
    // Calculate PP interval if P-waves detected
    if (pPeakCount >= 2) {
      int ppSamples = pPeaks[pPeakCount-1].position - pPeaks[pPeakCount-2].position;
      if (ppSamples < 0) ppSamples += BUFFER_SIZE;
      ppInterval = (float)ppSamples / SAMPLING_RATE * 1000;
    }
    
    // Calculate QT interval (R to T wave)
    if (tPeakCount > 0 && rPeakCount > 0) {
      int qtSamples = tPeaks[tPeakCount-1].position - rPeaks[rPeakCount-1].position;
      if (qtSamples < 0) qtSamples += BUFFER_SIZE;
      qtInterval = (float)qtSamples / SAMPLING_RATE * 1000;
    }
    
    // Calculate QRS duration (simplified - width of R-peak)
    // qrsInterval = 80; // Simplified: assume ~80ms QRS duration


    // Calculate QRS duration - FIXED VERSION
    if (rPeakCount > 0) {
      int rPos = rPeaks[rPeakCount-1].position;
      int qPos = rPos;
      int sPos = rPos;
      
      // Find Q-point
      while (qPos > 0 && ecgBuffer[qPos] < ecgBuffer[qPos - 1]) {
        qPos--;
      }
      
      // Find S-point
      while (sPos < BUFFER_SIZE - 1 && ecgBuffer[sPos] < ecgBuffer[sPos + 1]) {
        sPos++;
      }
      
      qrsInterval = (float)(sPos - qPos) / SAMPLING_RATE * 1000;
    }
}
    
    // Calculate PR interval (P to R wave)
    if (pPeakCount > 0 && rPeakCount > 0) {
      int prSamples = rPeaks[rPeakCount-1].position - pPeaks[pPeakCount-1].position;
      if (prSamples < 0) prSamples += BUFFER_SIZE;
      prInterval = (float)prSamples / SAMPLING_RATE * 1000;
    }
    if (heartRate < 30 || heartRate > 250) heartRate = NAN;
  }
  

void sendECGParameters( float tc , float hu , int notion , int filteredValue) {
  // Send individual parameters
  client.publish("fatiha/ecg/heart_rate", String(heartRate, 1).c_str());   
  client.publish("fatiha/ecg/rr_interval", String(rrInterval, 1).c_str());
  client.publish("fatiha/ecg/pp_interval", String(ppInterval, 1).c_str());
  client.publish("fatiha/ecg/qt_interval", String(qtInterval, 1).c_str());
  client.publish("fatiha/ecg/qrs_interval", String(qrsInterval, 1).c_str());
  client.publish("fatiha/ecg/pr_interval", String(prInterval, 1).c_str());
  client.publish("fatiha/ecg/raw", String(rawValue).c_str());
  client.publish("fatiha/ecg/notion", String(notion).c_str());
  client.publish("fatiha/ecg/temp", String(tc, 1).c_str());
  client.publish("fatiha/ecg/hum", String(hu, 1).c_str());

}

float* getSensorData() {
  // Assign all sensor values to the array
  sensorReadings[0] = heartRate;            // Temperature
  sensorReadings[1] = rrInterval;       // Humidity
  sensorReadings[2] = ppInterval ;    // Motion
  sensorReadings[3] = qtInterval  ; // Heart rate (BPM)
  sensorReadings[4] = qrsInterval ; // RR interval (ms)
  sensorReadings[5] = prInterval ; // PP interval (ms)
  sensorReadings[6] = hu;  // QT interval (ms)
  sensorReadings[7] = tc; // QRS duration (ms)
  sensorReadings[8] = motionDetected;  // PR interval (ms)
  
  // Additional numeric values (customize as needed)
  sensorReadings[9] = rawValue;
  
  return sensorReadings; // Returns pointer to the float array
}


// Simple low-pass filter to reduce noise
int applyLowPassFilter(int value) {
  static int prevValue = 0;
  int filteredValue = (value + prevValue) / 2;
  prevValue = value;
  return filteredValue;
}

bool isValidECGSignal(int rawValue) {
    static int lastValid = 2048;
    bool valid = (abs(rawValue - lastValid) < 1000) && (rawValue > 50 && rawValue < 4000); // fast change 
    if (valid) lastValid = rawValue;
    return valid;
}


void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    // delay(50);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32ECGAnalyzer", mqtt_user, mqtt_pass)) {
      Serial.println("Connected!");
      client.publish(mqtt_topic_status, "ECG Analyzer Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5s...");
      // delay(50);
    }
  }
}

void calibrateBaseline() {
  Serial.println("Calibrating baseline...");
  client.publish(mqtt_topic_status, "Calibrating baseline...");
  
  long sum = 0;
  const int samples = 500;
  
  for (int i = 0; i < samples; i++) {
    sum += analogRead(ECG_PIN);
    // delay(2);
  }
  
  BASELINE = sum / samples;
  Serial.print("Baseline calibrated: ");
  Serial.println(BASELINE);
  
  String baselineMsg = "Baseline: " + String(BASELINE);
  client.publish(mqtt_topic_status, baselineMsg.c_str());
}