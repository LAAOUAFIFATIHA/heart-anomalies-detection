#include <Arduino.h>
#include <Wire.h>
#include "NeuralNetwork.h"
#include "ECG_model.h" 
#include "config.h"
#include <U8x8lib.h> 

// WiFi Configuration
const char* ssid = "HUAWEI-B311-5D77";
const char* password = "J5L49HMJLGB";

// MQTT Configuration
const char* mqtt_server = "192.168.8.102";
const int mqtt_port = 1883;
const char* mqtt_user = "fatiha";
const char* mqtt_pass = "fatiha123";
const char* mqtt_topic_status = "fatiha/ecg/status";

// Client objects
WiFiClient espClient;
PubSubClient client(espClient);

const int ECG_PIN = 34;

// init model
NeuralNetwork nn; 
// // scaler values from the training 
const float scaler_mean[10] = {108.46122449 , 836.29931973 , 790.92414966  ,391.21376179,   97.34693878,
  439.57823129,   70.10136054,   28.81836735,    7.68707483, 2090.59183673 };
const float scaler_dev[10] = {69.48502746 , 871.38026732 , 714.46192877 , 27.16545164 , 99.41794231,
  182.16938836  , 14.08084039  ,  3.12553713  , 82.7873107 , 1061.43930492};

const int ledPin = 2;

U8X8_ST7565_ERC12864_4W_SW_SPI u8x8(/* clock=*/18, /* data=*/23, /* cs=*/5, /* reset=*/19 , 21);

uint8_t heart_bitmap[] U8X8_PROGMEM = {
  0b00000000,  // ........
  0b00000000,  // ........
  0b00011000,  // ...##... 
  0b00111100,  // ..####..
  0b01111110,  // .######.
  0b11111111,  // ########
  0b11111111,  // ########
  0b01100110   // .##..##.
};

void initializeDisplay() {
  u8x8.begin();
  // delay(200);                 
  u8x8.setPowerSave(0);       
  // delay(50);
  u8x8.setContrast(30);       
  u8x8.clearDisplay();
  u8x8.setFont(u8x8_font_5x8_r);
  
  // Initial persistent message
  u8x8.drawString(1, 1, "Heart Health");
  u8x8.drawTile(13, 1, 1, heart_bitmap);    

  // u8x8.drawString(1, 4, "AID MOUB");
}

void refreshDisplay(const char* res) {
  initializeDisplay();
  u8x8.clearDisplay();
  u8x8.drawString(1, 1, "Heart Health");  
  u8x8.drawTile(13, 1, 1, heart_bitmap);    
  // u8x8.drawString(1, 4, "AID MOUBARAK HAHAHA");  
  u8x8.drawString(1, 3, ("Raw value: " + String(rawValue)).c_str());
 

  u8x8.drawString(1, 6, res);
}

void setup() {
  initializeDisplay();       
  
  setup_1();                 
  
  // Refresh display after all initializations
  refreshDisplay("First");
}

void loop() {


    // Serial.println("helllllllllllllllllllllllllllllllllllllllllllllllllo");

   loop_1();


  float prediction = -1.00;
  float* data = getSensorData();
  // Input data matching your Python test case
  float data_to_test[10] = { data[0],	data[1],	data[2],	data[3],
                          	data[4],	data[5],	data[6],	data[7],	
                            data[8],data[9] };

  // Get input buffer pointer
  float* input = nn.getInputBuffer();

  // Normalize the data
  for (int i = 0; i < 10; i++) {
    input[i] = (data_to_test[i] - scaler_mean[i]) / scaler_dev[i];
  }

  // Make prediction
  if(!isnan(data_to_test[0]) && !isnan(data_to_test[1])){



  prediction = nn.predict();

  // Print results
  // Serial.print("Raw prediction value: ");
  // Serial.println(prediction, 10);


  // Serial.print("Raw value: ");
  // Serial.println(rawValue);
  
  
  
  //  bla bla 
  // client.publish("fatiha/ecg/resLocal", String(prediction).c_str());
  if (prediction >= 0.70) {
    client.publish("fatiha/ecg/resLocal", String(1).c_str());
    Serial.println("Prediction: anomaly detected");
    refreshDisplay("anomaly detected");  
    // delay(2000);
    digitalWrite(ledPin, HIGH); 
    Serial.println("Led ON");
  } else {
    client.publish("fatiha/ecg/resLocal", String(0).c_str());
    Serial.println("Prediction: NO anomaly");
    refreshDisplay("NO anomaly"); 
    // delay(2000); 
    digitalWrite(ledPin, LOW); 
    Serial.println("Led OF");
  } 
}

}