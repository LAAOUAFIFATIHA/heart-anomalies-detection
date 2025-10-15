// config.h
#pragma once
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "NeuralNetwork.h"


// WiFi Configuration
extern const char* ssid;
extern const char* password;

// MQTT Configuration
extern const char* mqtt_server;
extern const int mqtt_port;
extern const char* mqtt_user;
extern const char* mqtt_pass;
extern const char* mqtt_topic_status;

// Heart Rate Parameters
extern float heartRate ;
extern float rrInterval;
extern float ppInterval ;
extern float qtInterval ;
extern float qrsInterval;
extern float prInterval;
extern int rawValue ;
extern const int ECG_PIN;


// Add this if not already present
extern NeuralNetwork nn;
extern const float scaler_mean[10];
extern const float scaler_dev[10];


// Extern declarations (no memory allocation)
extern const char* mqtt_server;
extern WiFiClient espClient;
extern PubSubClient client;

// Function declarations
void sendECGParameters(float, float, int, int);
float*  getSensorData();
void setup_1();
void loop_1();


#include <U8x8lib.h>

extern U8X8_ST7565_ERC12864_4W_SW_SPI u8x8; 





