#define BLYNK_TEMPLATE_ID "TMPL2PCqEU6B2"
#define BLYNK_TEMPLATE_NAME "SEP 783 Project3 Smart Blind Ventilation System"
#define BLYNK_AUTH_TOKEN "-jvOwqn5bAVoUcuyXgfOSoIvbf9BLp2R"

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <BlynkSimpleEsp32.h>
#include "Si7020.h"
#include <Arduino_LSM9DS1.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Wi-Fi Credentials (Hotspot)
char ssid[] = "example_ssid";
char pass[] = "12345678";  // Replace with actual hotspot password

const char* guardmodeURL = "http://172.20.10.5/guardMode";

IPAddress local_IP(123, 45, 67, 6);    // Sensor board static IP
IPAddress gateway(123, 45, 67, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(123, 45, 67, 1);

// Below are relevant IP address I used:
// const char* guardmodeURL = "http://172.20.10.5/guardMode";
// IPAddress local_IP(172, 20, 10, 6);    // Sensor board static IP
// IPAddress gateway(172, 20, 10, 1);
// IPAddress subnet(255, 255, 255, 0);
// IPAddress dns(172, 20, 10, 1);

// Blynk Timer
BlynkTimer timer;
AsyncWebServer server(80);
// Sensors
Si7020 tempSensor;
LSM9DS1Class imu(Wire);

// Sensor data
float lastTemperature = 0.0;
float lastAccelMagnitude = 0.0;
bool lastDangerState = false;
int guardModeValue = 0;
int alarmState = 0;
int previoustime = 0;

// --- Function: Send temperature every 15 sec ---
void sendTemperature() {
  float temp = tempSensor.getTemp();
  if (!isnan(temp)) {
    lastTemperature = temp;
    Blynk.virtualWrite(V2, lastTemperature);
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" °C");
  } else {
    Serial.println("Failed to read temperature.");
  }
}

// --- Function: Check acceleration every 300ms ---
void checkForDangerousAcceleration() {
  float ax, ay, az;
  if (imu.readAcceleration(ax, ay, az)) {
    float magnitude = sqrt(ax * ax + ay * ay + az * az);
    lastAccelMagnitude = magnitude;
    
    Serial.print("Accel Magnitude: ");
    Serial.println(magnitude, 3);
    if (guardModeValue == 1){ 
      if (magnitude > 1.1 && !lastDangerState) {
      Serial.println(" DANGEROUS MOTION DETECTED!");
      Blynk.virtualWrite(V5, 1);  // Send alert ON
      
      lastDangerState = true;
      previoustime = millis();
      alarmState = 1;

     } else if (magnitude < 1.1 && alarmState == 1){
     
      int currenttime = millis();
      if (currenttime - previoustime > 10000){
        Blynk.virtualWrite(V5, 0); 
        lastDangerState = false;
        previoustime = currenttime;
        alarmState = 0;
      }
     }
      
   }
  } else {
    Serial.println("Failed to read acceleration.");
  }
}

BLYNK_CONNECTED() {
  Serial.println("Connected to Blynk server");
  
}
void setup() {
  Serial.begin(9600);

 

  tempSensor.begin();

  if (!imu.begin()) {
    Serial.println("IMU failed.");
    while (1);
  }
  WiFi.config(local_IP, gateway, subnet, dns);
  WiFi.begin(ssid, pass);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Hotspot.");
  Serial.print("Sensor Board IP: ");
  Serial.println(WiFi.localIP());

  // Set up HTTP endpoint for /outdoorTemp
  server.on("/outdoorTemp", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(lastTemperature));
  });
  server.begin();
  Serial.println("Sensor Board HTTP server started.");

  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();

  // Timer setup if needed
  timer.setInterval(15000L, sendTemperature);
  timer.setInterval(300L, checkForDangerousAcceleration);
}

void checkBlynkConnection() {
  static unsigned long lastReconnectAttempt = 0;
  static bool wasConnected = false;
  
  if (Blynk.connected()) {
    if (!wasConnected) {
      Serial.println("Connected to Blynk!");
      wasConnected = true;
    }
  } else {
    if (wasConnected) {
      Serial.println("Lost connection to Blynk!");
      wasConnected = false;
    }
    
    unsigned long currentTime = millis();
    if (currentTime - lastReconnectAttempt > 10000) { // Try every 10 seconds
      lastReconnectAttempt = currentTime;
      Serial.println("Attempting to reconnect...");
      Blynk.connect();
    }
  }
}

void loop() {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
     
      http.begin(guardmodeURL);
      int httpCode = http.GET();
     
      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        guardModeValue = payload.toInt();
      }
    }
  checkBlynkConnection();
  Blynk.run();
  timer.run();
}

