#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL2koX3T71B"
#define BLYNK_TEMPLATE_NAME "Project 3 IoT"
#define BLYNK_AUTH_TOKEN "Skl-GKSJhCPU0c5PhWyWXcKwj7lIMh14"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <MCP23017.h>

// WiFi credentials for Blynk connection
char ssid[] = "McMaster Graduate";
char pass[] = "10BayMcMaster";

// Sensor Board WiFi credentials
const char* sensorBoardSSID = "ESP32-SensorBoard";
const char* sensorBoardPassword = "sensor123";
const char* sensorBoardIP = "192.168.4.1"; // Default AP IP address
const char* sensorDataURL = "http://192.168.4.1/sensors";

// MCP23017 for LED control
MCP23017 mcp(0x20);

// Timer for sensor data fetch and upload
BlynkTimer timer;

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

// Pin definitions for MCP23017
#define ALARM_LED_PIN 0      // GPA0 for Window Alarm LED
#define ALARM_BUZZER_PIN 1   // GPA1 for Window Alarm buzzer

// Sensor data variables
float temperature = 0.0;
float humidity = 0.0;
float lightIntensity = 0.0;
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;

// Alarm state
bool alarmEnabled = false;
bool alarmTriggered = false;

// Window Alarm threshold
const float ACCEL_THRESHOLD = 0.1; // 0.1g threshold for window alarm

// Function prototypes
void setupMCP();
void fetchSensorData();
void checkAlarmCondition();
void updateBlynk();
void toggleAlarm(bool enabled);
void activateAlarm(bool activate);

// Blynk virtual pin handler for Window Alarm Switch
BLYNK_WRITE(V0) {
  alarmEnabled = param.asInt();
  Serial.print("Window Alarm Switch: ");
  Serial.println(alarmEnabled ? "ON" : "OFF");
  
  // Reset alarm if turned off
  if (!alarmEnabled) {
    activateAlarm(false);
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("Initializing Mac IoT Board...");
  
  // Initialize MCP23017
  setupMCP();
  
  // Connect to WiFi for Blynk
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.print("Connected to Mobile Hotspot, IP: ");
  Serial.println(WiFi.localIP());
  
  // Initialize Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  // Set up timer for periodic tasks
  timer.setInterval(5000L, fetchSensorData);    // Fetch sensor data every 5 seconds
  timer.setInterval(1000L, checkAlarmCondition); // Check alarm conditions every second
  timer.setInterval(5000L, updateBlynk);        // Update Blynk every 5 seconds
  
  Serial.println("Mac IoT Board initialization complete!");
}

void loop() {
  Blynk.run();
  timer.run();
}

void setupMCP() {
  mcp.init();
  
  // Set up LED and buzzer pins as outputs
  mcp.pinMode(ALARM_LED_PIN, OUTPUT);
  mcp.pinMode(ALARM_BUZZER_PIN, OUTPUT);
  
  // Initially turn off LED and buzzer
  mcp.digitalWrite(ALARM_LED_PIN, LOW);
  mcp.digitalWrite(ALARM_BUZZER_PIN, LOW);
  
  Serial.println("MCP23017 initialized");
}

void fetchSensorData() {
  if (WiFi.status() == WL_CONNECTED) {
    // Create a second WiFi client to connect to the sensor board
    WiFiClient client;
    HTTPClient http;
    
    // Fetch sensor data
    http.begin(client, sensorDataURL);
    int httpCode = http.GET();
    
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      
      // Parse JSON response (simplified parsing for demonstration)
      // In a real application, use ArduinoJson library for robust parsing
      int tempIndex = payload.indexOf("\"temperature\":");
      int humIndex = payload.indexOf("\"humidity\":");
      int lightIndex = payload.indexOf("\"light\":");
      int accelXIndex = payload.indexOf("\"accelX\":");
      int accelYIndex = payload.indexOf("\"accelY\":");
      int accelZIndex = payload.indexOf("\"accelZ\":");
      
      if (tempIndex >= 0) {
        temperature = payload.substring(tempIndex + 14, payload.indexOf(",", tempIndex)).toFloat();
      }
      
      if (humIndex >= 0) {
        humidity = payload.substring(humIndex + 11, payload.indexOf(",", humIndex)).toFloat();
      }
      
      if (lightIndex >= 0) {
        lightIntensity = payload.substring(lightIndex + 8, payload.indexOf(",", lightIndex)).toFloat();
      }
      
      if (accelXIndex >= 0) {
        accelX = payload.substring(accelXIndex + 9, payload.indexOf(",", accelXIndex)).toFloat();
      }
      
      if (accelYIndex >= 0) {
        accelY = payload.substring(accelYIndex + 9, payload.indexOf(",", accelYIndex)).toFloat();
      }
      
      if (accelZIndex >= 0) {
        accelZ = payload.substring(accelZIndex + 9, payload.indexOf("}", accelZIndex)).toFloat();
      }
      
      Serial.print("Sensor data - Temp: ");
      Serial.print(temperature);
      Serial.print("°C, Humidity: ");
      Serial.print(humidity);
      Serial.print("%, Light: ");
      Serial.print(lightIntensity);
      Serial.print(" lux, Accel: [");
      Serial.print(accelX);
      Serial.print(", ");
      Serial.print(accelY);
      Serial.print(", ");
      Serial.print(accelZ);
      Serial.println("]");
    } else {
      Serial.print("Error fetching sensor data, HTTP code: ");
      Serial.println(httpCode);
    }
    
    http.end();
  }
}

void checkAlarmCondition() {
  if (alarmEnabled) {
    // Check if acceleration exceeds threshold in any direction
    float accelMagnitude = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
    bool abnormalMovement = (abs(accelX) > ACCEL_THRESHOLD || 
                            abs(accelY) > ACCEL_THRESHOLD || 
                            abs(accelZ) > ACCEL_THRESHOLD);
    
    if (abnormalMovement) {
      Serial.println("Abnormal movement detected! Activating alarm.");
      activateAlarm(true);
    }
  }
}

void updateBlynk() {
  // Update Blynk virtual pins with sensor data
  Blynk.virtualWrite(V1, temperature);     // Temperature gauge
  Blynk.virtualWrite(V2, humidity);        // Humidity gauge
  Blynk.virtualWrite(V3, lightIntensity);  // Light intensity gauge
  
  // Update alarm status on Blynk
  Blynk.virtualWrite(V4, alarmTriggered);  // Window Alarm LED widget
}

void activateAlarm(bool activate) {
  alarmTriggered = activate;
  
  if (activate) {
    // Start alarm (blinking LED and buzzer)
    static bool ledState = false;
    ledState = !ledState;
    mcp.digitalWrite(ALARM_LED_PIN, ledState);
    mcp.digitalWrite(ALARM_BUZZER_PIN, ledState);
  } else {
    // Turn off alarm
    mcp.digitalWrite(ALARM_LED_PIN, LOW);
    mcp.digitalWrite(ALARM_BUZZER_PIN, LOW);
  }
}