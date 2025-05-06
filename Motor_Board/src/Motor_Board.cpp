// This code without PID is validated on an actual ESP32 board.
// This code is developed using a ESP32 Developing Board provided by McMaster University.

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESP32Encoder.h>

// WiFi credentials for Sensor Board connection
const char* ssid = "example_ssid";
const char* password = "12345678";
const char* MacIoTBoardIP = "123.45.67.5"; // Change your IP address here
// You also need to modify the IP in the following URLs
// make sure they are identical to the IP address in the Mac IoT Main Code.cpp
// const char* indoorTempURL = "http://123.45.67.5/indoorTemp";
const char* lightSensorURL = "http://123.45.67.5/brightness";
const char* ventilationURL = "http://123.45.67.5/manualMode";

// Below are my IP settings, I'll list them here for your reference
// const char* MacIoTBoardIP = "172.20.10.5"; // Default AP IP address
// // const char* indoorTempURL = "http://172.20.10.5/indoorTemp";
// const char* lightSensorURL = "http://172.20.10.5/brightness";
// const char* ventilationURL = "http://172.20.10.5/manualMode";

// Motor control pins
#define EN_PWM 32      // PWM control for motor speed
#define DIR 33         // Motor direction control
#define SLEEP 16       // Used to idle the motor
#define PMODE 27       // Drive mode for motor controller

// PWM settings
#define PWM_FREQ 10000 // PWM frequency
#define PWM_CHANNEL 0  // PWM channel
#define PWM_RES 8      // 8-bit resolution

// Encoder setup
ESP32Encoder encoder;
#define ENCODER_A_PIN 25
#define ENCODER_B_PIN 26
#define COUNTS_PER_REVOLUTION 533

// Light intensity thresholds
#define LIGHT_THRESHOLD_MAX 700
#define LIGHT_THRESHOLD_HIGH 500
#define LIGHT_THRESHOLD_MED 350
#define LIGHT_THRESHOLD_LOW 150

// Motor position control
long currentPosition = 0;  // Current position in encoder counts
long targetPosition = 0;   // Target position in encoder counts
int motorSpeed = 35;      // Default motor speed (0-255)

// Function prototypes
void setupMotor();
void setupEncoder();
void moveToPosition(long targetCounts);
void fetchLightIntensity();
long calculateTargetPosition(float lightIntensity);

void setup() {
  Serial.begin(9600);
  
  Serial.println("Initializing Motor Board...");
  
  // Set up motor control
  setupMotor();
  
  // Set up encoder
  setupEncoder();
  
  // Connect to Sensor Board WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Mac IoT Board");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.print("Connected to Sensor Board, IP: ");
  Serial.println(WiFi.localIP());
  
  // Reset encoder position
  encoder.clearCount();
  currentPosition = 0;
  
  Serial.println("Motor Board initialization complete!");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    
    http.begin(ventilationURL);
    int httpCode = http.GET();
    
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      int manualModeValue = payload.toInt();

      if (manualModeValue == 0) {
        http.end();  // Finish the first Request
        // Fetch light intensity from sensor board
        fetchLightIntensity();
        // Read the current position continually
        currentPosition = encoder.getCount();
  
        // Calculate the error between the target position and current position
        long error = targetPosition - currentPosition;
  
        if (abs(error) < 150) {
          // Close enough then stop the motor
          ledcWrite(PWM_CHANNEL, 0);
        } else {
          // Set the rotation oritation
          digitalWrite(DIR, error > 0 ? LOW : HIGH);
          ledcWrite(PWM_CHANNEL, motorSpeed);
        }
  
        // Print the current position
        Serial.print("Current position: ");
        Serial.print(currentPosition);
        Serial.print(", Target: ");
        Serial.println(targetPosition);
  
        delay(200);  // Small delay for stability
      }
      else if (manualModeValue >= 1 && manualModeValue <= 3) {
        switch(manualModeValue) {
          case 1: motorSpeed = 35; break;
          case 2: motorSpeed = 70; break;
          case 3: motorSpeed = 105; break;
        }
        digitalWrite(DIR, LOW);
        ledcWrite(PWM_CHANNEL, motorSpeed);
        Serial.printf("\rVentilation Scale: %3d PWM", motorSpeed);
      }
    }
  }
}

void setupMotor() {
  // Configure motor control pins
  pinMode(SLEEP, OUTPUT);
  pinMode(PMODE, OUTPUT);
  digitalWrite(SLEEP, HIGH);  // Enable motor driver
  digitalWrite(PMODE, LOW);   // Set drive mode
  
  // Set up PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(EN_PWM, PWM_CHANNEL);
  pinMode(DIR, OUTPUT);
  digitalWrite(DIR, HIGH);  // Set default direction
  
  // Initially stop motor
  ledcWrite(PWM_CHANNEL, 0);
  
  Serial.println("Motor control initialized");
}

void setupEncoder() {
  // Initialize encoder
  encoder.attachHalfQuad(ENCODER_A_PIN, ENCODER_B_PIN);
  encoder.setCount(0);
  
  Serial.println("Encoder initialized");
}

void fetchLightIntensity() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    
    http.begin(lightSensorURL);
    int httpCode = http.GET();
    
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      float lightIntensity = payload.toFloat();
      
      Serial.print("Light intensity: ");
      Serial.print(lightIntensity);
      Serial.println(" lux ");
      // Serial.printf("\rLight intensity: %.2f lux", lightIntensity);
      
      // Calculate target position based on light intensity
      long newTargetPosition = calculateTargetPosition(lightIntensity);
      
      // Move motor to new position if it's different from current target
      if (newTargetPosition != targetPosition) {
        targetPosition = newTargetPosition;
      }
    }
    else {
      Serial.print("Error fetching light data, HTTP code: ");
      Serial.println(httpCode);
    }
    http.end();
  }
}

long calculateTargetPosition(float lightIntensity) {
  // Calculate target position in encoder counts based on light intensity
  // Each full rotation is COUNTS_PER_REVOLUTION encoder counts
  
  long targetRevolutions = 0;
  
  if (lightIntensity > LIGHT_THRESHOLD_MAX) {
    // Maximum light - fully closed blinds (10 revolutions)
    targetRevolutions = 10;
  } else if (lightIntensity > LIGHT_THRESHOLD_HIGH) {
    // High light - mostly closed blinds (8 revolutions)
    targetRevolutions = 8;
  } else if (lightIntensity > LIGHT_THRESHOLD_MED) {
    // Medium light - partially closed blinds (6 revolutions)
    targetRevolutions = 6;
  } else if (lightIntensity > LIGHT_THRESHOLD_LOW) {
    // Low light - mostly open blinds (4 revolutions)
    targetRevolutions = 4;
  } else {
    // Minimal light - fully open blinds (0 revolutions)
    targetRevolutions = 0;
  }
  // Convert revolutions to encoder counts
  return targetRevolutions * COUNTS_PER_REVOLUTION;
}