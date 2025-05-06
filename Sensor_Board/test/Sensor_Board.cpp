#include <Arduino.h>
#include <Wire.h>
#include <Si7020.h>
#include <AsyncAPDS9306.h>
#include <Arduino_LSM9DS1.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// WiFi credentials for Access Point
const char* ssid = "ESP32-SensorBoard";
const char* password = "sensor123";

// Sensors
Si7020 si7020;
AsyncAPDS9306 lightSensor;

// Server
AsyncWebServer server(80);

// Sensor data variables
float temperature = 0.0;
float humidity = 0.0;
float lightIntensity = 0.0;
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;

// IMU calibration parameters
float Bias_X = 0, Bias_Y = 0, Bias_Z = 0;
float Scale_X = 1, Scale_Y = 1, Scale_Z = 1;

// Function prototypes
void calibrateAccelerometer();
void setupSensors();
void setupWiFi();
void setupServer();
void updateSensorData();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("Initializing Sensor Board...");
  
  // Initialize sensors
  setupSensors();
  
  // Set up WiFi Access Point
  setupWiFi();
  
  // Set up HTTP server
  setupServer();
  
  Serial.println("Sensor Board initialization complete!");
}

void loop() {
  // Update sensor readings periodically
  updateSensorData();
  delay(1000); // Update every second
}

void setupSensors() {
  // Initialize Si7020 temperature/humidity sensor
  si7020.begin();
  Serial.println("Si7020 sensor initialized");
  
  // Initialize APDS9306 light sensor
  lightSensor.begin(APDS9306_ALS_GAIN_1, APDS9306_ALS_MEAS_RES_20BIT_400MS);
  Serial.println("APDS9306 sensor initialized");
  
  // Initialize LSM9DS1 IMU
  if (IMU.begin()) {
    Serial.println("LSM9DS1 sensor initialized");
    IMU.setContinuousMode();
    calibrateAccelerometer();
  } else {
    Serial.println("Failed to initialize LSM9DS1 sensor");
  }
}

void setupWiFi() {
  Serial.print("Setting up Access Point...");
  
  // Setup Access point with SSID and Password
  WiFi.softAP(ssid, password);
  
  // Get the AP IP address
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

void setupServer() {
  // Route for sensor data
  server.on("/sensors", HTTP_GET, [](AsyncWebServerRequest *request){
    String jsonResponse = "{";
    jsonResponse += "\"temperature\":" + String(temperature, 2);
    jsonResponse += ", \"humidity\":" + String(humidity, 2);
    jsonResponse += ", \"light\":" + String(lightIntensity, 2);
    jsonResponse += ", \"accelX\":" + String(accelX, 3);
    jsonResponse += ", \"accelY\":" + String(accelY, 3);
    jsonResponse += ", \"accelZ\":" + String(accelZ, 3);
    jsonResponse += "}";
    request->send(200, "application/json", jsonResponse);
  });
  
  // Route specifically for light sensor data
  server.on("/light", HTTP_GET, [](AsyncWebServerRequest *request){
    String response = String(lightIntensity, 2);
    request->send(200, "text/plain", response);
  });
  
  // Start server
  server.begin();
  Serial.println("HTTP server started");
}

void updateSensorData() {
  // Read temperature and humidity
  temperature = si7020.getTemp();
  humidity = si7020.getRH();
  
  // Read light intensity
  lightSensor.startLuminosityMeasurement();
  while (!lightSensor.isMeasurementReady()) {
    delay(10);
  }
  AsyncAPDS9306Data lightData = lightSensor.getLuminosityMeasurement();
  lightIntensity = lightData.calculateLux();
  
  // Read accelerometer data
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    
    // Apply calibration
    accelX = (accelX - Bias_X) * Scale_X;
    accelY = (accelY - Bias_Y) * Scale_Y;
    accelZ = (accelZ - Bias_Z) * Scale_Z;
  }
  
  // Debug output
  Serial.print("Temperature: ");
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
}

void calibrateAccelerometer() {
  // Acceleration data in 6 directions (simplified values)
  float Xp = 0.97, Xn = -1.03;
  float Yp = 0.97, Yn = -1.01;
  float Zp = 0.97, Zn = -1.03;

  // Calculate zero bias
  Bias_X = (Xp + Xn) / 2;
  Bias_Y = (Yp + Yn) / 2;
  Bias_Z = (Zp + Zn) / 2;

  // Calculate the scale factor
  Scale_X = 2.0 / (Xp - Xn);
  Scale_Y = 2.0 / (Yp - Yn);
  Scale_Z = 2.0 / (Zp - Zn);

  Serial.println("Accelerometer calibration complete");
}