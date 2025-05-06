// This code is developed using a ESP32 Developing Board provided by McMaster University.

#define BLYNK_TEMPLATE_ID "TMPL2PCqEU6B2"
#define BLYNK_TEMPLATE_NAME "SEP 783 Project3 Smart Blind Ventilation System"
#define BLYNK_AUTH_TOKEN "-jvOwqn5bAVoUcuyXgfOSoIvbf9BLp2R"

#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <AsyncAPDS9306.h>
#include <MCP23017.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Sensor and GPIO
#define MCP23017_ADDR 0x20
MCP23017 mcp(MCP23017_ADDR);
const int ledPin = 4;  // GPA4

AsyncAPDS9306 lightSensor;

const int signalPin1 = 34;
const int signalPin2 = 39;
const float V_source = 3.3;
const float R0 = 10000.0;
const int numSamples = 10;
float temperatureSamples[numSamples];

float latestMeanTemperature = 0;
float latestLux = 0;
float temperatureThreshold = 0;
int manualMode = 0;
int guardMode = 0;
// Wi-Fi Credentials (Hotspot)
char ssid[] = "example_ssid";
char pass[] = "12345678";  // Replace with actual hotspot password

IPAddress local_IP(123, 45, 67, 5);   // Chosen IP (not in use)
IPAddress gateway(123, 45, 67, 1);      // From your PC's network info
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(123, 45, 67, 1);          // Same as gateway

// Below are my IP settings, I'll keep them here for your reference
// IPAddress local_IP(172, 20, 10, 5);   // Chosen IP (not in use)
// IPAddress gateway(172, 20, 10, 1);      // From your PC's network info
// IPAddress subnet(255, 255, 255, 0);
// IPAddress dns(172, 20, 10, 1);          // Same as gateway


// Blynk and HTTP server
BlynkTimer timer;
AsyncWebServer server(80);  // ← Fixed Port 80

float calculateThermistorResistance(int adc1, int adc2) {
  float Vb = adc1 * (V_source / 4096.0);
  float Va = adc2 * (V_source / 4096.0);
  float Vdiff = Vb - Va;
  float deltaR = (Vdiff * (2 * R0)) / V_source;
  float R2 = R0 + deltaR;
  return (R2 <= 0) ? 10000.0 : R2;
}

float steinhart(float R) {
  if (R <= 0) return NAN;
  float Rref = 10000.0;
  float A = 0.003354016;
  float B = 0.0002569850;
  float C = 0.000002620131;
  float D = 0.00000006383091;
  float E = log(R / Rref);
  float T = 1 / (A + (B * E) + (C * E * E) + (D * E * E * E));
  return T - 273.15;
}

float calculateMean(float data[], int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += data[i];
  return sum / size;
}

void readAndSendSensorData() {
  float tempVal1 = 0, tempVal2 = 0;
  float R2 = 0;

  for (int i = 0; i < numSamples; i++) {
    tempVal1 = analogRead(signalPin1);
    tempVal2 = analogRead(signalPin2);
    R2 = calculateThermistorResistance(tempVal1, tempVal2);
    temperatureSamples[i] = steinhart(R2);
  }

  latestMeanTemperature = calculateMean(temperatureSamples, numSamples);
  AsyncAPDS9306Data lightData = lightSensor.syncLuminosityMeasurement();
  latestLux = lightData.calculateLux();

  Blynk.virtualWrite(V0, latestMeanTemperature);
  Blynk.virtualWrite(V1, latestLux);

  Serial.print("Mean Temp: "); Serial.print(latestMeanTemperature); Serial.println(" °C");
  Serial.print("Light (lux): "); Serial.println(latestLux);
}

BLYNK_WRITE(V6) {
  temperatureThreshold = param.asFloat();
  Serial.print("Updated temp threshold from Blynk: ");
  Serial.println(temperatureThreshold);
}

BLYNK_WRITE(V7) {
  manualMode = param.asInt();
  Serial.print("Updated manual mode from Blynk: ");
  Serial.println(manualMode);
}


BLYNK_WRITE(V8) {
  guardMode = param.asInt();
  Serial.print("Updated guard mode from Blynk: ");
  Serial.println(guardMode);
}


void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Connect to Hotspot
  WiFi.config(local_IP, gateway, subnet, dns);
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start Blynk
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();

  // MCP
  mcp.init();
  mcp.portMode(MCP23017Port::A, 0x00);
  mcp.digitalWrite(ledPin, 0);

  // Blynk timer
  timer.setInterval(5000L, readAndSendSensorData);

  // HTTP Endpoints
  server.on("/indoorTemp", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(latestMeanTemperature));
  });

  server.on("/brightness", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(latestLux));
  });

  server.on("/tempThreshold", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(temperatureThreshold));
  });

  server.on("/manualMode", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(manualMode));
  });

  
  server.on("/guardMode", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(guardMode));
  });
  
  server.on("/systemStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{\"temperature\":" + String(latestMeanTemperature) +
                  ", \"brightness\":" + String(latestLux) + "}";
    request->send(200, "application/json", json);
  });

  server.begin();
  Serial.println("HTTP server started on port 80.");
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
  checkBlynkConnection();
  Blynk.run();
  timer.run();
}
