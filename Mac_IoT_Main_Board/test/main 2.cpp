#define BLYNK_TEMPLATE_ID "TMPL2koX3T71B"
#define BLYNK_TEMPLATE_NAME "Project 3 IoT"
#define BLYNK_AUTH_TOKEN "Skl-GKSJhCPU0c5PhWyWXcKwj7lIMh14"

#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <AsyncAPDS9306.h>

AsyncAPDS9306 lightSensor;


// Wi-Fi credentials
char ssid[] = "McMaster Graduate";
char pass[] = "10BayMcMaster";

const int signalPin1 = 34; // Vb (bridge output)
const int signalPin2 = 39; // Va (bridge output)
const float V_source = 3.3;
const float R0 = 10000.0;
const int numSamples = 10;
float temperatureSamples[numSamples];

BlynkTimer timer;

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

float calculateStandardDeviation(float data[], int size, float mean) {
  float sumSquaredDiffs = 0;
  for (int i = 0; i < size; i++) sumSquaredDiffs += pow(data[i] - mean, 2);
  return sqrt(sumSquaredDiffs / size);
}

void readAndSendSensorData() {
  float tempVal1 = 0, tempVal2 = 0;
  float R2 = 0;

  for (int i = 0; i < numSamples; i++) {
    tempVal1 = analogRead(signalPin1);
    tempVal2 = analogRead(signalPin2);

    R2 = calculateThermistorResistance(tempVal1, tempVal2);
    temperatureSamples[i] = steinhart(R2);

    delay(1000);
  }

  float meanTemperature = calculateMean(temperatureSamples, numSamples);
  float stdDev = calculateStandardDeviation(temperatureSamples, numSamples, meanTemperature);

  // Light sensor reading
  AsyncAPDS9306Data lightData = lightSensor.syncLuminosityMeasurement();
  float lux = lightData.calculateLux();


  // Sending data to Blynk
  Blynk.virtualWrite(V0, meanTemperature); // Mean temperature
  Blynk.virtualWrite(V1, lux);             // Light level (lux)
  
  Serial.print("Mean Temp: "); Serial.print(meanTemperature); Serial.println(" °C");
  Serial.print("Light (lux): "); Serial.println(lux);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Connecting to Wi-Fi and Blynk...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  if (Blynk.connected()) {
    Serial.println("Connected to Blynk successfully!");
  } else {
    Serial.println("Blynk connection failed!");
  }

  // if (!lightSensor.begin(APDS9306_ALS_GAIN_1, APDS9306_ALS_MEAS_RES_18BIT_100MS)) {
  //   Serial.println("APDS9306 not found!");
  //   while (1); // Halt if not found
  // }
  

  timer.setInterval(15000L, readAndSendSensorData);
  Serial.println("Initialized");
}


void loop() {
  Blynk.run();
  timer.run();
}

