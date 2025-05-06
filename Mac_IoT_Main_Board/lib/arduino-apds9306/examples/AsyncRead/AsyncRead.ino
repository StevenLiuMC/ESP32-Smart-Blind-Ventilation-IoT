/*
    AsyncRead.ino

    Created by Grégory Marti <greg.marti@gmail.com>
    Copyright 2017 Grégory Marti

    This file is part of the AsyncAPDS9306 library.

    AsyncAPDS9306 library is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <AsyncAPDS9306.h>

AsyncAPDS9306 sensor;

const APDS9306_ALS_GAIN_t again = APDS9306_ALS_GAIN_1;
const APDS9306_ALS_MEAS_RES_t atime = APDS9306_ALS_MEAS_RES_16BIT_25MS;

void setup() {
  Serial.begin(9600);
  if (sensor.begin(again, atime)) {
    Serial.print("Sensor found !");
  } else {
    Serial.print("Sensor not found !");
  }
}


void loop() {

  unsigned long startTime;
  unsigned long duration;

  Serial.println("Starting luminosity measurement");
  startTime = millis();

  sensor.startLuminosityMeasurement();

  while (!sensor.isMeasurementReady()) {
  }

  duration = millis() - startTime;

  AsyncAPDS9306Data data = sensor.getLuminosityMeasurement();

  Serial.print(duration);
  Serial.print("ms (following datasheet, should be "); Serial.print(data._integrationTime(atime)); Serial.println("ms");


  Serial.print(F("Raw: ")); Serial.println(data.raw);
  float lux = data.calculateLux();
  Serial.print("Luminosity : ");
  Serial.print(lux, 2);
  Serial.println("lux");
  delay(2000);
}

