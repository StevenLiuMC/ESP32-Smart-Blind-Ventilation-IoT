/*
    AsyncAPDS9306.cpp

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

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include "AsyncAPDS9306.h"

AsyncAPDS9306Data::AsyncAPDS9306Data(const uint32_t raw, const APDS9306_ALS_GAIN_t again, const APDS9306_ALS_MEAS_RES_t atime)
  : raw(raw), again(again), atime(atime) {}

float AsyncAPDS9306Data::calculateLux() {
  const float atimeValue = _integrationTime(atime);
  const float againValue = (float)_gainValue(again);

  return ((float)raw / againValue) * (100.0 / atimeValue);

}

float AsyncAPDS9306Data::_integrationTime(APDS9306_ALS_MEAS_RES_t atime) {
  switch (atime) {
    case APDS9306_ALS_MEAS_RES_20BIT_400MS : return 400.0;
    case APDS9306_ALS_MEAS_RES_19BIT_200MS : return 200.0;
    case APDS9306_ALS_MEAS_RES_18BIT_100MS : return 100.0;
    case APDS9306_ALS_MEAS_RES_17BIT_50MS : return 50.0;
    case APDS9306_ALS_MEAS_RES_16BIT_25MS : return 25.0;
    case APDS9306_ALS_MEAS_RES_13BIT_3MS : return 3.125;
  }
}
int AsyncAPDS9306Data::_gainValue(APDS9306_ALS_GAIN_t again) {
  switch (again) {
    case APDS9306_ALS_GAIN_1 : return 1;
    case APDS9306_ALS_GAIN_3 : return 3;
    case APDS9306_ALS_GAIN_6 : return 6;
    case APDS9306_ALS_GAIN_9 : return 9;
    case APDS9306_ALS_GAIN_18 : return 18;
  }
}


AsyncAPDS9306::AsyncAPDS9306() {}

bool AsyncAPDS9306::begin(APDS9306_ALS_GAIN_t again, APDS9306_ALS_MEAS_RES_t atime) {
  Wire.begin();

  int partId = _readRegister(APDS9306_CMD_PART_ID);
  if (!(partId == APDS9306_PART_ID_APDS9306 || partId == APDS9306_PART_ID_APDS9306065)) {
    return false;
  }
  setAlsGain(again);
  setAlsTime(atime);
  _powerOff();
  return true;
}

void AsyncAPDS9306::setAlsGain(const APDS9306_ALS_GAIN_t again) {
  _again = again;
  _writeValue(APDS9306_CMD_ALS_GAIN, _again, true);
}

void AsyncAPDS9306::setAlsTime(const APDS9306_ALS_MEAS_RES_t atime) {
  _atime = atime;
  _writeValue(APDS9306_CMD_ALS_MEAS_RATE, _atime | APDS9306_ALS_MEAS_RATE_2000MS, true);
}

APDS9306_ALS_GAIN_t AsyncAPDS9306::getAlsGain() {
  return _again;
}

APDS9306_ALS_MEAS_RES_t AsyncAPDS9306::getAlsTime() {
  return _atime;
}

int AsyncAPDS9306::_powerOn() {
  return _writeValue(APDS9306_CMD_MAIN_CTRL, APDS9306_MAIN_CTRL_ALS_ENABLE, true);
}

int AsyncAPDS9306::_powerOff() {
  int res = _writeValue(APDS9306_CMD_MAIN_CTRL, APDS9306_MAIN_CTRL_ALS_STANDBY, true);
  _readRegister(APDS9306_CMD_MAIN_STATUS);
  return res;
}

AsyncAPDS9306Data AsyncAPDS9306::syncLuminosityMeasurement() {
  startLuminosityMeasurement();
  while (!isMeasurementReady()) {}
  return getLuminosityMeasurement();
}

int AsyncAPDS9306::startLuminosityMeasurement() {
  return _powerOn();
}

AsyncAPDS9306Data AsyncAPDS9306::getLuminosityMeasurement() {
  _powerOff();

  uint32_t raw = _readRegister24(APDS9306_CMD_ALS_DATA_0);

  return AsyncAPDS9306Data(raw, _again, _atime);
}


bool AsyncAPDS9306::isMeasurementReady() {
  int reg = _readRegister(APDS9306_CMD_MAIN_STATUS);
  return reg & APDS9306_MAIN_STATUS_ALS_DATA;
}


uint8_t AsyncAPDS9306::_readRegister(uint8_t reg) {
  _write(reg, false);
  return _read();
}
uint32_t AsyncAPDS9306::_readRegister24(uint8_t reg) {
  _write(reg, false);
  return _read24();
}

uint8_t AsyncAPDS9306::_read() {
  Wire.requestFrom(APDS9306_I2C_ADDRESS, 1);
  return Wire.read();
}

uint32_t AsyncAPDS9306::_read24() {
  Wire.requestFrom(APDS9306_I2C_ADDRESS, 3);
  uint32_t result;
  result = Wire.read();
  result |= Wire.read() << 8;
  result |= Wire.read() << 8;
  return result;
}

int AsyncAPDS9306::_write(uint8_t reg, bool i2cStopMessage) {
  Wire.beginTransmission(APDS9306_I2C_ADDRESS);
  Wire.write(reg);
  return Wire.endTransmission(i2cStopMessage);
}

int AsyncAPDS9306::_writeValue(uint8_t reg, uint8_t value, bool i2cStopMessage) {
  Wire.beginTransmission(APDS9306_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(i2cStopMessage);
}

