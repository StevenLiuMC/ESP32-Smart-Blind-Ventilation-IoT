/*
    AsyncAPDS9306.h

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



#ifndef AsyncAPDS9306_H
#define AsyncAPDS9306_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define APDS9306_I2C_ADDRESS 0x52

//Command register
#define APDS9306_CMD_MAIN_CTRL  0x00
#define APDS9306_CMD_ALS_MEAS_RATE  0x04
#define APDS9306_CMD_ALS_GAIN 0x05
#define APDS9306_CMD_PART_ID 0x06
#define APDS9306_CMD_MAIN_STATUS 0x07
#define APDS9306_CMD_CLEAR_DATA_0 0x0A
#define APDS9306_CMD_CLEAR_DATA_1 0x0B
#define APDS9306_CMD_CLEAR_DATA_2 0x0C
#define APDS9306_CMD_ALS_DATA_0 0x0D
#define APDS9306_CMD_ALS_DATA_1 0x0E
#define APDS9306_CMD_ALS_DATA_2 0x0F
#define APDS9306_CMD_INT_CFG 0x19
#define APDS9306_CMD_INT_PERSISTENCE 0x1A
#define APDS9306_CMD_ALS_THRES_UP_0 0x21
#define APDS9306_CMD_ALS_THRES_UP_1 0x22
#define APDS9306_CMD_ALS_THRES_UP_2 0x23
#define APDS9306_CMD_ALS_THRES_LOW_0 0x24
#define APDS9306_CMD_ALS_THRES_LOW_1 0x25
#define APDS9306_CMD_ALS_THRES_LOW_2 0x26
#define APDS9306_CMD_ALS_THRES_VAR 0x27


//MAIN_CTRL register
#define APDS9306_MAIN_CTRL_SW_RESET 0x10
#define APDS9306_MAIN_CTRL_ALS_ENABLE 0x02
#define APDS9306_MAIN_CTRL_ALS_STANDBY 0x00


typedef enum
{

  APDS9306_ALS_MEAS_RES_20BIT_400MS = 0x00,
  APDS9306_ALS_MEAS_RES_19BIT_200MS = 0x10,
  APDS9306_ALS_MEAS_RES_18BIT_100MS = 0x20, //DEFAULT
  APDS9306_ALS_MEAS_RES_17BIT_50MS = 0x30,
  APDS9306_ALS_MEAS_RES_16BIT_25MS = 0x40,
  APDS9306_ALS_MEAS_RES_13BIT_3MS = 0x50
}
APDS9306_ALS_MEAS_RES_t;


typedef enum
{

  APDS9306_ALS_MEAS_RATE_25MS = 0x00,
  APDS9306_ALS_MEAS_RATE_50MS = 0x01,
  APDS9306_ALS_MEAS_RATE_100MS = 0x02,
  APDS9306_ALS_MEAS_RATE_200MS = 0x03,
  APDS9306_ALS_MEAS_RATE_500MS = 0x04,
  APDS9306_ALS_MEAS_RATE_1000MS = 0x05,
  APDS9306_ALS_MEAS_RATE_2000MS = 0x06
}
APDS9306_ALS_MEAS_RATE_t;

typedef enum
{

  APDS9306_ALS_GAIN_1 = 0x00,
  APDS9306_ALS_GAIN_3 = 0x01,
  APDS9306_ALS_GAIN_6 = 0x02,
  APDS9306_ALS_GAIN_9 = 0x03,
  APDS9306_ALS_GAIN_18 = 0x04
}
APDS9306_ALS_GAIN_t;




//PART_ID register
//#define APDS9306_PART_ID_PART_ID 0xF0
//#define APDS9306_PART_ID_REVISION_ID 0x0F

#define APDS9306_PART_ID_APDS9306 0xB1
#define APDS9306_PART_ID_APDS9306065 0xB3

//MAIN_STATUS register
#define APDS9306_MAIN_STATUS_POWER_ON 0x20
#define APDS9306_MAIN_STATUS_ALS_INT 0x10
#define APDS9306_MAIN_STATUS_ALS_DATA 0x08


class AsyncAPDS9306Data {
  public:
    AsyncAPDS9306Data(const uint32_t raw, const APDS9306_ALS_GAIN_t again, const APDS9306_ALS_MEAS_RES_t atime);

    const uint16_t raw;
    const APDS9306_ALS_GAIN_t again;
    const APDS9306_ALS_MEAS_RES_t atime;
    float _integrationTime(APDS9306_ALS_MEAS_RES_t atime);
    int _gainValue(APDS9306_ALS_GAIN_t again);

    float calculateLux();
};

class AsyncAPDS9306 {
  public:
    AsyncAPDS9306();

    bool begin(APDS9306_ALS_GAIN_t again, APDS9306_ALS_MEAS_RES_t atime);

    AsyncAPDS9306Data syncLuminosityMeasurement();

    int startLuminosityMeasurement();

    bool isMeasurementReady();

    AsyncAPDS9306Data getLuminosityMeasurement();


    void setAlsGain(const APDS9306_ALS_GAIN_t again);
    void setAlsTime(const APDS9306_ALS_MEAS_RES_t atime);

    APDS9306_ALS_GAIN_t getAlsGain();
    APDS9306_ALS_MEAS_RES_t getAlsTime();

  private:


    APDS9306_ALS_GAIN_t _again;
    APDS9306_ALS_MEAS_RES_t _atime;

    int _powerOn();
    int _powerOff();

    uint8_t _read();
    uint32_t _read24();
    int _write(uint8_t reg, bool i2cStopMessage);
    int _writeValue(uint8_t reg, uint8_t value, bool i2cStopMessage);

    uint8_t _readRegister(uint8_t reg);
    uint32_t _readRegister24(uint8_t reg);


};

#endif // AsyncAPDS9306_H








