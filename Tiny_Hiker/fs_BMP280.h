/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
//  subset / mix of a some libraries for BMP280
// Just a few function for the project witch i2Cdev interface
/***************************************************************************
  This is a library for the BMP280 pressure sensor

 ***************************************************************************/
#ifndef __BMP280_H__
#define __BMP280_H__

#include "Arduino.h"
#include <Wire.h>

/*=========================================================================
    I2C ADDRESS/BITS/SETTINGS
    -----------------------------------------------------------------------*/
#define BMP280_ADDRESS                (0x77)
#define BMP280_CHIPID                 (0x58)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum
{
  BMP280_REGISTER_DIG_T1              = 0x88,
  BMP280_REGISTER_DIG_T2              = 0x8A,
  BMP280_REGISTER_DIG_T3              = 0x8C,

  BMP280_REGISTER_DIG_P1              = 0x8E,
  BMP280_REGISTER_DIG_P2              = 0x90,
  BMP280_REGISTER_DIG_P3              = 0x92,
  BMP280_REGISTER_DIG_P4              = 0x94,
  BMP280_REGISTER_DIG_P5              = 0x96,
  BMP280_REGISTER_DIG_P6              = 0x98,
  BMP280_REGISTER_DIG_P7              = 0x9A,
  BMP280_REGISTER_DIG_P8              = 0x9C,
  BMP280_REGISTER_DIG_P9              = 0x9E,

  BMP280_REGISTER_CHIPID             = 0xD0,

  BMP280_REGISTER_SOFTRESET          = 0xE0,

  BMP280_REGISTER_CONTROL            = 0xF4,
  BMP280_REGISTER_CONFIG             = 0xF5,
  BMP280_REGISTER_PRESSUREDATA       = 0xF7,
  BMP280_REGISTER_TEMPDATA           = 0xFA,
};

/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
typedef struct
{
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;

  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;

} bmp280_calib_data;


class fs_BMP280
{
  public:
    /** Oversampling rate for the sensor. */
    enum sensor_sampling {
      /** No over-sampling. */
      SAMPLING_NONE = 0x00,
      /** 1x over-sampling. */
      SAMPLING_X1 = 0x01,
      /** 2x over-sampling. */
      SAMPLING_X2 = 0x02,
      /** 4x over-sampling. */
      SAMPLING_X4 = 0x03,
      /** 8x over-sampling. */
      SAMPLING_X8 = 0x04,
      /** 16x over-sampling. */
      SAMPLING_X16 = 0x05
    };

    /** Operating mode for the sensor. */
    enum sensor_mode {
      /** Sleep mode. */
      MODE_SLEEP = 0x00,
      /** Forced mode. */
      MODE_FORCED = 0x01,
      /** Normal mode. */
      MODE_NORMAL = 0x03,
      /** Software reset. */
      MODE_SOFT_RESET_CODE = 0xB6
    };

    /** Filtering level for sensor data. */
    enum sensor_filter {
      /** No filtering. */
      FILTER_OFF = 0x00,
      /** 2x filtering. */
      FILTER_X2 = 0x01,
      /** 4x filtering. */
      FILTER_X4 = 0x02,
      /** 8x filtering. */
      FILTER_X8 = 0x03,
      /** 16x filtering. */
      FILTER_X16 = 0x04
    };

    /** Standby duration in ms */
    enum standby_duration {
      /** 1 ms standby. */
      STANDBY_MS_1 = 0x00,
      /** 62.5 ms standby. */
      STANDBY_MS_63 = 0x01,
      /** 125 ms standby. */
      STANDBY_MS_125 = 0x02,
      /** 250 ms standby. */
      STANDBY_MS_250 = 0x03,
      /** 500 ms standby. */
      STANDBY_MS_500 = 0x04,
      /** 1000 ms standby. */
      STANDBY_MS_1000 = 0x05,
      /** 2000 ms standby. */
      STANDBY_MS_2000 = 0x06,
      /** 4000 ms standby. */
      STANDBY_MS_4000 = 0x07
    };

    fs_BMP280();

    bool  begin(uint8_t addr = BMP280_ADDRESS, uint8_t chipid = BMP280_CHIPID);
    float readTemperature(void);
    float readPressure(void);
    float readAltitude(float seaLevelhPa = 1013.25);
    float seaLevelForAltitude(float altitude, float atmospheric);

    void setSampling(sensor_mode mode = MODE_NORMAL,
                     sensor_sampling tempSampling = SAMPLING_X16,
                     sensor_sampling pressSampling = SAMPLING_X16,
                     sensor_filter filter = FILTER_OFF,
                     standby_duration duration = STANDBY_MS_1);


  private:
    /** Encapsulates the config register */
    struct config {
      /** Inactive duration (standby time) in normal mode */
      unsigned int t_sb : 3;
      /** Filter settings */
      unsigned int filter : 3;
      /** Unused - don't set */
      unsigned int none : 1;
      /** Enables 3-wire SPI */
      unsigned int spi3w_en : 1;
      /** Used to retrieve the assembled config register's byte value. */
      unsigned int get() {
        return (t_sb << 5) | (filter << 2) | spi3w_en;
      }
    };

    /** Encapsulates trhe ctrl_meas register */
    struct ctrl_meas {
      /** Temperature oversampling. */
      unsigned int osrs_t : 3;
      /** Pressure oversampling. */
      unsigned int osrs_p : 3;
      /** Device mode */
      unsigned int mode : 2;
      /** Used to retrieve the assembled ctrl_meas register's byte value. */
      unsigned int get() {
        return (osrs_t << 5) | (osrs_p << 2) | mode;
      }
    };

    void readCoefficients(void);


    uint8_t   _i2caddr;
    // int32_t   _sensorID;
    int32_t t_fine;  // pour lecture temp
    config _configReg;
    ctrl_meas _measReg;
    union {
      uint8_t  buf[sizeof(bmp280_calib_data)];
      bmp280_calib_data _bmp280_calib;
    } _u;
};

#endif
