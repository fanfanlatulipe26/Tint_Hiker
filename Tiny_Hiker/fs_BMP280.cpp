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
// Just a few function for the project with i2Cdev interface
/***************************************************************************
  This is a library for the BMP280 pressure sensor

 ***************************************************************************/
#include "Arduino.h"
#include <Wire.h>
//#include <SPI.h>
#include <I2Cdev.h>
#include "fs_BMP280.h"


fs_BMP280::fs_BMP280()
{ }

bool fs_BMP280::begin(uint8_t a, uint8_t chipid) {
  _i2caddr = a;
  uint8_t buf[1];
  I2Cdev::readByte(_i2caddr, BMP280_REGISTER_CHIPID, buf);
  if (buf[0] != chipid)
    return false;
  readCoefficients();
  I2Cdev::writeByte(_i2caddr, BMP280_REGISTER_SOFTRESET, 0xB6);// soft reset of the BMP just to be sure of the CONTROL and CONFIG regiser are reset
  setSampling();
  delay(100);

  return true;
}

/*!
   Sets the sampling config for the device.
   @param mode
          The operating mode of the sensor.
   @param tempSampling
          The sampling scheme for temp readings.
   @param pressSampling
          The sampling scheme for pressure readings.
   @param filter
          The filtering mode to apply (if any).
   @param duration
          The sampling duration.
*/
void fs_BMP280::setSampling(sensor_mode mode,
                                     sensor_sampling tempSampling,
                                     sensor_sampling pressSampling,
                                     sensor_filter filter,
                                     standby_duration duration) {
  _measReg.mode = mode;
  _measReg.osrs_t = tempSampling;
  _measReg.osrs_p = pressSampling;

  _configReg.filter = filter;
  _configReg.t_sb = duration;

 // write8(BMP280_REGISTER_CONFIG, _configReg.get());
//  write8(BMP280_REGISTER_CONTROL, _measReg.get());
  I2Cdev::writeByte(_i2caddr, BMP280_REGISTER_CONFIG,  _configReg.get());
  I2Cdev::writeByte(_i2caddr, BMP280_REGISTER_CONTROL, _measReg.get());

}



/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void fs_BMP280::readCoefficients(void)
{
  I2Cdev::readBytes(_i2caddr, BMP280_REGISTER_DIG_T1, 24, _u.buf);
}

/**************************************************************************/
//Return tmperature in °C
/**************************************************************************/
float fs_BMP280::readTemperature(void)
{
  int32_t var1, var2;
  int32_t  adc_T ;
  uint8_t buf[4];
  I2Cdev::readBytes(_i2caddr, BMP280_REGISTER_TEMPDATA, 3, buf);
  adc_T = ((uint32_t)buf[0] << 16) + ((uint16_t)buf[1] << 8) + buf[2];
  adc_T >>= 4;
  var1  = ((((adc_T >> 3) - ((int32_t)_u._bmp280_calib.dig_T1 << 1))) *
           ((int32_t)_u._bmp280_calib.dig_T2)) >> 11;

  var2  = (((((adc_T >> 4) - ((int32_t)_u._bmp280_calib.dig_T1)) *
             ((adc_T >> 4) - ((int32_t)_u._bmp280_calib.dig_T1))) >> 12) *
           ((int32_t)_u._bmp280_calib.dig_T3)) >> 14;

  t_fine = var1 + var2;

  float T  = (t_fine * 5 + 128) >> 8;
  return T / 100;
}

/**************************************************************************/
//return Barometric pressure in Pa.
/**************************************************************************/
float fs_BMP280::readPressure(void) {
  int64_t var1, var2, p;
  readTemperature();
  int32_t adc_P;
  uint8_t buf[4];
  I2Cdev::readBytes(_i2caddr, BMP280_REGISTER_PRESSUREDATA, 3, buf);
  adc_P = ((uint32_t)buf[0] << 16) + ((uint16_t)buf[1] << 8) + buf[2];
  adc_P >>= 4;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_u._bmp280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_u._bmp280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_u._bmp280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_u._bmp280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_u._bmp280_calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_u._bmp280_calib.dig_P1) >> 33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_u._bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_u._bmp280_calib.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)_u._bmp280_calib.dig_P7) << 4);
  return (float)p / 256;
}


/**************************************************************************/
//return approximate altitude in meter using barometric pressure and sea level pressure in hpA
//  from barometric formula ....
/**************************************************************************/
float fs_BMP280::readAltitude(float seaLevelhPa) {
  float altitude;
  float pressure = readPressure(); // Pascal
  pressure /= 100;
  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
  return altitude;
}

/**************************************************************************/
//   Return approximate pressure at sea level (in hPa) from the specified altitude (in meters)
//    and atmospheric pressure (in hPa).
//    from barometric formula ....
/**************************************************************************/
float fs_BMP280::seaLevelForAltitude(float altitude, float atmospheric) {

  return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}
