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
// Some general #define for tiny_hiker  compass/altimeter/podometer
// Hadrware I2C addresses ...
// pin number for buttons

//#define i2cAddressBMP280 0x76
#define i2cAddressBMP280 0x77
#define i2cAddressEEPROM 0x50   // EEPROM qui contient le microcode
//#define i2cAddressEEPROM 0x57   // EEPROM qui contient le microcode

//#define MPU9250_I2C_address 0x68                                        // I2C address for MPU9250
#define MPU9250_I2C_address 0x69                                        // I2C address for MPU9250 
#define AK8963_I2C_address 0x0C                                             // I2C address for AK8963 magnetometer included in the MPU9250

// OLED SSD1306 is on default address 0x3C

#define BUTTON1_PIN 2      // wakeup / change screen /  increment-decrement by 1
#define BUTTON2_PIN 3      // wakeup / change screen / increment-decrement by 10
