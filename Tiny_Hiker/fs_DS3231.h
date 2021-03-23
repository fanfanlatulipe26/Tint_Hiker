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
// version très alégée( !! ) de rodan https://github.com/rodan/ds3231
// Uniquement pour lire hour:min:sec dans une union de type ts7
// Utilisation I2Cdev
#ifndef __ds3231_h_
#define __ds3231_h_

//#if ARDUINO >= 100
#include <Arduino.h>
//#else
//#include <WProgram.h>
//#endif

// i2c slave address of the DS3231 chip
#define DS3231_I2C_ADDR             0x68

// timekeeping registers
#define DS3231_TIME_CAL_ADDR        0x00
#define DS3231_ALARM1_ADDR          0x07
#define DS3231_ALARM2_ADDR          0x0B
#define DS3231_CONTROL_ADDR         0x0E
#define DS3231_STATUS_ADDR          0x0F
#define DS3231_AGING_OFFSET_ADDR    0x10
#define DS3231_TEMPERATURE_ADDR     0x11

// control register bits
#define DS3231_CONTROL_A1IE     0x1   /* Alarm 2 Interrupt Enable */
#define DS3231_CONTROL_A2IE     0x2   /* Alarm 2 Interrupt Enable */
#define DS3231_CONTROL_INTCN    0x4   /* Interrupt Control */
#define DS3231_CONTROL_RS1      0x8   /* square-wave rate select 2 */
#define DS3231_CONTROL_RS2      0x10  /* square-wave rate select 2 */
#define DS3231_CONTROL_CONV     0x20  /* Convert Temperature */
#define DS3231_CONTROL_BBSQW    0x40  /* Battery-Backed Square-Wave Enable */
#define DS3231_CONTROL_EOSC     0x80  /* not Enable Oscillator, 0 equal on */


// status register bits
#define DS3231_STATUS_A1F      0x01   /* Alarm 1 Flag */
#define DS3231_STATUS_A2F      0x02   /* Alarm 2 Flag */
#define DS3231_STATUS_BUSY     0x04   /* device is busy executing TCXO */
#define DS3231_STATUS_EN32KHZ  0x08   /* Enable 32KHz Output  */
#define DS3231_STATUS_OSF      0x80   /* Oscillator Stop Flag */

union ts {
  struct ts1 {
    uint8_t sec;         /* seconds */
    uint8_t min;         /* minutes */
    uint8_t hour;        /* hours */
  } rtc;
  uint8_t rtcArray[3];
};

void DS3231_init(const uint8_t creg);
void DS3231_set(struct ts *t);
void DS3231_get(struct ts *t);

void DS3231_set_addr(const uint8_t addr, const uint8_t val);
uint8_t DS3231_get_addr(const uint8_t addr);

// control/status register
void DS3231_set_creg(const uint8_t val);
uint8_t DS3231_get_creg(void);

void DS3231_set_sreg(const uint8_t val);
uint8_t DS3231_get_sreg(void);



void DS3231_set_32kHz_output(const uint8_t on);

// helpers
uint8_t dectobcd(const uint8_t val);
uint8_t bcdtodec(const uint8_t val);
#endif
