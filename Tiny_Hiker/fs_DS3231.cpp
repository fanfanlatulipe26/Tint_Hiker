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
// version inspirée de la version de rodan  https://github.com/rodan/ds3231
// version mini, utilisation de I2Cdev, uniquement lecture/ecriture de hh,min,sec
// stockage dans une union
/*
 
  GNU GPLv3 license:

  This program is free software: you can redistribute it and/or modify
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

#include <Wire.h>
#include <I2Cdev.h>
#include <stdio.h>
#include "fs_ds3231.h"

#ifdef __AVR__
#include <avr/pgmspace.h>
// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
#ifdef PROGMEM
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))
#endif
#else
#define PROGMEM
#define xpgm_read_byte(addr) (*(const uint8_t *)(addr))
#endif

/* control register 0Eh/8Eh
  bit7 EOSC   Enable Oscillator (1 if oscillator must be stopped when on battery)
  bit6 BBSQW  Battery Backed Square Wave
  bit5 CONV   Convert temperature (1 forces a conversion NOW)
  bit4 RS2    Rate select - frequency of square wave output
  bit3 RS1    Rate select
  bit2 INTCN  Interrupt control (1 for use of the alarms and to disable square wave)
  bit1 A2IE   Alarm2 interrupt enable (1 to enable)
  bit0 A1IE   Alarm1 interrupt enable (1 to enable)
*/

void DS3231_init(const uint8_t ctrl_reg)
{
  DS3231_set_creg(ctrl_reg);
  DS3231_set_32kHz_output(false);
}

void DS3231_set(union ts *t)
{ 
  uint8_t TimeDate[3];  // second, minute, hour
  for (byte i = 0;i<=2;i++) {
    TimeDate[i] = dectobcd(t->rtcArray[i]);
    t->rtcArray[i] = TimeDate[i];
  }
  I2Cdev::writeBytes(DS3231_I2C_ADDR, DS3231_TIME_CAL_ADDR, 3, TimeDate);
}

void DS3231_get(union ts *t)
{
  uint8_t TimeDate[3];        //second,minute,hour
  I2Cdev::readBytes(DS3231_I2C_ADDR, DS3231_TIME_CAL_ADDR, 3, TimeDate);                  // Request 7 data bytes
  for (byte i = 0; i <= 2; i++) {
    t->rtcArray[i] = bcdtodec(TimeDate[i]);
  }
}

void DS3231_set_addr(const uint8_t addr, const uint8_t val)
{
  I2Cdev::writeByte(DS3231_I2C_ADDR, addr, val);
}

uint8_t DS3231_get_addr(const uint8_t addr)
{
  uint8_t rv;
  I2Cdev::readByte(DS3231_I2C_ADDR, addr, &rv);
}



// control register

void DS3231_set_creg(const uint8_t val)
{
  DS3231_set_addr(DS3231_CONTROL_ADDR, val);
}

uint8_t DS3231_get_creg(void)
{
  uint8_t rv;
  rv = DS3231_get_addr(DS3231_CONTROL_ADDR);
  return rv;
}

// status register 0Fh/8Fh

/*
  bit7 OSF      Oscillator Stop Flag (if 1 then oscillator has stopped and date might be innacurate)
  bit3 EN32kHz  Enable 32kHz output (1 if needed)
  bit2 BSY      Busy with TCXO functions
  bit1 A2F      Alarm 2 Flag - (1 if alarm2 was triggered)
  bit0 A1F      Alarm 1 Flag - (1 if alarm1 was triggered)
*/

void DS3231_set_sreg(const uint8_t val)
{
  DS3231_set_addr(DS3231_STATUS_ADDR, val);
}

uint8_t DS3231_get_sreg(void)
{
  uint8_t rv;
  rv = DS3231_get_addr(DS3231_STATUS_ADDR);
  return rv;
}



void DS3231_set_32kHz_output(const uint8_t on)
{
  /*
     Note, the pin1 is an open drain pin, therfore a pullup
     resitor is required to use the output.
  */
  if (on) {
    uint8_t sreg = DS3231_get_sreg();
    sreg &= ~DS3231_STATUS_OSF;
    sreg |= DS3231_STATUS_EN32KHZ;
    DS3231_set_sreg(sreg);
  } else {
    uint8_t sreg = DS3231_get_sreg();
    sreg &= ~DS3231_STATUS_EN32KHZ;
    DS3231_set_sreg(sreg);
  }
}

uint8_t dectobcd(const uint8_t val)
{
  return ((val / 10 * 16) + (val % 10));
  //    return (((val / 10) << 4) + (val % 10));  // idem code origine
}

uint8_t bcdtodec(const uint8_t val)
{
  return ((val / 16 * 10) + (val % 16));
  //      return ((val / 16 * 10) + (val & 0x0F));  // idem val % 16
  //   return (( val >>4 )*10 + (val %16));  // + gros
}
