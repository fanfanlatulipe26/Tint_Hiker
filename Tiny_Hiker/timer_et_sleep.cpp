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
/*

   Gestion d'un timer mis a jour par le watchdog (et qui fonctionne donc même si le CPU est
   en mode SLEEP_MODE_PWR_DOWN

   Gestion de la mise en sommeil , et reveil sur IT externe (action sur le bouton de l'altimetre)
   void sleepBuntonPressed ()

   Inspiré de Test vu dans http://forum.arduino.cc/index.php?topic=199576.0 post#38 de Nick Gammon
   et de http://forum.arduino.cc/index.php?topic=38046.0  (pour gestion timer

*/



#include <arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "timer_et_sleep.h"
#include "tiny_hiker.h"

extern volatile int sleep_count;
//extern volatile byte sleep_count;
extern volatile boolean extInterrupt;    //external interrupt flag (button)



// Internal function: Start watchdog timer
// byte psVal - Prescale mask
void WDT_On (byte psVal)
{
  // prepare timed sequence first
  byte ps = (psVal | (1 << WDIE)) & ~(1 << WDE);
  cli();
  wdt_reset();
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1 << WDRF);
  // start timed sequence
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // set new watchdog timeout value
  WDTCSR = ps;
  sei();
}

// Internal function.  Stop watchdog timer
void WDT_Off() {
  cli();
  wdt_reset();
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1 << WDRF);
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional time-out */
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;
  sei();
}

// wdt int service routine
ISR(WDT_vect) {
  sleep_count++;
}

void wakeOnInterrupt ()
{
  extInterrupt = true;
  // don't need the external interrupts any more
  detachInterrupt(digitalPinToInterrupt(leftButtonPin));
  detachInterrupt(digitalPinToInterrupt(rightButtonPin));
}

void sleepBuntonPressed () {
  byte adcsra = ADCSRA;          //save the ADC Control and Status Register A
  ADCSRA = 0;                    //disable the ADC
  noInterrupts ();     // timed sequences follow
  EIFR = bit (INTF0);  // clear flag for interrupt 0  (pin D2).  cancel any existing pending falling interrupt (interrupt 1)
  EIFR = bit (INTF1);  // clear flag for interrupt 1  (pin D3).  cancel any existing pending falling interrupt (interrupt 1)
  
  attachInterrupt (digitalPinToInterrupt(leftButtonPin), wakeOnInterrupt, FALLING);  // reveil par bouton central sur pin 2
  attachInterrupt (digitalPinToInterrupt(rightButtonPin), wakeOnInterrupt, FALLING);  // reveil par bouton gauche sur pin 3
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  sleep_enable();
  byte mcucr1 = MCUCR | bit(BODS) | bit(BODSE); //turn off the brown-out detector while sleeping
  byte mcucr2 = mcucr1 & ~bit(BODSE);
  MCUCR = mcucr1; //timed sequence
  MCUCR = mcucr2; //BODS stays active for 3 cycles, sleep instruction must be executed while it's active
  interrupts ();      // need interrupts now
  sleep_cpu();                   //go to sleep
  sleep_disable();               //wake up here   (from bounton action or watchdog)
  ADCSRA = adcsra;               //restore ADCSRA

}
