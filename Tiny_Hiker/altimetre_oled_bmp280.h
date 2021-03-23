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
#include <Arduino.h>  // for type definitions
void showScreen(const __FlashStringHelper *label, float value, byte unit);
void drawScreen(void (*draw)());
void displayTitleBattery(const __FlashStringHelper *label);
//void displayPressureForecast() ;
void displayCumul();
void displayPodometre();
void displaySettingAltitude();
void displaySettingSleep();
void displaySettingPressure();

void debugPression () ;

//void savePressureSample(float pressure);
float getPressureAverage() ;

void setAltiMinMax();
void resetAltiMinMax();
//void handleButtonReleaseEvents(Button &btn);
//void handleButtonHoldEvents(Button &btn);

void drawCar(byte sx, byte sy, byte num, const uint8_t *font, byte fw, byte fh);
void drawSymbol(byte sx, byte sy, byte num);
void drawFloatValue(byte sx, byte sy, float val, byte unit);

//void displayTitleBattery(const __FlashStringHelper *label);
void displayTitleBattery(const __FlashStringHelper );
void showBatterylevel(int vcc);
int readVcc() ;

void resetHistory() ;
void storeHistory(float value);
void displayHistory();
void drawTicsScale();
void display_mSpace();
void blink() ;
void flashScreen(byte count = 1) ;
