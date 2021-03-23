// 3/2021  beta1
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
// Rien fait en fait pour v3j !!
// 19/1/2021 Debut v3j    (1 seul refresh pour photo/video)
//19/1/2021 fin v3i. Debut v3j    (1 seul refresh pour photo/video)
// version pour MPU9250 (mpu6500)
// 17/1/2021
// version 3i avec new BMP280:  29384/30720     875/1166 
//  (avant fin version 3h (ancien BMP280): 29684/30720      888)
//
// mettre historyLast en local ??????
// 29498/882

// calcul dénivelé mauvais farfelu ?? Semble lié à altitude min qui ne s'affich pas (donc inf à -999 ????
// Quand on revient du sleep , parfois on part en setting ??  Semble uniquement pour clock
// v3j
//  BMP ??voir

// A faire: pourquoi avoir CurrentAltitude, et ne pas utiliser directement altitude dans le calcul de weather forecast ???????
// currentAltitude n'est mis a jour que si on change le QNH ??????
//Interer de sauvegarder currentAltitude en EEPROM ????
//  Utilisation de pression ??   presionpa ??     pressionpa inutile
// 23/11/2020: debut v3f  (clock ??)


// #define baseTime 8   // durée en sec du watchdog  ?????? Pourquoi pas la vraie valeur 8.55

// verif par rapport au Versoud https://www.infoclimat.fr/observations-meteo/temps-reel/grenoble-le-versoud/07487.html

/*
  Tiny altimeter / pedometer
  Feature sleepmode for power saving,

  Arduino Pro Mini 3v 8Mhz, BMP280 pressure sensor, MPU6050 accelerometer, EEPROM, OLED display 128x64
  The start point for this work is the TinyAltimeter an original development from  "ogaillard": ogaillard63@gmail.com

  We use the embeded pedometer feature of the DMP in the MPU6050. It is always running when the DMP is enabled.
  For saving space in the program memory or the Arduino, an external I2C EEPROM is used. It contain the microcode
  of the DMP that is loaded in the MPU/DMP at startup (4kb needed with release 5.1.3 from InvenSense)
  ============
  - sleep mode
  - history
  - total gain/lost


    Short press on the button 1 (middle one): wake up if sleeping or switch to next screen
    Short press on the button 2 (left one): wake up if sleeping or switch to previous screen
    Screen 1:Altitude
             Long press button 1 (middle): enter calibration mode
                Increment altitude (up arrow)
                Short press button 1 (middle): increment altitude by 1 meter
                Short press button 2 (left): increment altitude by 10 meters
                Long press button 1 (middle): decrement altitude (down arrow)
                Short press button 1 (middle): decrement altitude by 1 meter
                Short press button 2 (left): decrement altitude by 10 meters
                Long press button 1 (middle): exit calibration mode
    Screen 2:Altitude Max
             Long press button 1 (middle): reset altitude Max, altitude Min and history
    Screen 3:Altitude Min
             Long press button 1 (middle): reset altitude Max, altitude Min and history
    Screen 4:Pressure
             Long press button 1 (middle): enter pressure mode menu.
                Short press on button 1 or 2:  toggle pressure mode sea level / absolute  / Trend mode
                Long press button 1 (middle): exit pressure mode menu
    Screen 5:Temperature
             Long press button 1 (middle): enter sleep mode setting on / off
                Short press on button 1 or 2:  toggle sleep mode on / off
                Long press button 1 (middle): exit sleep mode setting.
    Screen 6:Battery
            Long press button 1 (middle): enter freeze history menu.
                Short press on button 1 or 2: toggle freeze on/off.
                Long press button 1 (middle): exit freeze setting menu.
    Screen 7:Graphical History
            Long press button 1 (middle): reset altitude Max, altitude Min and history
    Scrren 8:Cumulated gain / loss
            Long press button 1 (middle): reset cumulated gain / loss
    Screen 9:Pedometer
            Long press button 1 (middle): reset pedometer count

*/

/*



  pin 2     rightButton (right) wake up, next  screen , enter setup, reset, increment / decrement by 1
  pin 3     leftButton (left) previous screen, increment / decrement by 10

  I2C devices   ddresses:
  60 (0x3C)   OLED 128x64
  80 (0x50)   EEPROM with microcode for MPU/DMP6050
  104 (0x68)  MPU6050 accelerometer/gyroscope
  118 (0x76)  BMP280 pressure sensor

  Variation de 1hPa pour 8m environ

*/



#include <Wire.h>
#include <I2Cdev.h>
#include <Button.h>    //   utiliser la version de button de https://github.com/t3db0t/Button ( Tom Igoe's fork , pas l'original de Brevig)
#include <EEPROM.h>
#include <U8glib.h>
#include "fs_BMP280.h"
#include "fs_DS3231.h"
#include "altimetre_oled_bmp280.h"
#include "timer_et_sleep.h"
#include "mpu_dmp_mini_podometre.h"
#include "tiny_hiker.h"

long T0, T1;
//#define signatureAltimetre 0xA4
#define signatureAltimetre 0xA5  // signature dans EEPROM processeur pour savoir si data valide
struct objectEEPROM   // configuration sauvegardée en EEPROM processeur. Valeur par defauts utiles à la 1er execution
{
  byte signature = signatureAltimetre;
  char Version[3] = "v3i";  //
  float QNH = 1013.25;
  float currentAltitude = 0.0;
  int Mag_xyz_offset[3] = {123, -30, -419}; // Hard-iron offsets
  float Mag_xyz_scale[3] = {0.72, 0.74, 3.77}; // Soft-iron scale factors
} configEEPROM;

const long screenTimeOut = 15000; // timeout si pas d'activité, en ms
unsigned long t0ScreenTimeOut;
boolean powerSaver = true; // par defaut le system de mise en sommeil/coupure écran est utilisé: ON
//boolean powerSaver = false; //  par defaut le system de mise en sommeil/coupure écran n'est pas utilisé OFF
boolean screenOff = false;  // l'écran est allumé au demarrage  (screenOff true: screen is turned OFF)

boolean pressureAbsolute = false ;// by default for the pressure screen display the sea level pressure

boolean freezeHistory = false; //
// access code in the Symbol font
const byte  HPA = 0;
const byte  METER = 1;
const byte  DEG = 2;
const byte  VOLT = 3;
const byte arrowUp = 4;
const byte arrowDown = 5;

U8GLIB_SSD1306_128X64 display(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0); // I2C / TWI

fs_BMP280 pressure; // I2C

Button rightButton = Button(rightButtonPin, BUTTON_PULLUP_INTERNAL, true, 50 );
Button leftButton = Button(leftButtonPin, BUTTON_PULLUP_INTERNAL, true, 50 );

//struct ts rtc;
union  ts rtcUnion;

extern const uint8_t Font24x40[];
extern const uint8_t Symbol[];
extern const uint8_t Battery[];
extern const uint8_t Die_2[]; //  Splash screen

boolean longPush = false;

enum lesSettings : byte {
  settingNo,
  settingAltitudeModeIncr,
  settingAltitudeModeDecr,
  settingFreezeHistoryMode,
  settingPowerSaverMode,
  settingPressureMode,
  settingCalibrateMode,
  settingWaitingForSave,     // waiting for save of calibration
  settingSaveCalibration,
  settingClockHour,
  settingClockMinute

} stateSetting = settingNo;

float QNH;
float temperature, pression, Pweather, Simpleweatherdiff;
float pressionPa, pressionPaPrevious; // pour debug et +.print
float baseAltitude = 0, currentAltitude;
float altiMin = 9999.0;
float altiMax = -9999.0;
float altitude = 0.0;

// ----- Magnetometer

#define AK8963_cntrl_reg_1 0x0A                                             // CNTL[4]=#bits, CNTL[3:0]=mode
#define AK8963_status_reg_1 0x02                                            // ST1[0]=data ready
#define AK8963_data_ready_mask 0b00000001                                   // Data ready mask
#define AK8963_overflow_mask 0b00001000                                     // Magnetic sensor overflow mask
#define AK8963_data 0x03                                                    // Start address of XYZ data                                                                
#define AK8963_fuse_ROM 0x10
#define MPU9250_Interface_bypass_mux_enable 0x37                        // INT_PIN_CFG[1]= BYPASS_EN

int mag_xyz[3];   //int mag_x, mag_y, mag_z;   // raw values with only ASAX correction
int Mag_xyz[3];   //  Mag_x = (mag_x_raw - Mag_x_offset) * Mag_x_scale;
float Mag_xyz_dampened[3] ; // float   Mag_x_dampened,       Mag_y_dampened,       Mag_z_dampened;
float ASA[3]  ; // ASAX, ASAY, ASAZ   magnetometer factory adjustement
float Heading;
/*
  Obtain your magnetic declination from http://www.magnetic-declination.com/
  Uncomment the declination code within the main loop() if you want True North.
*/

float   Declination = +2.18;     //  pour Grenoble  2° 11' Degrees ... replace this declination with yours

float averagePressure = 0;

enum lesEcrans : byte {
  dummyFirst,
  AltitudeS,
  AltitudeMaxS,
  AltitudeMinS,
  PressureS,
  TemperatureS,
  BatteryS,
  HistoryS,
  GainLossS,
  PedometerS,
  CompassS,  // 10
  ClockS,
  dummyLast
} screen = AltitudeS; // numero d'ecran
// Dans ancien code:
//   1:Alt,  2:Alt max, 3:Alt Min, 4:Pression(sea level, absolute, trend),
//   5:Temp, 6:Batterie, 7:historique, 8:denivellé, 9 podomètre
//  Overloaded operator ++ and --  taking care of wrapp,  (just for fun !!)
lesEcrans& operator++(lesEcrans& d)
{
  return d = (d == (dummyLast - 1) ? (dummyFirst + 1) : static_cast<lesEcrans>(static_cast<byte>(d) + 1));
}
lesEcrans& operator--(lesEcrans& d)
{
  return d = (d == (dummyFirst + 1) ? (dummyLast - 1) : static_cast<lesEcrans>(static_cast<byte>(d) - 1));
}

const byte maxHistory = 128;
char history[maxHistory];
int historyLast;  // int pour éviter débordement ...
const byte rangeHistory = 127; // absolute max value stored in a char
byte indexHistory = 0;

byte history0, historyM;
const float scaleHistoryInit = 10.0;
float scaleHistory = scaleHistoryInit;

//  ps :  .Près scaler. durée du watchdog
// #define ps bit(WDP2)   // 4   267.55 en moyenne
// #define ps bit(WDP2)| bit(WDP0)  // 5   535.07ms en moyenne
// #define ps bit(WDP2)| bit(WDP1)  // 6  1070.13ms en moyenne
// #define baseTime 1
// #define ps bit(WDP2)| bit(WDP1) | bit(WDP0) // 7  2140.15ms
// #define ps bit(WDP3) // 32  4280.04ms
#define ps bit(WDP3) | bit(WDP0) // 33 8559.5ms
#define baseTime 8   // durée en sec du watchdog
#define sleep_totalInit 1
int sleep_total = sleep_totalInit; // nombre de déclanchement du WD entre 2 échantillons de l'historique
#define LED_ON_TIME 200   // blink led quand on stocke un echantillon

float pOffset;
float pMin = 99999.0;
float pMax = 0.0;

float cumulPos = 0.0;
float cumulNeg = -1.0E-6;
float  altitudePrevious = -100;
long altitudePreviousCm;

#define cumulSeuil 1 // seuil difference altitude pour prendre en compte dans calcul cumul denivelé

unsigned long  countPodo; // compteur podomètre
unsigned long time;   // pour compteur temps podomètre
byte Hours, Minutes, Seconds;  // pour compteur temps podomètre

bool onoff = true; // for blink of ":" in hour display


//----------------------------------------------------------------
//  variable globale pour la gestion du timer watchdog et de la mise en sommeil en attente d'action sur le bouton
volatile int sleep_count = 0 ;
volatile boolean extInterrupt = false;   //external interrupt flag (button)

//--------------------------------------------------------
const uint8_t degre[] PROGMEM = {
  // (degr)  8x7.  Used for heading
  B00111000,
  B01111100,
  B11000110,
  B11000110,
  B11000110,
  B01111100,
  B00111000
};

const uint8_t weatherIcons[] PROGMEM = {
  // icons are 24 width, 16 hight, 48 bytes per icon.
  // 'soleil'
  0x01, 0x00, 0x00, 0x41, 0x04, 0x00, 0x21, 0x08, 0x00, 0x17, 0xd0, 0x00, 0x0f, 0xe0, 0x00, 0x1f,
  0xf0, 0x00, 0x1f, 0xf0, 0x00, 0xff, 0xfe, 0x00, 0x1f, 0xf0, 0x00, 0x1f, 0xf0, 0x00, 0x0f, 0xe0,
  0x00, 0x17, 0xd0, 0x00, 0x21, 0x08, 0x00, 0x41, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  // 'soleil_nuage'
  0x04, 0x00, 0x00, 0x44, 0x40, 0x00, 0x2e, 0x8f, 0x80, 0x1f, 0x3f, 0xc0, 0x3f, 0xbf, 0xc0, 0xff,
  0xff, 0xe0, 0x3f, 0xff, 0xfc, 0x1f, 0xff, 0xfe, 0x2f, 0xff, 0xfe, 0x47, 0xff, 0xff, 0x05, 0xff,
  0xff, 0x01, 0xff, 0xff, 0x01, 0xff, 0xff, 0x00, 0xff, 0xfe, 0x00, 0x7f, 0xe0, 0x00, 0x07, 0xc0,
  // 'pluie'
  0x00, 0xfc, 0x00, 0x03, 0xff, 0x00, 0x07, 0xff, 0x80, 0x07, 0xff, 0x80, 0x0f, 0xff, 0xc0, 0x3f,
  0xff, 0xfc, 0x7f, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0x19, 0x99, 0x99, 0x33, 0x33, 0x33, 0x66, 0x66, 0x66, 0xcc, 0xcc, 0xcc, 0x99, 0x99, 0x98
};
enum  : byte {
  SunSymbol = 0 ,
  SunCloudSymbol = 48,
  RainSymbol = 96  // begin address of the symbol in weatherIcons font
} Simpleweatherstatus;

/* ------------------------------------ setup ------------------------------------------ */

void setup()   {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  rightButton.releaseHandler(handleButtonReleaseEvents);
  leftButton.releaseHandler(handleButtonReleaseEvents);
  rightButton.holdHandler(handleButtonHoldEvents, 1500);  // 2000
  leftButton.holdHandler(handleButtonHoldEvents, 1500);


  if (EEPROM.read(0) != signatureAltimetre ) EEPROM.put(0, configEEPROM);
  EEPROM.get(0, configEEPROM);
  QNH = configEEPROM.QNH;
  currentAltitude = configEEPROM.currentAltitude;

  display.setFont(u8g_font_6x12r);   // assez voisin de font default gfx.  Hauteur corps principal 8
  // display.setFontPosBaseline();
  //  display.setColorIndex(1);

  display.firstPage();
  do {
    display.drawBitmapP(0, 0, 16, 64, Die_2); // personal splash screen
    display.setPrintPos(100, 56 + 8);
    display.print(configEEPROM.Version);
  } while ( display.nextPage() );

  //  if (!pressure.begin(i2cAddressBMP280) || mpu_init_load_firmware_EEPROM(i2cAddressEEPROM) )
  if (!pressure.begin(i2cAddressBMP280)) screenMessage(F("BMP fails"));;  // return TRUE if OK
  if ( mpu_init_load_firmware_EEPROM(i2cAddressEEPROM)) screenMessage(F("MPU fail"));; // return 0 if OK : FALSE
  pressure.setSampling(fs_BMP280::MODE_NORMAL,     /* Operating Mode. */
                       fs_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                       fs_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                       fs_BMP280::FILTER_X16,      /* Filtering. */
                       fs_BMP280::STANDBY_MS_1); /* Standby time. */
  configure_magnetometer();
  delay(100);
  pOffset = getPressureAverage(); // used for saving pressure history
  t0ScreenTimeOut = millis();  // init pour le timeout pour pour arreter le display et passer en mode sommeil
  sleep_count = 0;
  WDT_On(ps);// lancer  le watchdog pour gérer l'instant pour stocker un echantillon de l'historique
}

/* ------------------------------------ loop ------------------------------------------ */
void loop() {

  if (extInterrupt) {  // we have an external interrupt coming from one of the buttons on pin 2 & 3. Flag is  set by the ISR
    display.sleepOff();
    screenOff = false;
    extInterrupt = false;
    t0ScreenTimeOut = millis();  // reset timeout pour eteindre ecran
  }
  if (!screenOff) {
    rightButton.process();
    leftButton.process();
  }
  temperature = pressure.readTemperature();
  averagePressure = getPressureAverage(); // Get the current pressure from BMP280 sensor, in hpa, filtering done in the BMP280
  pression = averagePressure;
  computeHeading();
  // https://www.deleze.name/marcel/sec2/applmaths/pression-altitude/pression-altitude.pdf
  // QNH sealevel presure
  altitude = 44330 * (1.0 - pow(averagePressure / QNH, 0.1903));  // le calcul direct gagne de place par rapport à pressure.readAltitude
  //altitude = pressure.readAltitude(QNH);  //  44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
  if (sleep_count >= sleep_total) {  // si timer à echeance, stocker un echantillon dans historique/
    // sleep_count est incrementé à chaque déclanchement du watchdog
    if (!freezeHistory) {
      storeHistory( averagePressure);
    }
    noInterrupts();
    sleep_count = 0;  // relancer le timer de stockage
    interrupts();
  }
  if (stateSetting == settingNo) setAltiMinMax(); // ne pas mettre à jour si on est en état setting   +++++++++++++++++++++++++++  ????????????????

  // Si on est pas en mode setting et si on est en mode powerSaver et l'écran est ON et que le timeout t0ScreenTimeOut est échu
  // ou si l'écran est déja coupé (dans ce cas on a été réveillé par le WD:   retourner dormir.
  // Ne pas couper si on est dans etat setting

  if (( (stateSetting == settingNo) && powerSaver && !screenOff && (millis() - t0ScreenTimeOut) > screenTimeOut ) || screenOff) {
    screenOff = true;
    display.sleepOn();
    sleepBuntonPressed (); // Aller dormir. On revient du sleep sur action bouton ou time out watchdog stockage historique
    // retour du sleep
    longPush = true; // faire croire que on a eu un longPush pour s'affranchir des bounces qui font changer d'écran dans handleButtonReleaseEvents
  }
  else {
    if (stateSetting == settingNo) {
      switch (screen) {
        case AltitudeS:
          showScreen(F("ALTITUDE"), altitude, METER); // display the altitude as an integer. Round the value (don't trunc with only int() )
          break;
        case AltitudeMaxS:
          showScreen(F("ALTITUDE MAX"), altiMax, METER);
          break;
        case AltitudeMinS:
          showScreen(F("ALTITUDE MIN"), altiMin, METER);
          break;
        case PressureS:
          // P(z)= 1013.25 (1-0.0065*z/288.15)**5.255
          // simple weather forecast: comparison of the mesured pressure at altitude, and the pressure given by
          // the barometric formula at this altitude. More or less OK if altitude is good (use the setting screen !)
          // and does not change
          // Good information in the application note from Freescale: http://www.nxp.com/assets/documents/data/en/application-notes/AN3914.pdf
          Pweather = 1013.25 * pow((1 - 0.0065 * currentAltitude / 288.15), 5.255); // barometric formula
          Simpleweatherdiff = pression - Pweather;
          if (Simpleweatherdiff > 2.5) Simpleweatherstatus = SunSymbol;
          else if (Simpleweatherdiff < (-2.5)) Simpleweatherstatus = RainSymbol;
          //  if ((Simpleweatherdiff <= 2.5) || (Simpleweatherdiff >= (-2.5))) Simpleweatherstatus = SunCloudSymbol;
          else Simpleweatherstatus = SunCloudSymbol; //Sun/Cloud Symbol
          if (!pressureAbsolute ) showScreen(F("PRESSURE (s)"), pression / pow(1 - altitude / 44330.0, 5.255877), HPA);
          else showScreen(F("PRESSURE (a)"), pression, HPA);
          break;
        case TemperatureS:
          showScreen(F("TEMPERATURE"), temperature, DEG);
          break;
        case BatteryS:
          //  showScreen(F("BATTERY"), float(readVcc()) / 1000.0 , VOLT);
          showScreen(F("BATTERY"), float(readVcc()) / 100.0 , VOLT);
          break;
        case HistoryS:
          drawScreen(displayHistory);
          break;
        case GainLossS:
          drawScreen(displayCumul);
          break;
        case PedometerS:
          dmp_get_pedometer_step_count (&countPodo);
          dmp_get_pedometer_walk_time(&time);
          time = time / 1000;
          Hours = time / (60 * 60);
          Minutes = (time % (60 * 60)) / (60);
          //  Seconds = ((time % (60 * 60)) % (60));
          drawScreen(displayPodometre);
          break;
        case CompassS:
          drawScreen(displayCompass);
          break;
        case ClockS:
          DS3231_get(&rtcUnion);
          T1 = millis();
          if (T1 > T0 + 460 ) { // adjust to get more or less a tic blink per second ...
            onoff = !onoff;
            T0 = T1;
          }
          drawScreen(displayClock);
          break;
        default: ;
      }  // fin du switch (screen), cas settingNo
    }   // fin stateSetting == settingNo
    //  a switch for the following takes more place in memory ....
    else if (stateSetting == settingPowerSaverMode) {
      drawScreen(displaySettingSleep);
    }
    else if (stateSetting == settingPressureMode) {
      drawScreen(displaySettingPressure);
    }
    else if (stateSetting == settingFreezeHistoryMode) {
      drawScreen(displaySettingFreezeHistory);
    }
    else if (stateSetting == settingAltitudeModeIncr || stateSetting == settingAltitudeModeDecr ) {
      onoff = !onoff;
      drawScreen(displaySettingAltitude);
    }
    else if (stateSetting == settingCalibrateMode) {
      calibrate_magnetometer();
    }
    else if (stateSetting ==  settingClockHour || stateSetting == settingClockMinute ) {
      onoff = !onoff;
      drawScreen(displayClock);
    }
    // fin setting
  }
}

// Affiche les données d'un ecran
void showScreen(const __FlashStringHelper *label, float value, byte unit) {
  display.firstPage();
  do {
    rightButton.process();
    leftButton.process();
    displayTitleBattery(label);
    if (screen == PressureS) {
      display.drawBitmapP( 75, 0 , 3, 16, weatherIcons +  Simpleweatherstatus); // weather forecast on pressure screen
    }
    drawFloatValue(0, 20, value, unit);
  } while ( display.nextPage() );
}

void displayTitleBattery(const __FlashStringHelper *label) {
  display.setPrintPos(0, 0 + 8);
  display.print(label);
  showBatterylevel(readVcc());
}
void drawScreen(void (*draw)()) {
  display.firstPage();
  do {
    rightButton.process();
    leftButton.process();
    draw();
  } while ( display.nextPage() );
}

void displayCumul()   {
  displayTitleBattery(F("TOTAL GAIN / LOSS "));
  display.setScale2x2();
  display.setPrintPos(0, 10 + 8);
  display.print(F("+"));
  // const u8g_pgm_uint8_t string_0[] U8G_PROGMEM  = "First Line";
  //display.drawStrP(0,10+8,u8g_pgm_uint8_t("+"));
  display.print(cumulPos, 0);
  display_mSpace();  //  pour gain de place !!
  display.setPrintPos(0, 22 + 8);
  display.print(cumulNeg, 0);
  display_mSpace();  //  pour gain de place
  display.undoScale() ;
}

void displayPodometre() {
  displayTitleBattery(F("PEDOMETER "));
  display.setScale2x2();
  display.setPrintPos(0, 10 + 8);
  display.print(countPodo);
  display.setPrintPos(0, 30);
  display.print(Hours); display.print(F("h"));
  display.print(Minutes ); display.print(F("m"));
  display.undoScale() ;
}

void displayClock() {
  displayTitleBattery(F("CLOCK"));
  drawFloatValue(0, 20, rtcUnion.rtc.hour * 100 + rtcUnion.rtc.min, DEG);
}

void displaySettingAltitude() {
  showScreen(F("ALTI CALIBRATION"), baseAltitude, stateSetting == settingAltitudeModeIncr ? arrowUp : arrowDown);
}

void displaySettingSleep() {
  display.setPrintPos(0, 0 + 8);
  display.print(F("SLEEP MODE: "));
  display.setScale2x2();
  display.setPrintPos(0, 20 + 8);
  //  display.setPrintPos(0, 14);
  display.print (F("Sleep "));
  if (powerSaver) display.print(F("ON"));
  else display.print(F("OFF"));
  display.undoScale() ;
}

void displaySettingFreezeHistory() {
  display.setPrintPos(0, 0 + 8);
  display.print(F("HISTORY MODE: "));
  display.setScale2x2();
  display.setPrintPos(0, 20 + 8);
  //  display.setPrintPos(0, 14);
  display.print (F("Freeze "));
  if (freezeHistory) display.print(F("ON"));
  else display.print(F("OFF"));
  display.undoScale() ;
}

void displaySettingPressure() {
  display.setPrintPos(0, 0 + 8);
  display.print(F("PRESSURE MODE"));
  display.setScale2x2();
  display.setPrintPos(0, 20 + 8);
  //  display.setPrintPos(0, 14);
  if (!pressureAbsolute) display.print(F("Sea level"));
  else  display.print(F("Absolute"));
  display.undoScale() ;
}

float getPressureAverage() {
  // Get the current pressure from BMP280 sensor in hPa. Filtering done the BMP280
  pressionPa = pressure.readPressure();
  return  pressionPa / 100; // pour avoir en HPA comme sur BMP180
}

// Enregistre les altitudes Min & Max
void setAltiMinMax() {
  if (altitude > altiMax) altiMax = altitude;
  if (altitude < altiMin) altiMin = altitude;
  //altiMax = max(altiMax,altitude);
  //altiMin = min(altiMin,altitude);

  // calcul denivelés cumulés
  //int altitudeInt = round(altitude);
  float altitudeInt = altitude;
  // long altitudeIntCm = round(altitude * 100);
  if (altitudePrevious < 0 ) {
    altitudePrevious = altitudeInt;
    // altitudePreviousCm = altitudeIntCm;
  }
  float diff = altitudeInt - altitudePrevious;
  // int diffCm = altitudeIntCm - altitudePreviousCm;
  if (abs(diff) >= cumulSeuil) {
    //pressionPaPrevious = pressionPa;
    altitudePrevious = altitudeInt;
    //   altitudePreviousCm = altitudeIntCm;
    if (diff > 0 )
      cumulPos = cumulPos + diff;
    else
      cumulNeg = cumulNeg + diff;
  }
}
void resetAltiMinMax() {
  altiMax = altiMin = altitude;
  flashScreen();
}
// Gestion du bouton relaché
void handleButtonReleaseEvents(Button &btn) {
  t0ScreenTimeOut = millis();    // reset timeout activité bouton
  if (longPush) // ignore a release event from a long pushT
    longPush = false;
  else
    switch (stateSetting) {
      case settingAltitudeModeIncr:
        if (btn == rightButton) baseAltitude =   baseAltitude + 1;
        else baseAltitude = baseAltitude + 10;
        // recalculer le QNH
        QNH = pression / pow(1 - baseAltitude / 44330.0, 5.255877);
        break;
      case settingAltitudeModeDecr:
        if (btn == rightButton) baseAltitude =   baseAltitude - 1;
        else baseAltitude = baseAltitude - 10;
        // recalculer le QNH
        QNH = pression / pow(1 - baseAltitude / 44330.0, 5.255877);
        break;
      case settingPowerSaverMode:
        powerSaver = !powerSaver;  // rightButton or leftButton
        break;
      case settingPressureMode:
        pressureAbsolute = !pressureAbsolute;
        break;
      case settingFreezeHistoryMode:
        freezeHistory = !freezeHistory;
        break;
      case settingCalibrateMode://do nothing. Ignore release event during calibrate.
        break;
      case settingWaitingForSave:
        stateSetting = settingSaveCalibration;
        break;
      case settingClockHour:
        if (btn == rightButton) rtcUnion.rtc.hour = (rtcUnion.rtc.hour == 23 ? 0 : rtcUnion.rtc.hour + 1);
        else rtcUnion.rtc.hour = (rtcUnion.rtc.hour == 0 ? 23 : rtcUnion.rtc.hour - 1);
        break;
      case settingClockMinute:
        if (btn == rightButton) rtcUnion.rtc.min = (rtcUnion.rtc.min == 59 ? 0 : rtcUnion.rtc.min + 1);
        else rtcUnion.rtc.min = (rtcUnion.rtc.min == 0 ? 59 : rtcUnion.rtc.min - 1);
        break;
      default:
        if ( btn == rightButton) screen++;  // next screen (wrapp handled by the overloaded operator ++)
        else    screen--;  // previous screen
        break;
    }  // fin switch (stateSetting)
}

// Gestion de l'appui prolongé sur le bouton
void handleButtonHoldEvents(Button & btn) {
  t0ScreenTimeOut = millis();
  longPush = true;
  switch (screen) {
    case AltitudeS:
      if (stateSetting == settingNo) {
        stateSetting = settingAltitudeModeIncr;
        baseAltitude = round(altitude);
      }
      else if (stateSetting == settingAltitudeModeIncr) stateSetting = settingAltitudeModeDecr;
      else {
        stateSetting = settingNo;  // sortie de l'état de Settings
        // Save QNH in EEPROM
        currentAltitude = baseAltitude; // used for weather forecast. We supose that the altimeter will stay at this altitude
        configEEPROM.QNH = QNH;
        configEEPROM.currentAltitude = currentAltitude;
        EEPROM.put(0, configEEPROM);
        altitudePrevious = -100;  // pour ne pas influencer le calcul du denivelé
        baseAltitude = 0;  // forcer nouvelle initialisation de baseAltitude
      }
      break;
    case AltitudeMaxS:
    case AltitudeMinS:
    case HistoryS:
      resetAltiMinMax();
      resetHistory();
      break;
    case PressureS: //Pression.  Choice absolute / sea level
      if (stateSetting == settingNo) stateSetting = settingPressureMode;
      else stateSetting = settingNo;
      break;
    case TemperatureS: // Temperature. Choice sleep mode on/off and then Freeze history on/off
      if (stateSetting == settingNo) stateSetting = settingPowerSaverMode;
      else if (stateSetting == settingPowerSaverMode) stateSetting = settingFreezeHistoryMode;
      else stateSetting = settingNo;
      break;
    case BatteryS: // screen  battery. Choice freezeHitory true/false
      if (stateSetting == settingNo) stateSetting = settingFreezeHistoryMode;
      else stateSetting = settingNo;
      break;
    case GainLossS: // écran cumul. RAZ des cumuls
      cumulPos = 0.0;
      cumulNeg = -1.0E-6;
      flashScreen();
      altitudePrevious = -100;
      break;
    case PedometerS: // écran podomètre. RAZ du count
      dmp_set_pedometer_step_count(0);
      dmp_set_pedometer_walk_time(0);  // ou option séparée ....
      flashScreen();
      break;
    case CompassS:
      if (stateSetting == settingNo) stateSetting = settingCalibrateMode;
      else stateSetting = settingNo;
      break;

    case ClockS:
      if (stateSetting == settingNo) stateSetting = settingClockHour;
      else if ( stateSetting == settingClockHour) stateSetting = settingClockMinute;
      else {
        DS3231_set(&rtcUnion);
        stateSetting = settingNo;
      }
  }  // fin switch(screen)
}

// Affiche un caractére en x, y
// fw and Fh in pixel
void drawCar(byte sx, byte sy, byte num, const uint8_t *font, byte fw, byte fh) {
  display.drawBitmapP(sx, sy, fw / 8, fh, font + num * (fh * fw / 8));
}

// Affiche un symbole en x, y
void drawSymbol(byte sx, byte sy, byte num) {
  drawCar(sx, sy, num, Symbol, 16, 12) ;
}

// display a floating point value, with a big font
// Nice only with the local font Font24x40 ....
// will draw nothing if val > 10000 or < -999
// Print one digit after decimal point for value <=999.9,  but don't print ".0" (simulate integer ...)
// On a screen we have a max of 4 places for decimal (or minus sign), and a dot. 999.9  or 9999 or -999
// Draw also in hh:mm format for the clock, and blink the ":"  ...;
// IT's the mess .....
void drawFloatValue(int sx, int sy, float val, byte unit) {
  char charBuf[7] = "------";  // overflow code
  byte nbcar = 4;
  if (val < 10000 && val > -999.0)  nbcar = my_dtostrf(val, charBuf, ' ');  // nbcar: number of character before the decimal point
  //  Standard Arduino dtostrf is too big  (more than 1.5K ...).

  // If more than 5 characters generated, we have a value greater than 999.9
  // Only print the first 4 and stop before the decimal .
  byte NBcar = 6;
  byte debut = 1;
  if (nbcar > 3 || screen == ClockS) {
    NBcar = 4;
    debut = 0;
  }
  for (byte i = debut; i < NBcar; i++) {
    char carac = charBuf[i];
    if (screen == ClockS && carac == ' ') carac = '0'; // left padding for clock for the case x:yz. We want 0x:yz
    if ((i == 2) && (screen == ClockS)) {  // need the ":" for clock ??
      // Blink the ":" if not setting clock
      if (((stateSetting == settingNo ) && onoff ) || (stateSetting != settingNo)) {
        display.drawBox(sx, sy + 39 - 6, 6, 6);
        display.drawBox(sx, sy + 25 - 6, 6, 6);
      }
      sx = sx + 6 + 2;
    }
    // if setting the clock:  blink a line below the hours or minutes
    if (i < 2 && stateSetting == settingClockHour && onoff)
      display.drawBox(0, sy + 42, 50, 6);
    else if   (i >= 2 && stateSetting == settingClockMinute && onoff )
      display.drawBox(0 + 60, sy + 42, 50, 6);
    if ( carac == '.' ) {
      display.drawBox(sx, sy + 39 - 6, 6, 6);
      sx = sx + 6 + 2;
    } else
    {
      if (carac == '-')
        display.drawBox(sx, sy + 17, 22, 6);
      else  if (carac != ' ') drawCar(sx , sy, carac - '0', Font24x40, 24, 40);
      sx = sx + 24 + 2;
    }
  }
  // if setting alttitude, draw and blink a long line below the nbr
  if ((stateSetting == settingAltitudeModeIncr || stateSetting == settingAltitudeModeDecr ) && onoff)
    display.drawBox(0, sy + 42, sx - 0, 6);
  // Draw the unit symbol and in case of altitude setting blink the up/down arrow
  if ( !(screen  == ClockS || (stateSetting == settingAltitudeModeIncr || stateSetting == settingAltitudeModeDecr ) && onoff))
    drawSymbol(sx, sy, unit);
}

/*
   Convert a float to a string of character.
   1 digit after decimal point. Fixed format
   OK for values from -999.9 to 9999.9.  6 character array  (7 with the end of string)
   Any padding on the left character may be used, but in case the padding is not
   a space the result is a bit strange for negative number

*/
byte my_dtostrf( float  val, char charBuf[], char padding) {
  //
  int val_int, val_fra;
  byte nbcar = 0;
  float valRound = abs(val) + 0.05;
  val_int = (int) valRound;   // compute the integer part of the float
  val_fra = (int) ((valRound - (float)val_int) * 10);   // compute 1 decimal
  // convertir partie entière. 4 caractères max
  for (int8_t i = 3; i >= 0; i--) {  // i type signé !!
    if (val_int == 0 && i != 3 ) {
      charBuf[i] = padding;
      continue;
    }
    else charBuf[i] = val_int % 10 + '0';
    nbcar++;
    val_int = val_int / 10; //
  }
  if (val < 0) {
    charBuf[3 - nbcar] = '-'; // ugly if padding with 0
    nbcar++;
  }
  charBuf[4] = '.';
  charBuf[5] =  val_fra + '0';
  charBuf[6] = 0; // end of string
  return nbcar;
}



// Show battery level icon
void showBatterylevel(int vcc) {
  /*  When ProMini is powered by the lipo 1S directly on VCC
    byte logoOffset = 84;  // each logo has a size of 21 bytes. Init as empty (vcc < 3000)
    if (vcc > 3600) logoOffset = 0;
    else if (vcc > 3400) logoOffset =  21;
    else if (vcc > 3200) logoOffset = 42;
    else if (vcc > 3000) logoOffset = 63;
  */
  //  When ProMini is powered by the lipo 1S directly on RAW
  byte logoOffset = 84;  // each logo has a size of 21 bytes. Init as empty (vcc < 3000)
  if (vcc >=  3250 )  logoOffset =  0;
  else if (vcc >= 3100 ) logoOffset =  21;
  else if (vcc >= 3050) logoOffset = 42;
  else if (vcc >= 3000) logoOffset = 63;
  display.drawBitmapP(104, 1, 3, 7, Battery + logoOffset);

}
// Read VCC en mV
int readVcc() {
  //  long readVcc() {
  long result; // laisser "long" car overflow possible dans le calcul
  // Read 1.1V reference against AVcc
  const long InternalReferenceVoltage = 1083;  //trouvé sur Arduino Pro Mni 3V (1082 sur 5V ...)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  //  result = 1126400L / result; // Back-calculate AVcc in mV (1024*1100=1126400)
  result = (InternalReferenceVoltage * 1024) / result; // Back-calculate AVcc in mV
  return result;
}

void  resetHistory() {
  indexHistory = 0;
  pOffset = pMin = pMax = getPressureAverage();
  scaleHistory = scaleHistoryInit;
  sleep_total = sleep_totalInit;
  noInterrupts();
  sleep_count = 0;  // relancer le timer de stockage
  interrupts();
}

void storeHistory(float value) {
  // blink();
  if (indexHistory == maxHistory ) { // historique plein: le retasser en gardant 1 echantillon sur 2
    for (byte i = 0; i < maxHistory / 2; i++) {
      history[i] = history[2 * i];
    }
    indexHistory = maxHistory / 2;
    sleep_total = sleep_total * 2; // multiplier par 2 le temps entre echantillons
    noInterrupts();
    sleep_count = 0;
    interrupts();
  }

  /*
    on stocke la difference de pression par rapport à pOffset.
    Dynamique de -127 à +128 hpa soit environ +1000m / -1000m
    Quand débordement on fait varier le facteur d'échelle
  */
  historyLast = (value - pOffset) * scaleHistory;
  if (abs(historyLast) >= rangeHistory) { // debordement. Changer l'échelle
    for (byte i = 0; i < indexHistory; i++) {
      history[i] = history[i] / 2;
    }
    scaleHistory = scaleHistory / 2;
    historyLast = historyLast / 2;
  }
  history[indexHistory++] = historyLast;
  if (value > pMax) pMax = value;
  if (value < pMin) pMin = value;
}

void displayHistory() {

  // on trace l'historique des presssions
  display.setPrintPos(0, 0 + 8);
  display.print(F("HISTORY    (SP "));
  display.print(sleep_total * baseTime); display.print(F("s)"));
  char previousJ = -1;
  byte historyIprevious;
  for (byte i = 0; i < indexHistory; i++) {  // échantillons valides de 0 à indexHistory-1
    // l'echantillon de rang i sera mappé sur la colonne j de l'écran
    byte j = map (i, 0, indexHistory - 1, 0, 127);  // repartir les valeurs disponibles sur toute la largeur de l'ecran
    if (indexHistory == 1)j = 0; // si 1 seul point stocké forcer le mapping (map donne -1 dans ce cas ....)
    // mapper les valeurs stockées sur 0 (pmax) à 48  (pmin).     47=63-16+   on commence à la ligne 16
    byte historyI = map ( history[i ], (pMin - pOffset) * scaleHistory, ( pMax - pOffset) * scaleHistory, 47, 0);
    if (i == 0) history0 = historyI; // remember the first point for drawing the left horizontal ticmarks (no INVERSE in u8glib ....)
    // tracer des lignes verticales entre previousJ et J
    for (byte x = previousJ + 1; x <= j; x++) {
      if (x == 0) historyIprevious = historyI;
      byte y = map(x, previousJ, j, historyIprevious, historyI);
      //   display.drawFastVLine(x, 63 - y, y + 1, WHITE);
      display.drawVLine(x, 63 - y, y + 1);
    }
    previousJ = j;
    historyIprevious = historyI;
  }
  historyM = historyIprevious;
  drawTicsScale();
}

void drawTicsScale() {
  // No "INVERSE" color in u8glib, such as in ADAFRUIT GFX .... Try to guess for the horizontal tics marks using history0 and .Vertical tics are always clear.
#define nbrHTics 3
#define nbrVTics 3
#define ticLength 5

  byte step = 128 / (nbrHTics + 1);
  display.setColorIndex(0);
  for (byte x = step; x < 127; x = x + step) {
    //  display.drawFastVLine(x, 63 - (ticLength - 1), ticLength, INVERSE);   //  for ADAFRUIT GFX
    display.drawVLine(x, 63 - (ticLength - 1), ticLength);
  }
  display.setColorIndex(1);
  step = (64 - 16) / (nbrVTics + 1);
  for (byte y =  16; y < 63; y = y + step) {
    //   display.drawFastHLine(0, y, ticLength, INVERSE);
    //  display.drawFastHLine(127 - (ticLength - 1), y, ticLength, INVERSE);
    if ((63 - history0) < y ) display.setColorIndex(0);
    display.drawHLine(0, y, ticLength);
    display.setColorIndex(1);
    if ((63 - historyM) < y ) display.setColorIndex(0);
    display.drawHLine(127 - (ticLength - 1), y, ticLength);
    display.setColorIndex(1);
  }
  float aMax = 44330 * (1.0 - pow(pMin / QNH, 0.1903));
  float aMin = 44330 * (1.0 - pow(pMax / QNH, 0.1903));
  long seconds = long((indexHistory - 1) * sleep_total * baseTime) / (nbrHTics + 1);
  byte Hours = seconds / (60 * 60);
  byte Minutes = (seconds % ( 60 * 60)) / (60);
  byte Seconds = ((seconds % ( 60 * 60)) % (60));
  display.setPrintPos(0, 8 + 8); // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  display.print(F("Scale "));
  if (Hours == 0) {
    if (Minutes != 0) {
      display.print(Minutes); display.print(F("m"));
    }
    display.print(Seconds); display.print(F("s"));
  } else {
    display.print(Hours); display.print(F("h"));
    display.print(Minutes + Seconds / 30); display.print(F("m")); // round up if Seconds>30
  }

  display.print(F(" ")); display.print((aMax - aMin) / (nbrVTics + 1), 0);
  // display.print(F("m "));
  display_mSpace();  //   pour gain de place !!
  display.print(indexHistory); // For debug number+1 of samples in the history

}

void display_mSpace() {
  display.print(F("m ")); // pour gain de place !!
}

void blink() {
  digitalWrite(13, HIGH);
  delay(LED_ON_TIME);
  digitalWrite(13, LOW);
}


void computeHeading()
{
  // ----- Read the magnetometer
  read_magnetometer();  // du style : Mag_x = (mag_x corrigée ASAX - Mag_x_offset) * Mag_x_scale;

  // ----- Dampen any data fluctuations. Exponential smoothing 
  for (byte i = 0; i <= 2; i++) Mag_xyz_dampened[i] = Mag_xyz_dampened[i]  * 0.8 + Mag_xyz[i] * 0.2;

  // ----- Calculate the heading
  //  Heading = atan2(Mag_x_dampened, Mag_y_dampened) * RAD_TO_DEG;  // Magnetic North
  // With the wiring used in the TicTac box, X axis is toward the front and Y axis is toward the left.
  // So we must change the sign of the Y component to compute the heading.
  Heading = atan2(Mag_xyz_dampened[1], Mag_xyz_dampened[0]) * RAD_TO_DEG;  // Magnetic North
  /*
     By convention, declination is positive when magnetic north
     is east of true north, and negative when it is to the west.
  */

  //  Heading += Declination;               // Geographic North
  if (Heading > 360.0) Heading -= 360.0;
  if (Heading < 0.0) Heading += 360.0;

  // ----- Allow for under/overflow
  if (Heading < 0) Heading += 360;
  if (Heading >= 360) Heading -= 360;
}

void displayCompass()
{
  const int  r =  24;
  const int cx = 82;
  // const int cx = 127 - r;
  const int   cy = 40;
  const int   xHeading =  3;
  const int   yHeading  = 25;
  // ----- Display Heading, Pitch, and Roll
  // We draw the compas itself, with the needle toward the magnetic north
  displayTitleBattery(F("COMPASS / HEADING"));
  float delta = Heading * DEG_TO_RAD;
  display.drawLine(cx , cy, cx - r * sin(delta ), cy - r * cos(delta) );
  ticFour ( 0  + delta ,  cx,  cy, r, 10);
  ticFour ( PI / 4 + delta,  cx,  cy, r, 5);
  ticFour ( PI / 8 + delta , cx,  cy, r, 2);
  ticFour ( 3 * PI / 8 + delta, cx,  cy, r, 2);
  byte xH = xHeading;
  int intHeading = int(Heading);
  if (intHeading < 100) xH = xH + 6 ; // font u8g_font_6x12r.  Space padding.
  if (intHeading < 10) xH = xH + 6;
  display.setScale2x2();
  display.setPrintPos(xH, yHeading);
  display.print(intHeading);
  display.undoScale();
  display.drawBitmapP(42, yHeading + 4 , 1, 7, degre); // ° symbol

}
void ticFour ( float startAngle, int cx, int cy, int r, int lg)
{
  for (byte i = 0; i < 4; i++)
  {
    //  display.drawLine(cx + (r - lg)*cos(startAngle), cy + (r - lg)*sin(startAngle), cx + r * cos(startAngle ), cy + r * sin(startAngle));
    display.drawLine(cx + (r - lg)*sin(startAngle), cy + (r - lg)*cos(startAngle), cx + r * sin(startAngle ), cy + r * cos(startAngle));
    // aiguille display.drawLine(cx , cy, cx + r * sin((Heading + 90) * DEG_TO_RAD ), cy + r * cos((Heading + 90) * DEG_TO_RAD ));

    startAngle = startAngle + PI / 2;
  }
}

// -------------------------------
//  Read magnetometer
// Set values in global variables:
//  mag_x, mag_y, mag_z: raw values from AK8963 register, with ASAX/Y/Z corrections
//  Mag_x, ...  : values with full correction (ASA, Mag_x_offset, Mag_x_scale
// If new data are not ready or if overflow:  no change done.
// -------------------------------
void read_magnetometer()
{
  byte status_reg_2;
  byte status_reg_1;
  byte buf[7];
  // ----- Point to status register 1
  I2Cdev::readByte(AK8963_I2C_address, AK8963_status_reg_1, &status_reg_1);
  if (status_reg_1 & AK8963_data_ready_mask)                     // Check data ready bit
  {
    // ----- Read data from each axis (LSB,MSB) and status reg 2) (it will be cleared)
    I2Cdev::readBytes(AK8963_I2C_address, AK8963_data, 7, buf);                  // Request 7 data bytes
    for (byte i = 0; i <= 2; i++)  mag_xyz[i] = (buf[2 * i] | buf[2 * i + 1] << 8) * ASA[i]; // Combine LSB,MSB X-axis, apply ASA corrections
    status_reg_2 = buf[6];                                 // Read status and signal data read

    // ----- Validate data
    if (!(status_reg_2 & AK8963_overflow_mask))                 // Check HOFL flag in ST2[3]
    {
      for (byte i = 0; i <= 2; i++) Mag_xyz[i] = (mag_xyz[i] - configEEPROM.Mag_xyz_offset[i]) * configEEPROM.Mag_xyz_scale[i];
    }
    else {  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      screenMessage (F("Ovfl mag"));
    }
  }
}
void configure_magnetometer()
{

  /*
     The MPU-9250 contains an AK8963 magnetometer and an
     MPU-6050 gyro/accelerometer within the same package.

     To access the AK8963 magnetometer chip The MPU-9250 I2C bus
     must be changed to pass-though mode. To do this we must:
      - disable the MPU-9250 slave I2C and
      - enable the MPU-9250 interface bypass mux
  */

  // ----- Enable MPU9250 interface bypass mux
  I2Cdev::writeByte(MPU9250_I2C_address, MPU9250_Interface_bypass_mux_enable, 0x02);
  delay(100);
  // ----- Access AK8963 fuse ROM
  /* The factory sensitivity readings for the XYZ axes are stored in a fuse ROM.
     To access this data we must change the AK9863 operating mode.
  */
  I2Cdev::writeByte(AK8963_I2C_address, AK8963_cntrl_reg_1, 0b00011111); // Output data=16-bits; Access fuse ROM
  delay(100);                                                       // Wait for mode change


  // ----- Get factory XYZ sensitivity adjustment values from fuse ROM
  /* There is a formula on page 53 of "MPU-9250, Register Map and Decriptions, Revision 1.4":
      Hadj = H*(((ASA-128)*0.5)/128)+1 where
      H    = measurement data output from data register
      ASA  = sensitivity adjustment value (from fuse ROM)
      Hadj = adjusted measurement data (after applying
  */

  byte buf[3];
  I2Cdev::readBytes(AK8963_I2C_address, AK8963_fuse_ROM, 3, buf);
  for (byte i = 0; i <= 2; i++) ASA[i] = (buf[i] - 128) * 0.5 / 128 + 1;

  // ----- Set output to mode 2 (16-bit, 100Hz continuous)
  // Output=16-bits; Measurements = 100Hz continuous
  I2Cdev::writeByte(AK8963_I2C_address, AK8963_cntrl_reg_1, 0b00010110);
  delay(100);                                                       // Wait for mode change
}
// -------------------------------
//  Calibrate magnetometer
//   The algorithm for calibrating the magnetometer is
//   described in the following article by Kris Winer:
//   https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
// -------------------------------
void calibrate_magnetometer()
{
  const byte ymes = 23;
  const byte ybar = 40;
  const byte newLine = 10;
  const int sample_count = 2500 ;


  // ----- Locals
  int mag_xyz_max[3] = { -32768, -32768, -32768};
  int mag_xyz_min[3] = {32767, 32767, 32767};                                       // Raw data extremes
  float chord_xyz[3];   //float chord_x,  chord_y,  chord_z;                              // Used for calculating scale factors
  float chord_average;
  // ----- Record min/max XYZ compass readings
  for (int counter = 0; counter < sample_count ; counter ++)             // Run this code 16000 times
  {
    rightButton.process();
    leftButton.process();
    if (stateSetting == settingNo) return;  // allow abort on a longpress
    if (counter % 100 == 0)                       //
    {
      display.firstPage();
      do {
        display.setPrintPos(0, 0 + 8);
        //       display.print(F("COMPASS CALIBRATION"));  // ++++++++++++++++++++
        display.print(F("CALIBRATION"));
        display.setPrintPos(0, ymes);
        //      display.print(F("Rotate the device")); //+++++++++++++++++
        display.print(F("Rotate Tiny Hiker"));
        display.setPrintPos(10, ymes + newLine);
        display.print(F("on a flat surface"));
        //       display.setPrintPos(10, ymes + 2 * newLine);
        //        display.print(F("until done")) ;
        display.drawFrame(0, ybar, 128, 10);
        display.drawBox(0, ybar + 1, map(counter, 0, sample_count, 0, 128), 8 );
        display.setPrintPos(0, ybar + 2 * newLine);
        display.print(F("Long press to cancel"));
      } while ( display.nextPage() );
    }
    read_magnetometer() ;
    // ----- Find max/min xyz values
    for (byte i = 0; i <= 2; i++)
    {
      mag_xyz_min[i] = min(mag_xyz[i], mag_xyz_min[i]);
      mag_xyz_max[i] = max(mag_xyz[i], mag_xyz_max[i]);
    }
    delay(12); // at 100 Hz ODR, new mag data is available every 10 ms  +++++++++++++++++++++++++++++
  }

  for (byte i = 0; i <= 2; i++)
  {
    // ----- Calculate hard-iron offsets
    configEEPROM.Mag_xyz_offset[i] = (mag_xyz_max[i] + mag_xyz_min[i]) / 2;                     // Get average magnetic bias in counts
    chord_xyz[i] = ((float)(mag_xyz_max[i] - mag_xyz_min[i])) / 2;                 // Get average max chord length
  }
  // ----- Calculate soft-iron scale factors
  chord_average = (chord_xyz[0] + chord_xyz[1] + chord_xyz[2]) / 3;              // Calculate average chord length
  for (byte i = 0; i <= 2; i++)  configEEPROM.Mag_xyz_scale[i] = chord_average / chord_xyz[i];                          // Calculate X scale factor
  display.firstPage();
  do {
    display.setPrintPos(0, 10);
    display.print(F("Long press to cancel")) ;
    display.setPrintPos(0, ymes);
    display.print(F("Mag_xyz_offset:"));
    display.setPrintPos(10, ymes + newLine);
    for (byte i = 0; i <= 2; i++) {
      display.print( configEEPROM.Mag_xyz_offset[i]); display.print(F(" "));
    }
    display.setPrintPos(0, ymes + 2 * newLine);
    display.print(F("Mag_xyz_scale:"));
    display.setPrintPos(10, ymes + 3 * newLine);
    for (byte i = 0; i <= 2; i++) {
      display.print( configEEPROM.Mag_xyz_scale[i]); display.print(F(" "));
    }
    display.setPrintPos(0, ymes + 4 * newLine);
    display.print(F("Short press to save")) ;
  } while ( display.nextPage() );
  stateSetting = settingWaitingForSave;
  do {
    rightButton.process();
    leftButton.process();
  } while ( stateSetting == settingWaitingForSave) ; // we may have settingNo if abort (result of long push) or  settingSaveCalibration(result of clik)
  if (stateSetting == settingSaveCalibration) {
    EEPROM.put(0, configEEPROM);
    //  EEPROM.get(0, configEEPROM); // for debug. Never store the new value and relaod the old ones +++++++++++++++++++++++++++++++++++++++
    flashScreen(2);
  }
  else {
    EEPROM.get(0, configEEPROM); // abort .restor the calibration and bias to the original values.
  }
  stateSetting = settingNo;
}

void flashScreen(byte count = 1) {
  for (byte i = 1; i <= count; i++) {
    display.sleepOn();
    delay(100);
    display.sleepOff();
    delay(200);
  }
}


void screenMessage(   const __FlashStringHelper *message ) {
  display.sleepOff();
  display.firstPage();
  do {
    display.setPrintPos(0, 20 + 8);
    display.print(message);
  } while ( display.nextPage() );
  delay (10000);
}
