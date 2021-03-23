# Tiny_Hiker
TinyHiker, an Arduino based altimeter, compass, pedometer, and clock.
TinyHiker is a small altimeter including also a compass and a pedometer. It uses an OLED display and fit in a small TicTac box.
![](/img/schema.JPG)
<img src="/img/TicTac.JPG" width="300"><img src="/img/DSC04059.JPG" width="300">
# Main features.

TinyHiker uses a pressure/temperature sensor for altitude and temperature measurement, a 9DOF- (Gyro + Accelerometer + Magnetometer) for pedometer, a magnetometer for the compass, and a real time accurate clock chip. 
The user interface includes a set of screens showing
-	Current altitude
-	Maximum altitude reached
-	Minimum altitude reached
-	Atmospheric pressure
-	Temperature
-	Graphical history of altitude
-	Cumulative elevation gain/loss
-	Number of steps and walk time
-	Graphical compass and heading
-	Accurate clock
-	Battery level icons
-	 Weather forecast icons
-	
Sleep mode of the Arduino, combined with power management of the OLED display give a good autonomy to the design.
At the very beginning, this project was inspired from Olivier Gaillard’s TinyAltimeter. ( https://keeptronic.wordpress.com/2017/01/09/tiny-altimeter/)
The challenge was to fit all this stuff in the memory of the Arduino and in the TicTac box  ;-)

# Parts needed:
-	Arduino ProMini ATMEGA328P 3v / 8Mhz
-	A combo module featuring a MPU9250 Gyro + Accelerometer + Magnetometer and a BMP280 pressure sensor
-	OLED LCD Display 128x64  0.96" I2C
-	RTC clock module DS3231
-	On/Off Switch
-	2 push buttons
-	EEPROM with I2C interface: at least 4kb. AT24C32 or bigger
-	LiPo 1S 150/180mAh
-	Micro USB LiPo Charger Module with Protection (including over discharge protection if the LiPo cell does not include one)
-	**TicTac box  ;-)**

Here are some pictures of the components used in this project.
Sometimes several boards with more or less the same functionalities exist, but keep in mind that the TicTac box is small!!

 

### Arduino ProMini

Classical Atmega358P 3.3v 8Mhz




 

### MPU9250 + BMP280

I used this combo board including a MPU9250 and a BMP280. This is a 3.3v board, with no regulator nor level shifter. The brand is CJMCU (?)
The MPU9250 chip features a MPU6500 3-Axis Gyro/3-Axis Accelerometer and an AK8963 3-Axis magnetometer
There is a more common board with voltage regulator, level shifters, working at 3/5v, but it is a bit bigger and I didn't manage to have it working nicely with the 3.3v Arduino ProMini in my prototype.
 
### Serial I2C EEPROM
The MPU6500 includes an internal Digital Motion Processing™ (DMP™) engine that performs complex 3D Motion Processing and offers also a low-power pedometer functionality allowing the host processor to sleep while the DMP maintains the step count and the walk time. This feature is always enabled as long as the DMP is enabled. The drawback is that the DMP must be loaded at each power up by the host processor with a 3kb specific microcode. In order not to waste the memory space of the Arduino, the microcode for the DMP is stored in an EEPROM preloaded with the code. A 4kb I2C EEPROM such as an AT24C32 is OK.


 
### I2C 0.96 inch OLED 128X64 OLED Display Module
The I2C OLED display comes in a double color yellow/blue version of this OLED display gives a nice looking to the project: the headers of the screens are in yellow, and the main information in blue. It is also better to use a pure I2C version, even if some SPI version can be configured as I2C but with extra wiring.
 


### Tiny Real Time Clock Module RTC DS3231
It usually comes with the header already in place, but better to remove it for our setup.
There is also a well-known DS3231 RTC module that includes also an AT24C32 (4kb) EEPROM. This module is perfect for a prototype on a breadboard because we need an EEPROM for the microcode of the MPU9250 DMP, but really too big to fit in our TicTac box 
### Switches
 
The switches used in this project  …

 
### LiPo
I used this kind of LiPo cell. It doesn't include protection cell and so you need a charge/discharge LiPo control module. You can find a bit bigger ones that will fit in the TicTac box, some without connector and with protection cell
 
### LiPo charger
If your LiPo cell includes an over discharge protection system than you my use a simpler module but in any case these modules usually give a 1A charge current and it is too much for a small LiPo such as the one used in this project. You must change a resistor R3. Original value is usually 1.2KΩ but better to use a 10kΩ one, giving around 130mA charging current. Detailed information is readily available on the web for TP4056 based charger

### The TicTac box 
  
Size 62x37x14mm
Any flavor is OK 




## Technical implementation:

## Software:

Main libraries used in the project:

**Library wire and EEPROM** are the standard libraries of the IDE 
**I2Cdev**	from Jeff Rowberg. 
I2Cdev is a part of the I2C Device Library (i2cdevlib), a collection of uniform and well-documented classes to provide simple and intuitive interfaces to I2C devices. Each device is built to make use of the generic "I2Cdev" class, which abstracts the I2C bit- and byte-level communication away from each specific device class. We just need here this library I2Cdev
1.	Download the entire contents of the i2cdevlib repository: https://github.com/jrowberg/i2cdevlib/archive/master.zip
2.	Unzip the downloaded file named i2cdevlib-master.zip
3.	(In the Arduino IDE) Sketch > Include Library > Add .ZIP Library > select the folder i2cdevlib-master/Arduino/I2Cdev > Open
(The "Add .ZIP Library" command can also be used to install libraries from folders as well as .zip files)

**Button** We use here the version from  https://github.com/t3db0t/Button 
U8glib	The great graphics library from Oliver Kraus https://github.com/olikraus/u8glib . This library can be installed from the library manager in Arduino IDE. (Last version 1.19.1 used)


The project itself is composed of sketches:
- Tiny_Hiker_EEPROM_init.ino	used for the initialization of the EEPROM  that contains the microcode for the DMP embedded in the MPU9250. It must be executed only once

- Tiny_Hiker.ino		the altimeter/compass itself

 Just unzip the archives file in the corresponding directory.

**Power supply.**
The specifications of the MPU9250 gives VCC  between 2.4v and 3.6v and even if others components used in the project support higher voltage, we must power Arduino Promini on the RAW pin and use the VCC pin (3.3v)  for the others parts. (Remember that a fully charged LiPo will give around 4.2v). The drawback is that very often the Lipo will be at 3.7v and the 3.3v ldo regulator on the ProMini will not work in very good conditions … Maybe be a step-up module could help but we don't have a lot of place in the TicTac box.


**Remarks:**
The I2C address of the RTC DS3231 is fixed: 0x68. The default address for the MPU9250 on the combo board is also 0x68 if the A0 pin is left unconnected.(there is a pull down resistor on the module ). So we must set AD0 pin high in order to get address 0x69 and avoid conflict. This will set also the BMP180address to 0x77.
With the modules and the wiring we used, we have the following I2C address map:
 - 0x 3C	OLED 
 - 0x 50	EEPROM microcode 24L256 
 - 0x 68	RTC  DS3231 
 - 0x 69	MPU9250    (MPU6500 + magnetometer) 
 - 0x C	magnetometer  (onlyvisible on the bus if the MPU is in  bypass) 
 - 0x 77	BMP280 





