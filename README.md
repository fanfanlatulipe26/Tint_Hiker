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
