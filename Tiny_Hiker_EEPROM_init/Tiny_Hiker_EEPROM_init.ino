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
  Initialisation EEPROM I2C avec le microcode MPU6050
  Microcode de la version 5..3, venant du package officiel Invensence
  Le microcode est stocké à partir de l'adresse 0

  Ecriture et vérification byte à byte ......
 Verifier que le fichier a_mpi_dmp_mini_podometre.h est à jour .....
*/
#include <Wire.h>
#include <I2Cdev.h>
#include "tiny_hiker.h"
#include "a_mpu_dmp_mini_podometre.h"

void setup() {
  Serial.begin(9600);
  Serial.println("Debut");
  for (int ii = 0; ii < DMP_CODE_SIZE; ii++) {
    byte data = pgm_read_byte(dmp_memory + ii);
    byte cur;
    writeByteEEPROM(i2cAddressEEPROM, ii, data );
    readStringEEPROM(i2cAddressEEPROM, ii, &cur, 1);
    Serial.print(ii, HEX); Serial.print(" ("); Serial.print(ii); Serial.print(")  "); Serial.println(data, HEX);
    if (cur != data ) {
      Serial.print("+++++++++++++Erreur adresse: ");
      Serial.print(ii, HEX); Serial.print(" ("); Serial.print(ii); Serial.print(")");
      Serial.print("  ecrit:"); Serial.print(data, HEX);
      Serial.print("  lu:"); Serial.println(cur, HEX);
    }
  }
  Serial.println("The end !");
}

void loop() {
  // put your main code here, to run repeatedly:

}
