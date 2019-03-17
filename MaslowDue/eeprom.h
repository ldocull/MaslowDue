/*
  This file is part of the Maslow Due Control Software.
    The Maslow Due Control Software is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    Maslow Due Control Software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with the Maslow Control Software.  If not, see <http://www.gnu.org/licenses/>.

    Created by:  Larry D O'Cull 
 
    BitBanged I2C to allow the use of any available pins
    since Maslow shield uses SDA/SCL for encoder inputs. :(  
*/


#ifndef eeprom_h
#define eeprom_h


#define EERD 0x01
#define EEWR 0x00
#define EECC 0x50  /* 24LC256 device address is 0x50 */
#define I2C_DELAY 1
#define EEPROM_WRITE_TIME 5010  /* takes about 5ms to flash after a write.. */


unsigned char eeprom_get_char(unsigned int addr);
void eeprom_put_char(unsigned int addr, unsigned char new_value);
void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size);
int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size);
void store_current_machine_pos(void);
void recall_current_machine_pos(void);
void EEPROM_viewer(void);
#endif
