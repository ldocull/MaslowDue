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

#include "MaslowDue.h"
#include "grbl.h"
#include "eeprom.h"

struct {
  unsigned char control_byte;
  unsigned char address_high;
  unsigned char address_low;
} ee_CS = {0,0,0};

void eeprom_init(void)
{
    pinMode(SDApin, INPUT);
    pinMode(SCLpin, OUTPUT);
    digitalWrite(SCLpin, LOW); // default to inputs with pullups on
}

int eeprom_get_ack()
{
    int a;
    
    pinMode(SDApin, INPUT);     // prepare for acknowledge
    delayMicroseconds(I2C_DELAY);  
    digitalWrite(SCLpin, LOW);  // drop clk line   
    delayMicroseconds(I2C_DELAY);  
    digitalWrite(SCLpin, HIGH);  // raise clk line   
    delayMicroseconds(I2C_DELAY);  
    a = digitalRead(SDApin);
    digitalWrite(SCLpin, LOW);  // drop clk line   
    return a;
}

void eeprom_stop()
{
   pinMode(SDApin, OUTPUT);     // prepare for stop
   digitalWrite(SDApin, LOW);  // clk line low (should be low from ack
   delayMicroseconds(I2C_DELAY);  
   digitalWrite(SCLpin, HIGH);  // clk line low (should be low from ack
   delayMicroseconds(I2C_DELAY);  
   pinMode(SDApin, INPUT);
}

int eeprom_start(int read_write)
{
    int i;
    
    pinMode(SCLpin, OUTPUT);
    digitalWrite(SCLpin, LOW);
    delayMicroseconds(I2C_DELAY); 
    digitalWrite(SCLpin, HIGH);
    pinMode(SDApin, OUTPUT);
    digitalWrite(SDApin, HIGH);   // setup start state
    delayMicroseconds(I2C_DELAY);          
    digitalWrite(SDApin, LOW);   // setup start state
    delayMicroseconds(I2C_DELAY);  
    digitalWrite(SCLpin, LOW);   // SDA drops followed by SDA to create a start state
    ee_CS.control_byte = 0 | (EECC<<1) | read_write;  // send out control byte
    for(i=0; i<8; i++)
    {
          delayMicroseconds(I2C_DELAY);  
          digitalWrite(SDApin,((ee_CS.control_byte >> (7-i)) & 1)); // shift out data
          delayMicroseconds(I2C_DELAY);  
          digitalWrite(SCLpin, HIGH);  // drop clk line   
          delayMicroseconds(I2C_DELAY);          
          digitalWrite(SCLpin, LOW);  // drop clk line   
    }
    return eeprom_get_ack();        
}

void eeprom_set_addr(byte read_write, uint16_t addr)
{
    int i, ack;
    
     // start condition
    eeprom_init();    
    ack = eeprom_start(EEWR);   // write control byte

    digitalWrite(SCLpin, LOW);   // SDA drops followed by SDA to create a start state
    delayMicroseconds(I2C_DELAY);  
    pinMode(SDApin, OUTPUT);
    ee_CS.address_high = addr >> 8;  // send out high address byte  
    for(i=0; i<8; i++)
    {
          delayMicroseconds(I2C_DELAY);  
          digitalWrite(SDApin,((ee_CS.address_high >> (7-i)) & 1)); // shift out data
          delayMicroseconds(I2C_DELAY);  
          digitalWrite(SCLpin, HIGH);  // drop clk line   
          delayMicroseconds(I2C_DELAY);          
          digitalWrite(SCLpin, LOW);  // drop clk line   
    }
    ack = eeprom_get_ack();   

    ee_CS.address_low = addr & 0xFF;  // send out low address byte
    digitalWrite(SCLpin, LOW);   // SDA drops followed by SDA to create a start state
    delayMicroseconds(I2C_DELAY);      
    pinMode(SDApin, OUTPUT);
    for(i=0; i<8; i++)
    {
          delayMicroseconds(I2C_DELAY);  
          digitalWrite(SDApin,((ee_CS.address_low >> (7-i)) & 1)); // shift out data
          delayMicroseconds(I2C_DELAY);  
          digitalWrite(SCLpin, HIGH);  // drop clk line   
          delayMicroseconds(I2C_DELAY);          
          digitalWrite(SCLpin, LOW);  // drop clk line   
    }
    ack = eeprom_get_ack(); 
}

unsigned char eeprom_get_current_address()
{
    int i;
    byte databyte;
    
    eeprom_start(EERD);   //  control byte signals read
    pinMode(SDApin, INPUT);
    delayMicroseconds(I2C_DELAY);      
    digitalWrite(SCLpin, LOW);   // SDA drops followed by SDA to create a start state
    for(i=0; i<8; i++)
    {
          delayMicroseconds(I2C_DELAY);      
          databyte <<= 1;
          databyte |= digitalRead(SDApin); // shift in data
          digitalWrite(SCLpin, HIGH);  // drop clk line   
          delayMicroseconds(I2C_DELAY);  
          digitalWrite(SCLpin, LOW);  // drop clk line   
    }
    //    eeprom_get_ack();         No acknowedge!
    delayMicroseconds(I2C_DELAY);  
    eeprom_stop();
        
    return databyte;
}
unsigned char eeprom_get_char( unsigned int addr )
{
    eeprom_set_addr(EEWR, addr);    // write address for random read 
    eeprom_get_current_address();
}

void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
    int i;
    
    eeprom_set_addr(EEWR, addr);

    digitalWrite(SCLpin, LOW);   // SDA drops followed by SDA to create a start state
    delayMicroseconds(I2C_DELAY);      
    pinMode(SDApin, OUTPUT);
    for(i=0; i<8; i++)
    {
          delayMicroseconds(I2C_DELAY);  
          digitalWrite(SDApin,((new_value >> (7-i)) & 1)); // shift out data
          delayMicroseconds(I2C_DELAY);  
          digitalWrite(SCLpin, HIGH);  // drop clk line   
          delayMicroseconds(I2C_DELAY);          
          digitalWrite(SCLpin, LOW);  // drop clk line   
    }
    eeprom_get_ack();   
    eeprom_stop();
 
    delayMicroseconds(EEPROM_WRITE_TIME);  // write cycle time for eeprom is 5ms
}

void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) { 
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++)); 
  }
  eeprom_put_char(destination, checksum);
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) { 
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;    
    *(destination++) = data; 
  }
  return(checksum == eeprom_get_char(source));
}

void store_current_machine_pos(void)
{
   memcpy_to_eeprom_with_checksum(EEPROM_ADDR_MACHINE_STATE+0x10,(char *)sys_position,sizeof(sys_position));
   memcpy_to_eeprom_with_checksum(EEPROM_ADDR_MACHINE_STATE+0x40,(char *)pl.position,sizeof(pl.position));
   memcpy_to_eeprom_with_checksum(EEPROM_ADDR_MACHINE_STATE+0x80,(char *)gc_state.coord_system,sizeof(gc_state.coord_system));
   memcpy_to_eeprom_with_checksum(EEPROM_ADDR_MACHINE_STATE+0x100,(char *)gc_state.coord_offset,sizeof(gc_state.coord_offset));
   memcpy_to_eeprom_with_checksum(EEPROM_ADDR_MACHINE_STATE+0x180,(char *)&gc_state.tool_length_offset,sizeof(gc_state.tool_length_offset));
//   DEBUG_COM_PORT.print("MPOS SAVED\n");
}

void recall_current_machine_pos(void)
{
   if(eeprom_get_char(EEPROM_ADDR_MACHINE_STATE) != 0xA5) // test tag and init if not tagged
   {
      for(int i=0; i<0x200; i++)
        eeprom_put_char(EEPROM_ADDR_MACHINE_STATE+i,0x00);
      eeprom_put_char(EEPROM_ADDR_MACHINE_STATE,0xA5);      
      DEBUG_COM_PORT.print("EEMS Init\n");  
   }
   memcpy_from_eeprom_with_checksum((char *)sys_position,EEPROM_ADDR_MACHINE_STATE+0x10,sizeof(sys_position));
   memcpy_from_eeprom_with_checksum((char *)pl.position,EEPROM_ADDR_MACHINE_STATE+0x40,sizeof(pl.position));
   memcpy_from_eeprom_with_checksum((char *)gc_state.coord_system,EEPROM_ADDR_MACHINE_STATE+0x80,sizeof(gc_state.coord_system));
   memcpy_from_eeprom_with_checksum((char *)gc_state.coord_offset,EEPROM_ADDR_MACHINE_STATE+0x100,sizeof(gc_state.coord_offset));
   memcpy_from_eeprom_with_checksum((char *)&gc_state.tool_length_offset,EEPROM_ADDR_MACHINE_STATE+0x180,sizeof(gc_state.tool_length_offset));
//   DEBUG_COM_PORT.print("MPOS RECALLED\n");
}


unsigned char _cnvrt_char(unsigned char t)
{
    unsigned char temp = t;

    if(temp < '9') temp -= '0';  // decimal digit
    else if (temp >= 'a') temp -= 87; // lower case alpha
    else temp -= 55;  // upper case alpha
    
    return temp;
}

unsigned char serial_parseHexByte()
{
    while (serial_get_rx_buffer_count() < 2)
        ; 

    // read the incoming byte:
    unsigned char highChar = serial_read();
    unsigned char lowChar = serial_read();
    unsigned char temp = _cnvrt_char(highChar);
    temp <<= 4;
    temp |= _cnvrt_char(lowChar);

    return temp;
}

unsigned int serial_parseHexInt()
{
    unsigned int dataword = serial_parseHexByte();
    dataword <<= 8;
    dataword |= serial_parseHexByte();

    return dataword;
}

void EEPROM_viewer(void) 
{
  uint16_t addr;
  byte data;
  
  MACHINE_COM_PORT.print("EEPROM:>\n");
  while(1)
  {
    if ( serial_get_rx_buffer_count() > 0) 
    {
      byte incomingByte =  serial_read();
      switch(incomingByte)
      {

        case 't':
          data = serial_parseHexByte();
          MACHINE_COM_PORT.print("0x");
          MACHINE_COM_PORT.print(data,HEX);
          MACHINE_COM_PORT.print(",");
          MACHINE_COM_PORT.print(data,DEC);
          MACHINE_COM_PORT.print("\n");
          break;
          
        case 'w':
          addr = serial_parseHexInt();
          data = serial_parseHexByte();
          eeprom_put_char(addr,data);   
          MACHINE_COM_PORT.print("W");
          MACHINE_COM_PORT.print(">0x");
          MACHINE_COM_PORT.print(addr, HEX);
          MACHINE_COM_PORT.print("=0x");
          data = eeprom_get_char(addr);
          MACHINE_COM_PORT.print(data, HEX);
          MACHINE_COM_PORT.print("\n");
          break;
           
        case 'r':
          addr = serial_parseHexInt();
          MACHINE_COM_PORT.print("R");
          MACHINE_COM_PORT.print(">0x");
          MACHINE_COM_PORT.print(addr, HEX);
          MACHINE_COM_PORT.print("=0x");       
          data = eeprom_get_char(addr);
          MACHINE_COM_PORT.print(data, HEX);
          MACHINE_COM_PORT.print("\n");
          break;

                     
        case 'n':
          MACHINE_COM_PORT.print("N");
          MACHINE_COM_PORT.print(">0x");     
          data = eeprom_get_current_address();
          MACHINE_COM_PORT.print(data, HEX);
          MACHINE_COM_PORT.print("\n");
          break;

        case 'd':
          addr = serial_parseHexInt(); // dump block from address   
        case ' ':     
          for(int a=0; a<16; a++)
          {
              MACHINE_COM_PORT.print("\n0x");
    
              if(addr > 0x7FF0) addr = 0;

              MACHINE_COM_PORT.print(addr + (a*16),HEX);
              MACHINE_COM_PORT.print(" - ");
              data = eeprom_get_char(addr + (a*16));
              for(int p=0; p<16; p++)
              {
                 MACHINE_COM_PORT.print(" 0x");
                 MACHINE_COM_PORT.print(data,HEX);
                 MACHINE_COM_PORT.print(" ");
                 data = eeprom_get_current_address();
              }
          }
          addr += 256;
          MACHINE_COM_PORT.print("\n");
          break;

        case 'x':
          return;
      }

    }
  }
}

// end of file
