/*
  Serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

  reworked for Maslow-Due (Arduino Due) by Larry D O'Cull  Feb 10, 2019
*/
#include "MaslowDue.h"
#include "grbl.h"
#include "DueTimer.h"

uint8_t serial_rx_buffer[RX_BUFFER_SIZE];
uint8_t serial_rx_buffer_head = 0;
volatile uint8_t serial_rx_buffer_tail = 0;

void serialScanner_handler(void);

#ifdef ENABLE_XONXOFF
  volatile uint8_t flow_ctrl = XON_SENT; // Flow control state variable
#endif

  
void serial_reset_read_buffer() 
{
    while(MACHINE_COM_PORT.available() != 0)
      MACHINE_COM_PORT.read();
    serial_rx_buffer_tail = serial_rx_buffer_head;

    #ifdef ENABLE_XONXOFF
      flow_ctrl = XON_SENT;
      MACHINE_COM_PORT.write(flow_ctrl);
    #endif
}

// Returns the number of bytes used in the RX serial buffer.
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{
  return (TX_BUFFER_SIZE - MACHINE_COM_PORT.availableForWrite());
}


void serial_init()
{
  // Set baud rate

    MACHINE_COM_PORT.begin(BAUD_RATE);
    // defaults to 8-bit, no parity, 1 stop bit
    serial_reset_read_buffer();
    
  // fill serial recieve buffer every 2ms with available chars 
//    Timer7.attachInterrupt(serialScanner_handler).start(2000);  
}


// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data) 
{
      while(MACHINE_COM_PORT.availableForWrite() == 0)
      { 
        if (sys_rt_exec_state & EXEC_RESET) 
          return;  // Only check for abort to avoid an endless loop.
      }     
      MACHINE_COM_PORT.write(data);
}


// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
  uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial_rx_buffer[tail];
    
    tail++;
    if (tail == RX_BUFFER_SIZE) { tail = 0; }
    serial_rx_buffer_tail = tail;

    #ifdef ENABLE_XONXOFF
      if ((serial_get_rx_buffer_count() < RX_BUFFER_LOW) && flow_ctrl == XOFF_SENT) { 
        flow_ctrl = SEND_XON;
        MACHINE_COM_PORT.write(flow_ctrl);
      }
    #endif
    
    return data;
  }
}

void serialScanner_handler(void)
{
  // work with Arduino serial library by scanning and loading the recieve buffer
  // as if it were directly responding to the ISR -- which it isn't
  uint8_t data = 0;
  uint8_t next_head;

  if(MACHINE_COM_PORT.available() == 0)
      return;
  
  data = MACHINE_COM_PORT.read();
  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the buffer, but these set system state flag bits for realtime execution.
  switch (data) {
    case CMD_STATUS_REPORT: bit_true_atomic(sys_rt_exec_state, EXEC_STATUS_REPORT); break; // Set as true
    case CMD_CYCLE_START:   bit_true_atomic(sys_rt_exec_state, EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     bit_true_atomic(sys_rt_exec_state, EXEC_FEED_HOLD); break; // Set as true
    case CMD_SAFETY_DOOR:   bit_true_atomic(sys_rt_exec_state, EXEC_SAFETY_DOOR); break; // Set as true
    case CMD_RESET: case '&': mc_reset(); break; // Call motion control reset routine.
    default: // Write character to buffer    
      next_head = serial_rx_buffer_head + 1;
      if (next_head == RX_BUFFER_SIZE) { next_head = 0; }
    
      // Write data to buffer unless it is full.
      if (next_head != serial_rx_buffer_tail) {
        serial_rx_buffer[serial_rx_buffer_head] = data;
        serial_rx_buffer_head = next_head;    
        
        #ifdef ENABLE_XONXOFF
          if ((serial_get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT) {
            flow_ctrl = SEND_XOFF;
              MACHINE_COM_PORT.write(flow_ctrl);
          } 
        #endif
        
      }
  }
}
