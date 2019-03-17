/*
  cpu_map_atmega328p.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon

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
*/

/* Grbl officially supports the Arduino Uno, but the other supplied pin mappings are
   supplied by users, so your results may vary. This cpu_map file serves as a central
   pin mapping settings file for AVR 328p used on the Arduino Uno.  */
   
//   #include "sam.h"
#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif

/*
DUE
	Os X
		Step C8
		Dir A15
		Enable D1
	Os Y
		Step D3
		Dir D9
		Enable D10
		
	Os Z
		Step C2
		Dir C4
		Enable C6



*/

#define GRBL_PLATFORM "MaslowDUE"

#define X_STEP_BIT      1  // Due Digital = C.1 pin 33
#define Y_STEP_BIT      2  // Due Digital = C.2 pin 34
#define Z_STEP_BIT      3  // Due Digital = C.3 pin 35
#define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

#define X_DIRECTION_BIT   4  // Due Digital = C.4 pin 36
#define Y_DIRECTION_BIT   5  // Due Digital = C.5 pin 37
#define Z_DIRECTION_BIT   6  // Due Digital = C.6 pin 38
#define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

#define STEPPERS_DISABLE_BIT    7  // Due Pin = C.7 pin 39
#define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)

// Define homing/hard limit switch input pins and limit interrupt vectors. 
// NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (CONTROL).

#define X_LIMIT_BIT      8  // Due pin 40 - C.8
#define Y_LIMIT_BIT      9  // Due pin 41 - C.9
#ifdef VARIABLE_SPINDLE   // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
  #define Z_LIMIT_BIT     13 // Due pin 50 - C.13
#else
  #define Z_LIMIT_BIT    12  // Due pin 51 - C.12
#endif

//  
//// Define probe switch input pin.

#define PROBE_BIT       5  // Uno Analog Pin 5
#define PROBE_MASK      (1<<PROBE_BIT)
//
//// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE
//  // Advanced Configuration Below You should not need to touch these variables
  #define PWM_MAX_VALUE    255.0

#endif // End of VARIABLE_SPINDLE
