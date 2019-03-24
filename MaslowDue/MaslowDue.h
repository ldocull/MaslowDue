/* This file is part of the Maslow Due Control Software.
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

    Created by:  Larry D O'Cull  Feb 10, 2019
    
    Some portions of this package directly or indirectly pull from from the Maslow CNC 
    firmware for Aduino Mega.   Those parts are Copyright 2014-2017 Bar Smith <https://www.maslowcnc.com/>
    
    */
// This is the main maslow include file

#ifndef maslow_h
#define maslow_h

// HARDWARE PIN MAPPING
#define HeartBeatLED 13
#define YP_PWM 6      /* Y-axis positive direction PWM output */
#define YM_PWM 4      /* Y-axis negative direction PWM output */
#define Y_ENABLE 5
#define ZP_PWM 7      /* Z-axis positive direction PWM output */
#define ZM_PWM 9      /* Z-axis negative direction PWM output */
#define Z_ENABLE 8
#define XP_PWM 12     /* X-axis positive direction PWM output */
#define XM_PWM 11     /* X-axis negative direction PWM output */
#define X_ENABLE 10

#define SCLpin  15    /* EEPROM i2c signals */
#define SDApin  14

#define Spindle_PWM 16
#define Spindle_PERIOD 2000 /* 500 hz */

#define Encoder_YA 20 /* Y encoder phases A & B */
#define Encoder_YB 21
#define Encoder_ZA 19 /* Z encoder phases A & B */
#define Encoder_ZB 18
#define Encoder_XA 2  /* X encoder phases A & B */
#define Encoder_XB 3
        
#define X_STEP  33  /* GRBL harware interface */
#define X_DIRECTION 36
#define Z_STEP  34
#define Z_DIRECTION 37
#define Y_STEP  35
#define Y_DIRECTION 38
#define AXES_DISABLE 39

#define MAX_PWM_LEVEL 255
#define MIN_PWM_LEVEL 5

  
struct PID_MOTION 
{
  long int Kp;
  long int Ki;
  long int Imax;
  long int Kd;
  long int Error;
  long int Integral;
  long int axis_Position;
  long int last_Position;
  long int target;
  long int target_PS;
  long int last;
  long int Speed;
  long int totalSpeed;
  long int DiffTerm;
  long int iterm;
  long int stepSize;
  int P_PWM;
  int M_PWM;
  int ENABLE;
};

extern struct PID_MOTION x_axis, y_axis, z_axis;
extern long int xSpeed, ySpeed, zSpeed;  // current speed each axis
extern int healthLEDcounter;
extern int stepTestEnable;
extern int posEnabled;

void motorsEnabled(void);
void motorsDisabled(void);

#endif
