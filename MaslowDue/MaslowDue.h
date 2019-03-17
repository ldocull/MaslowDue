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

// Maslow Firmware Version tracking
#define VERSIONNUMBER 1.99

// Define standard libraries used by maslow.

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

// PID position loop factors              X: Kp = 25000 Ki = 15000 Kd = 22000 Imax = 5000
// 14.000 fixed point arithmatic S13.10
#define default_xKp     (25.000*1024)
#define default_xKi     (17.000*1024)
#define default_xImax   (5000)
#define default_xKd     (21.000*1024)

#define default_yKp     (25.000*1024)
#define default_yKi     (17.000*1024)
#define default_yImax   (5000)
#define default_yKd     (21.000*1024)

#define default_zKp     (22.000*1024)
#define default_zKi     (17.000*1024)
#define default_zImax   (5000)
#define default_zKd     (20.000*1024)

//#define default_yKp     (5.000*1024)  /* Small test motor setup */
//#define default_yKi     (4.000*1024)
//#define default_yImax   (5000)
//#define default_yKd     (8.000*1024)

#ifdef MASLOWCNC
  #define default_machineWidth        (96*25.4)
  #define default_machineHeight       (48*25.4)
  #define default_distBetweenMotors   (118.9375*25.4)
  #define default_motorOffsetY        (22.75*25.4)
  #define default_chainLength         (132.283*25.4)
  #define default_chainOverSprocket   (0)
  #define default_chainSagCorrection  (59.504839)
  #define default_leftChainTolerance  (0)
  #define default_rightChainTolerance (0)
  #define default_rotationDiskRadius  (104.3)
  #define default_sledHeight          (139)
  #define default_sledWidth           (310)
  #define default_XcorrScaling        (1.003922)
  #define default_YcorrScaling        (0.998440)
#endif
  
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
