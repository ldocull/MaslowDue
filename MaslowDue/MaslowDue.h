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

//
// -- SHIELD SELECTION
//

//#define MakerMadeCNC_V2   /* Uncomment for V2 MakerMade CNC (1.0) Shield */
#define MakerMadeCNC_V1   /* Uncomment for V1 MakerMade CNC (1.0) Shield */
//#define DRIVER_L298P_12    /* Uncomment this for a L298P version 1.2 Shield */
//#define DRIVER_L298P_11    /* Uncomment this for a L298P version 1.1 Shield */
//#define DRIVER_L298P_10    /* Uncomment this for a L298P version 1.0 Shield */
//#define DRIVER_TLE5206       /* Uncomment this for a TLE5206 version Shield */

// uncomment this to work on PID settings and such using a terminal window
//#define TUNING_MODE 1

// HARDWARE PIN MAPPING
#define HeartBeatLED 13

#ifdef MakerMadeCNC_V2
  #define YP_PWM 6      /* Y-axis positive direction PWM output */
  #define YM_PWM 4      /* Y-axis negative direction PWM output */
  #define Y_ENABLE 5

  #define ZP_PWM 7      /* Z-axis positive direction PWM output */
  #define ZM_PWM 9      /* Z-axis negative direction PWM output */
  #define Z_ENABLE 8

  #define XP_PWM 12     /* X-axis positive direction PWM output */
  #define XM_PWM 11     /* X-axis negative direction PWM output */
  #define X_ENABLE 10
#endif

#ifdef MakerMadeCNC_V1
  #define YP_PWM 6      /* Y-axis positive direction PWM output */
  #define YM_PWM 4      /* Y-axis negative direction PWM output */
  #define Y_ENABLE 5

  #define ZP_PWM 7      /* Z-axis positive direction PWM output */
  #define ZM_PWM 9      /* Z-axis negative direction PWM output */
  #define Z_ENABLE 8

  #define XP_PWM 12     /* X-axis positive direction PWM output */
  #define XM_PWM 11     /* X-axis negative direction PWM output */
  #define X_ENABLE 10
#endif

#ifdef DRIVER_L298P_12
  #define YP_PWM 6      /* Y-axis positive direction PWM output */
  #define YM_PWM 4      /* Y-axis negative direction PWM output */
  #define Y_ENABLE 5

  #define ZP_PWM 7      /* Z-axis positive direction PWM output */
  #define ZM_PWM 9      /* Z-axis negative direction PWM output */
  #define Z_ENABLE 8

  #define XP_PWM 12     /* X-axis positive direction PWM output */
  #define XM_PWM 11     /* X-axis negative direction PWM output */
  #define X_ENABLE 10
#endif

#ifdef DRIVER_L298P_11
  #define YP_PWM 4      /* Y-axis positive direction PWM output */
  #define YM_PWM 6      /* Y-axis negative direction PWM output */
  #define Y_ENABLE 5

  #define ZP_PWM 9      /* Z-axis positive direction PWM output */
  #define ZM_PWM 7      /* Z-axis negative direction PWM output */
  #define Z_ENABLE 8

  #define XP_PWM 10     /* X-axis positive direction PWM output */
  #define XM_PWM 11     /* X-axis negative direction PWM output */
  #define X_ENABLE 12
#endif

#ifdef DRIVER_L298P_10
  #define YP_PWM 8      /* Y-axis positive direction PWM output */
  #define YM_PWM 9      /* Y-axis negative direction PWM output */
  #define Y_ENABLE 6

  #define ZP_PWM 11      /* Z-axis positive direction PWM output */
  #define ZM_PWM 10      /* Z-axis negative direction PWM output */
  #define Z_ENABLE 7

  #define XP_PWM 13     /* X-axis positive direction PWM output */
  #define XM_PWM 12     /* X-axis negative direction PWM output */
  #define X_ENABLE 5
#endif

#ifdef DRIVER_TLE5206
  #define YP_PWM 6      /* Y-axis positive direction PWM output */
  #define YM_PWM 4      /* Y-axis negative direction PWM output */
  #define Y_FAULT 5   /* 5 this is NOT an ENA output, this is a fault-line input */

  #define ZP_PWM 9      /* Z-axis positive direction PWM output */
  #define ZM_PWM 7      /* Z-axis negative direction PWM output */
  #define Z_FAULT 8   /* 8 this is NOT an ENA output, this is a fault-line input */

  #define XP_PWM 10     /* X-axis positive direction PWM output */
  #define XM_PWM 11     /* X-axis negative direction PWM output */
  #define X_FAULT 12   /* 12 this is NOT an ENA output, this is a fault-line input */
#endif

#define SCLpin  15    /* EEPROM i2c signals */
#define SDApin  14


#define SPINDLE_TIMER Timer1
#define Spindle_PWM 16      /* output pin for Spindle PWM */
#define Spindle_PERIOD 2000 /* 500 hz */

#ifdef MakerMadeCNC_V1
  #define Encoder_YA 20 /* Y encoder phases A & B */
  #define Encoder_YB 21
  #define Encoder_ZA 19 /* Z encoder phases A & B */
  #define Encoder_ZB 18
  #define Encoder_XA 2  /* X encoder phases A & B */
  #define Encoder_XB 3
#endif

#ifdef MakerMadeCNC_V2
  #define Encoder_YA 20 /* Y encoder phases A & B */
  #define Encoder_YB 21
  #define Encoder_ZA 18 /* Z encoder phases A & B */
  #define Encoder_ZB 19
  #define Encoder_XA 2  /* X encoder phases A & B */
  #define Encoder_XB 3
#endif

#ifdef DRIVER_L298P_12
  #define Encoder_YA 20 /* Y encoder phases A & B */
  #define Encoder_YB 21
  #define Encoder_ZA 19 /* Z encoder phases A & B */
  #define Encoder_ZB 18
  #define Encoder_XA 2  /* X encoder phases A & B */
  #define Encoder_XB 3
#endif
#ifdef DRIVER_L298P_11
  #define Encoder_YA 20 /* Y encoder phases A & B */
  #define Encoder_YB 21
  #define Encoder_ZA 19 /* Z encoder phases A & B */
  #define Encoder_ZB 18
  #define Encoder_XA 2  /* X encoder phases A & B */
  #define Encoder_XB 3
#endif
#ifdef DRIVER_L298P_10
  #define Encoder_YA 18 /* Y encoder phases A & B */
  #define Encoder_YB 19
  #define Encoder_ZA 2 /* Z encoder phases A & B */
  #define Encoder_ZB 3
  #define Encoder_XA 21  /* X encoder phases A & B */
  #define Encoder_XB 20
#endif
#ifdef DRIVER_TLE5206
  #define Encoder_YA 20 /* Y encoder phases A & B */
  #define Encoder_YB 21
  #define Encoder_ZA 19 /* Z encoder phases A & B */
  #define Encoder_ZB 18
  #define Encoder_XA 2  /* X encoder phases A & B */
  #define Encoder_XB 3
#endif


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

// publicly available wrapper functions for computing kinematics (defers to triangular functions).
void  chainToPosition(float aChainLength, float bChainLength, float *x,float *y );
void  positionToChain(float xTarget,float yTarget, float* aChainLength, float* bChainLength);

#endif
