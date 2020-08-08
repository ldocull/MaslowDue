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

   This file (module) specifically contains the step-direction control used to drive the Maslow gear motors
   using the Maslow CNC shield board. Note: 10nf and 3.9K (parallel) must be added from each encoder phase
   lead to ground on the shield PCB in order for it to be compatible with the 3.3V high-speed inputs of the Arduino Due.

   The orignal Maslow firmware does not drive the motor in a 3-state (braking mode) or in a position loop, but
   is structured more as a precise velocity-loop based system. The behavior of this new drive setup will sound
   and act a bit different than what may have been experienced with a previous stock-Maslow setup. Generally,
   the GRBL driven system will seem faster overall due to the splining of vectors as the machine moves. The
   top speed will still be limited by the use of the original gear motors which will only go to about 20RPM.

   Also note that the chain configuration of this supplied software is for an 'under-sprocket' chain to a sled-ring
   system. The sled-ring simplifies the math to a triangular system and that makes it easier to compensate out any error.

 */
#include "MaslowDue.h"
#include "grbl.h"
#include "DueTimer.h"

int incomingByte = 0;
int healthLEDcounter = 0;

#ifdef TUNING_MODE
int stepTestEnable = 0;        // constant speed enabled for tuning tool
int posEnabled = 1;            // positioning enable for tuning tool
char which_axis = 'X';         // selected axis indicator
#endif

struct PID_MOTION *selected_axis;

struct PID_MOTION x_axis = {default_xKp,default_xKi,default_xImax,default_xKd,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
struct PID_MOTION y_axis = {default_yKp,default_yKi,default_yImax,default_yKd,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
struct PID_MOTION z_axis = {default_zKp,default_zKi,default_zImax,default_zKd,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// Declare system global variable structure
system_t sys;
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override; // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.
#ifdef DEBUG
  volatile uint8_t sys_rt_exec_debug;
#endif

int fault_was_low = 0;
int Motors_Disabled = 0;

long int xSpeed, ySpeed, zSpeed;  // current speed each axis
int lastXAstate,lastXBstate,lastYAstate,lastYBstate,lastZAstate,lastZBstate;

void serial_init(void);
void settings_init(void); // Load Grbl settings from EEPROM
void stepper_init(void);  // Configure stepper pins and interrupt timers
void system_init(void);   // Configure pinout pins and pin-change interrupt
void serial_reset_read_buffer(void); // Clear serial read buffer
void serialScanner_handler(void);
void gc_init(void); // Set g-code parser to default state
void spindle_init(void);
void coolant_init(void);
void limits_init(void);
void probe_init(void);
void plan_reset(void); // Clear block buffer and planner variables
void st_reset(void); // Clear stepper subsystem variables.
void plan_sync_position(void);
void gc_sync_position(void);
int protocol_main_loop(void);
void protocol_init(void);
void tuningLoop(void);

//
//  computes new axis speed based on motor positions state and target
//
long compute_PID(struct PID_MOTION *axis_ptr)
{
  int sign;
  long Speed;
  int PWM_value;

  axis_ptr->Speed = ((axis_ptr->Kp) * axis_ptr->Error);    // proportional term..

  axis_ptr->Integral += axis_ptr->Error;
  if(axis_ptr->Integral > axis_ptr->Imax) axis_ptr->Integral = axis_ptr->Imax;
  if(axis_ptr->Integral < (-axis_ptr->Imax)) axis_ptr->Integral = (-axis_ptr->Imax);
  axis_ptr->iterm = (axis_ptr->Ki) * axis_ptr->Integral;    // integral term..
  axis_ptr->Speed += axis_ptr->iterm >> 3;

  axis_ptr->DiffTerm = (axis_ptr->last_Position - axis_ptr->axis_Position);
  axis_ptr->last_Position = axis_ptr->axis_Position;       // differential term..
  axis_ptr->Speed += ((axis_ptr->Kd) * axis_ptr->DiffTerm);

  Speed = axis_ptr->Speed;
  sign = 1;
  if(Speed < 0)
    sign = -1;  // remember sign of direction

  Speed = abs(Speed); // make a magnitude
  Speed += 128;  // round in the fraction..  ie V=101720 CMD=101 (102)
  Speed >>= 10;  // Scale out, but leave the fraction!

  PWM_value = (int) Speed;
  if(PWM_value > MAX_PWM_LEVEL) PWM_value = MAX_PWM_LEVEL;  // PWM limiter
  #if (MIN_PWM_LEVEL > 0)
    if(PWM_value < MIN_PWM_LEVEL) PWM_value = MIN_PWM_LEVEL;  // PWM limiter
  #endif

 #ifdef TUNING_MODE
  if(!posEnabled) Speed = 0;   // all stop (forced)
 #endif

  if(sign > 0)
  {
  #ifdef DRIVER_TLE5206
      digitalWrite(axis_ptr->M_PWM, 1);       // spin Positive
      if(Motors_Disabled)
        digitalWrite(axis_ptr->P_PWM, 1);
      else
        analogWrite(axis_ptr->P_PWM, (255-PWM_value));
  #else
      analogWrite(axis_ptr->M_PWM, 0);       // spin Positive
      analogWrite(axis_ptr->P_PWM, PWM_value);
  #endif
  }
  else
  {
   #ifdef DRIVER_TLE5206
      digitalWrite(axis_ptr->P_PWM, 1);       // spin Negative
      if(Motors_Disabled)
        digitalWrite(axis_ptr->M_PWM, 1);
      else
      analogWrite(axis_ptr->M_PWM, (255-PWM_value));
   #else
      analogWrite(axis_ptr->P_PWM, 0);       // spin Negative
      analogWrite(axis_ptr->M_PWM, PWM_value);
   #endif
  }

  return(axis_ptr->Speed);
}

void motorsDisabled(void)
{
    Motors_Disabled = 1;
  #ifndef DRIVER_TLE5206
    digitalWrite(X_ENABLE, 0);  // disable the motor driver
    digitalWrite(Y_ENABLE, 0);
    digitalWrite(Z_ENABLE, 0);
  #endif
//  DEBUG_COM_PORT.print("MOTORS OFF\n");
}

void motorsEnabled(void)
{
    Motors_Disabled = 0;
  #ifndef DRIVER_TLE5206
    digitalWrite(X_ENABLE, 1);  // Enable the motor driver
    digitalWrite(Y_ENABLE, 1);
    digitalWrite(Z_ENABLE, 1);
  #endif
//  DEBUG_COM_PORT.print("MOTORS ON\n");
}

void MotorPID_Timer_handler(void)  // PID interrupt service routine
{
    x_axis.Error = x_axis.target - x_axis.axis_Position; // current position error
    y_axis.Error = y_axis.target - y_axis.axis_Position; // current position error
    z_axis.Error = z_axis.target - z_axis.axis_Position; // current position error

    interrupts();  // reenable interrupts so encoder pulses will all be counted!!

    xSpeed = compute_PID(&x_axis);
    ySpeed = compute_PID(&y_axis);
    zSpeed = compute_PID(&z_axis);

  #ifdef TUNING_MODE
    if(stepTestEnable)
    {
      x_axis.target_PS += x_axis.stepSize;
      x_axis.target = x_axis.target_PS >> 7;   // if in position, then move the target
      y_axis.target_PS += y_axis.stepSize;
      y_axis.target = y_axis.target_PS >> 7;   // if in position, then move the target
      z_axis.target_PS += z_axis.stepSize;
      z_axis.target = z_axis.target_PS >> 7;   // if in position, then move the target
    }
  #endif

  //  #ifndef VARIABLE_SPINDLE
  //   serialScanner_handler(); // work the serial buffer pre-parser from this timer!
  //  #endif

    digitalWrite(HeartBeatLED, healthLEDcounter++ & 0x40);
}

void update_Encoder_XA(void)
{
  volatile int encState  = 0;
  delayMicroseconds(2);
  encState = (digitalRead(Encoder_XA) << 1) + digitalRead(Encoder_XB);
  if(lastXAstate == encState) return; // noise reject
  switch(encState)
  {
    case 0:
    case 3:
      x_axis.axis_Position++;
      lastXAstate = encState;
      break;

    case 1:
    case 2:
      x_axis.axis_Position--;
      lastXAstate = encState;
      break;
  }
}

void update_Encoder_XB(void)
{
  volatile int encState  = 0;
  delayMicroseconds(2);
  encState = (digitalRead(Encoder_XA) << 1) + digitalRead(Encoder_XB);
  if(lastXBstate == encState) return; // noise reject
  switch(encState)
  {
    case 0:
    case 3:
      x_axis.axis_Position--;
      lastXBstate = encState;
      break;

    case 1:
    case 2:
      x_axis.axis_Position++;
      lastXBstate = encState;
      break;
  }
}

void update_Encoder_YA(void)
{
  volatile int encState  = 0;
  delayMicroseconds(2);
  encState = (digitalRead(Encoder_YA) << 1) + digitalRead(Encoder_YB);
  if(lastYAstate == encState) return; // noise reject
  switch(encState)
  {
    case 0:
    case 3:
      y_axis.axis_Position++;
      lastYAstate = encState;
      break;

    case 1:
    case 2:
      y_axis.axis_Position--;
      lastYAstate = encState;
      break;
  }
}

void update_Encoder_YB(void)
{
  volatile int encState  = 0;
  delayMicroseconds(2);
  encState = (digitalRead(Encoder_YA) << 1) + digitalRead(Encoder_YB);
  if(lastYBstate == encState) return; // noise reject
  switch(encState)
  {
    case 0:
    case 3:
      y_axis.axis_Position--;
      lastYBstate = encState;
      break;

    case 1:
    case 2:
     y_axis.axis_Position++;
     lastYBstate = encState;
     break;
  }
}

void update_Encoder_ZA(void)
{
  volatile int encState  = 0;
  delayMicroseconds(2);
  encState = (digitalRead(Encoder_ZA) << 1) + digitalRead(Encoder_ZB);
  if(lastZAstate == encState) return; // noise reject
  switch(encState)
  {
    case 0:
    case 3:
      z_axis.axis_Position++;
      lastZAstate = encState;
      break;

    case 1:
    case 2:
      z_axis.axis_Position--;
      lastZAstate = encState;
      break;
  }
}

void update_Encoder_ZB(void)
{
  volatile int encState  = 0;
  delayMicroseconds(2);
  encState = (digitalRead(Encoder_ZA) << 1) + digitalRead(Encoder_ZB);
  if(lastZBstate == encState) return; // noise reject
  switch(encState)
  {
    case 0:
    case 3:
      z_axis.axis_Position--;
      lastZBstate = encState;
      break;

    case 1:
    case 2:
      z_axis.axis_Position++;
      lastZBstate = encState;
      break;
  }
}

//
//  Initialize hardware and preset variables
//
void setup()
{
  int microseconds_per_millisecond = 1000;

  noInterrupts();           // disable all interrupts

  pinMode(HeartBeatLED, OUTPUT);
  digitalWrite(HeartBeatLED, LOW);

  pinMode(XP_PWM, OUTPUT);
  pinMode(XM_PWM, OUTPUT);
  pinMode(Encoder_XA, INPUT_PULLUP);
  pinMode(Encoder_XB, INPUT_PULLUP);
  x_axis.P_PWM = XP_PWM;  // preload hardware abstraction
  x_axis.M_PWM = XM_PWM;
  #ifdef DRIVER_TLE5206
    digitalWrite(X_FAULT,1);    // setup fault lines..
    pinMode(X_FAULT,INPUT);
    analogWrite(XP_PWM,255);
    analogWrite(XM_PWM,255);
    x_axis.ENABLE = X_FAULT;
  #else
    analogWrite(XP_PWM, 0);
    analogWrite(XM_PWM, 0);
    pinMode(X_ENABLE, OUTPUT);
    x_axis.ENABLE = X_ENABLE;
  #endif

  pinMode(YP_PWM, OUTPUT);
  pinMode(YM_PWM, OUTPUT);
  pinMode(Encoder_YA, INPUT_PULLUP);
  pinMode(Encoder_YB, INPUT_PULLUP);
  y_axis.P_PWM = YP_PWM;
  y_axis.M_PWM = YM_PWM;
  #ifdef DRIVER_TLE5206
    digitalWrite(Y_FAULT,1);
    pinMode(Y_FAULT,INPUT);
    analogWrite(YP_PWM,255);
    analogWrite(YM_PWM,255);
    y_axis.ENABLE = Y_FAULT;
  #else
    analogWrite(YP_PWM, 0);
    analogWrite(YM_PWM, 0);
    pinMode(Y_ENABLE, OUTPUT);
    y_axis.ENABLE = Y_ENABLE;
  #endif

  pinMode(ZP_PWM, OUTPUT);
  pinMode(ZM_PWM, OUTPUT);
  pinMode(Encoder_ZA, INPUT_PULLUP);
  pinMode(Encoder_ZB, INPUT_PULLUP);
  z_axis.P_PWM = ZP_PWM;
  z_axis.M_PWM = ZM_PWM;
  #ifdef DRIVER_TLE5206
    digitalWrite(Z_FAULT,1);
    pinMode(Z_FAULT,INPUT);
    analogWrite(ZP_PWM,255);
    analogWrite(ZM_PWM,255);
    z_axis.ENABLE = Z_FAULT;
  #else
    analogWrite(ZP_PWM, 0);
    analogWrite(ZM_PWM, 0);
    pinMode(Z_ENABLE, OUTPUT);
    z_axis.ENABLE = Z_ENABLE;
  #endif

  pinMode(Spindle_PWM, OUTPUT);
  digitalWrite(Spindle_PWM, 0);

  motorsDisabled();

    // initialize hard-time MotorPID_Timer for servos (10ms loop)
  Timer5.attachInterrupt(MotorPID_Timer_handler).setPeriod(10000).start();

    // hook up encoders
  attachInterrupt(digitalPinToInterrupt(Encoder_XA), update_Encoder_XA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_XB), update_Encoder_XB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_YA), update_Encoder_YA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_YB), update_Encoder_YB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_ZA), update_Encoder_ZA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_ZB), update_Encoder_ZB, CHANGE);

  serial_init();   // Setup serial baud rate and interrupts for machine port

  settings_init(); // Load Grbl settings from EEPROM

  x_axis.Kp = settings.x_PID_Kp; // get loop values from storage
  x_axis.Ki = settings.x_PID_Ki;
  x_axis.Kd = settings.x_PID_Kd;
  x_axis.Imax = settings.x_PID_Imax;
  x_axis.axis_Position = 0;
  x_axis.target = 0;
  x_axis.target_PS = 0;
  x_axis.Integral = 0;

  y_axis.Kp = settings.y_PID_Kp; // get loop values from storage
  y_axis.Ki = settings.y_PID_Ki;
  y_axis.Kd = settings.y_PID_Kd;
  y_axis.Imax = settings.y_PID_Imax;
  y_axis.axis_Position = 0;
  y_axis.target = 0;
  y_axis.target_PS = 0;
  y_axis.Integral = 0;

  z_axis.Kp = settings.z_PID_Kp; // get loop values from storage
  z_axis.Ki = settings.z_PID_Ki;
  z_axis.Kd = settings.z_PID_Kd;
  z_axis.Imax = settings.z_PID_Imax;
  z_axis.axis_Position = 0;
  z_axis.target = 0;
  z_axis.target_PS = 0;
  z_axis.Integral = 0;

  selected_axis = &x_axis;    // default select axis for diagnostics

  stepper_init();  // Configure stepper pins and interrupt timers
  system_init();   // Configure pinout pins and pin-change interrupt

  extern system_t sys;
  memset(&sys, 0, sizeof(system_t));  // Clear all system variables
  memset(sys_position,0,sizeof(sys_position)); // Clear machine position.
  sys.abort = true;   // Set abort to complete initialization

  interrupts();               // enable all interrupts

  #if (DEBUG_COM_PORT != MACHINE_COM_PORT)
    DEBUG_COM_PORT.begin(BAUD_RATE); // setup serial for tuning mode only
    DEBUG_COM_PORT.print("DEBUG\n");
  #endif


#ifndef TUNING_MODE
  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif

  // Force Grbl into an ALARM state upon a power-cycle or hard reset.
  #ifdef FORCE_INITIALIZATION_ALARM
    sys.state = STATE_ALARM;
  #endif

    // Start Grbl main loop. Processes program inputs and executes them.
   protocol_init();
#else
   motorsEnabled();  // for tuning loop
#endif
}

void loop()
{
#ifndef TUNING_MODE
    // Arduinos love for this loop to run free -- so no blocking!
    if(protocol_main_loop() !=0)  // in exeptions, holds, etc.. reinitialise!
       protocol_init();
#else
    tuningLoop();
#endif
}

 #ifdef TUNING_MODE
// TEST stuff for using ascii terminal monitor to arrive at motor PID coefficients
    float t1,t2, xx, yy;
    void settings_restore(uint8_t restore_flag);
  void tuningLoop(void)
  {
      // Scan for user stuff
  //  if(selected_axis->axis_Position != selected_axis->target)
  //  {
  //         DEBUG_COM_PORT.print(which_axis);
  //         DEBUG_COM_PORT.print(": Tar=");
  //         DEBUG_COM_PORT.print(selected_axis->target,DEC);
  //         DEBUG_COM_PORT.print(" Pos=");
  //         DEBUG_COM_PORT.print(selected_axis->axis_Position,DEC);
  //         DEBUG_COM_PORT.print(" err=");
  //         DEBUG_COM_PORT.print(selected_axis->Error,DEC);
  //         DEBUG_COM_PORT.print("\tCMD=");
  //         DEBUG_COM_PORT.print(selected_axis->Speed,DEC);
  //
  //         DEBUG_COM_PORT.print("\n");
  //  }
    if ( DEBUG_COM_PORT.available() > 0)
    {
              // read the incoming byte:
      incomingByte =  DEBUG_COM_PORT.read();
      switch(incomingByte)
      {
        case 'x':
          selected_axis = &x_axis;
           DEBUG_COM_PORT.print("X-Axis Selected\n");
          which_axis = 'X';
          break;

        case 'y':
          selected_axis = &y_axis;
           DEBUG_COM_PORT.print("Y-Axis Selected\n");
          which_axis = 'Y';
          break;

        case 'z':
          selected_axis = &z_axis;
           DEBUG_COM_PORT.print("Z-Axis Selected\n");
          which_axis = 'Z';
          break;

        case 'g':
          posEnabled = 1;
          break;

        case 'r':   // reset current position
          settings_restore(0xFF); // Load Grbl settings from EEPROM

          selected_axis->axis_Position = 0;
          selected_axis->target = 0;
          selected_axis->target_PS = 0;
          selected_axis->Integral = 0;
          stepTestEnable = 0;
          posEnabled = 0;
          break;

        case 'i':
          selected_axis->Ki =  DEBUG_COM_PORT.parseInt();
           DEBUG_COM_PORT.print("Ki == ");
           DEBUG_COM_PORT.print(selected_axis->Ki,DEC);
          break;

        case 'm':
          selected_axis->Imax =  DEBUG_COM_PORT.parseInt();
           DEBUG_COM_PORT.print("Imax == ");
           DEBUG_COM_PORT.print(selected_axis->Imax,DEC);
          break;

        case 'd':
          selected_axis->Kd =  DEBUG_COM_PORT.parseInt();
           DEBUG_COM_PORT.print("Kd == ");
           DEBUG_COM_PORT.print(selected_axis->Kd,DEC);
          break;

        case 'p':
          selected_axis->Kp =  DEBUG_COM_PORT.parseInt();
           DEBUG_COM_PORT.print("Kp == ");
           DEBUG_COM_PORT.print(selected_axis->Kp,DEC);
          break;

        case 's':
          stepTestEnable = 1;
          selected_axis->stepSize =  DEBUG_COM_PORT.parseInt();
           DEBUG_COM_PORT.print("S == ");
           DEBUG_COM_PORT.print(selected_axis->stepSize,DEC);
          break;

    #ifdef MASLOWCNC

        case 'a':
        // TEST
          xx = DEBUG_COM_PORT.parseFloat();
          yy = DEBUG_COM_PORT.parseFloat();
          positionToChain(xx, yy, &t1, &t2);   // test kinematics
          DEBUG_COM_PORT.print("\n xx,yy = ");   // from x,y to Left/Right back to x,y
          DEBUG_COM_PORT.print(xx);
          DEBUG_COM_PORT.print(",");
          DEBUG_COM_PORT.print(yy);
          DEBUG_COM_PORT.print(" -> L,R: ");
          DEBUG_COM_PORT.print(t1);
          DEBUG_COM_PORT.print(",");
          DEBUG_COM_PORT.print(t2);

          chainToPosition(t1, t2, &xx, &yy);
          DEBUG_COM_PORT.print(" <= L,R: ");
          DEBUG_COM_PORT.print(t1);
          DEBUG_COM_PORT.print(",");
          DEBUG_COM_PORT.print(t2);
          DEBUG_COM_PORT.print(" xx,yy = ");
          DEBUG_COM_PORT.print(xx);
          DEBUG_COM_PORT.print(",");
          DEBUG_COM_PORT.print(yy);
          DEBUG_COM_PORT.print("\n");

          break;
    #endif

        case '+':   // Move
          selected_axis->target += 1000;
          break;

        case '-':   // Move
          selected_axis->target -= 1000;
          break;

        case '*':   // Move
          selected_axis->target += 1;
          break;

        case '/':   // Move
          selected_axis->target -= 1;
          break;


        default:
           DEBUG_COM_PORT.print(which_axis);
           DEBUG_COM_PORT.print(": Kp = ");
           DEBUG_COM_PORT.print(selected_axis->Kp,DEC);
           DEBUG_COM_PORT.print(" Ki = ");
           DEBUG_COM_PORT.print(selected_axis->Ki,DEC);
           DEBUG_COM_PORT.print(" Kd = ");
           DEBUG_COM_PORT.print(selected_axis->Kd,DEC);
           DEBUG_COM_PORT.print(" Imax = ");
           DEBUG_COM_PORT.print(selected_axis->Imax,DEC);

           DEBUG_COM_PORT.print("\nerr=");
           DEBUG_COM_PORT.print(selected_axis->Error,DEC);
           DEBUG_COM_PORT.print("\t\ti=");
           DEBUG_COM_PORT.print(selected_axis->Integral,DEC);
           DEBUG_COM_PORT.print("\tiT=");
           DEBUG_COM_PORT.print(selected_axis->iterm,DEC);
           DEBUG_COM_PORT.print("\td=");
           DEBUG_COM_PORT.print(selected_axis->DiffTerm,DEC);
           DEBUG_COM_PORT.print("\tV=");
           DEBUG_COM_PORT.print(selected_axis->totalSpeed,DEC);
           DEBUG_COM_PORT.print("\txCMD=");
           DEBUG_COM_PORT.print(xSpeed,DEC);
           DEBUG_COM_PORT.print("\tyCMD=");
           DEBUG_COM_PORT.print(ySpeed,DEC);
           DEBUG_COM_PORT.print("\tzCMD=");
           DEBUG_COM_PORT.print(zSpeed,DEC);
           DEBUG_COM_PORT.print("\n");

          break;
      }
       DEBUG_COM_PORT.print("\n");
    }
  }
 #endif
