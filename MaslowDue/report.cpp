/*
  report.c - reporting and messaging methods
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

/* 
  This file functions as the primary feedback interface for Grbl. Any outgoing data, such 
  as the protocol status messages, feedback messages, and status reports, are stored here.
  For the most part, these functions primarily are called from protocol.c methods. If a 
  different style feedback is desired (i.e. JSON), then a user can change these following 
  methods to accomodate their needs.
*/

#include "grbl.h"

// Handles the primary confirmation protocol response for streaming interfaces and human-feedback.
// For every incoming line, this method responds with an 'ok' for a successful command or an 
// 'error:'  to indicate some error event with the line or some critical system error during 
// operation. Errors events can originate from the g-code parser, settings module, or asynchronously
// from a critical error, such as a triggered hard limit. Interface should always monitor for these
// responses.
// NOTE: In silent mode, all error codes are greater than zero.
// TODO: Install silent mode to return only numeric values, primarily for GUIs.
void report_status_message(uint8_t status_code) 
{
  if (status_code == 0) { // STATUS_OK
    printPgmString( ("ok\r\n"));
  } else {
    printPgmString( ("error: "));
    #ifdef REPORT_GUI_MODE
      print_uint8_base10(status_code);
    #else
      switch(status_code) {          
        case STATUS_EXPECTED_COMMAND_LETTER:
        printPgmString( ("Expected command letter")); break;
        case STATUS_BAD_NUMBER_FORMAT:
        printPgmString( ("Bad number format")); break;
        case STATUS_INVALID_STATEMENT:
        printPgmString( ("Invalid statement")); break;
        case STATUS_NEGATIVE_VALUE:
        printPgmString( ("Value < 0")); break;
        case STATUS_SETTING_DISABLED:
        printPgmString( ("Setting disabled")); break;
        case STATUS_SETTING_STEP_PULSE_MIN:
        printPgmString( ("Value < 3 usec")); break;
        case STATUS_SETTING_READ_FAIL:
        printPgmString( ("EEPROM read fail. Using defaults")); break;
        case STATUS_IDLE_ERROR:
        printPgmString( ("Not idle")); break;
        case STATUS_ALARM_LOCK:
        printPgmString( ("Alarm lock")); break;
        case STATUS_SOFT_LIMIT_ERROR:
        printPgmString( ("Homing not enabled")); break;
        case STATUS_OVERFLOW:
        printPgmString( ("Line overflow")); break;
        #ifdef MAX_STEP_RATE_HZ
          case STATUS_MAX_STEP_RATE_EXCEEDED: 
          printPgmString( ("Step rate > 30kHz")); break;
        #endif      
        // Common g-code parser errors.
        case STATUS_GCODE_MODAL_GROUP_VIOLATION:
        printPgmString( ("Modal group violation")); break;
        case STATUS_GCODE_UNSUPPORTED_COMMAND:
        printPgmString( ("Unsupported command")); break;
        case STATUS_GCODE_UNDEFINED_FEED_RATE:
        printPgmString( ("Undefined feed rate")); break;
        default:
          // Remaining g-code parser errors with error codes
          printPgmString( ("Invalid gcode ID:"));
          print_uint8_base10(status_code); // Print error code for user reference
      }
    #endif  
    printPgmString( ("\r\n"));
  }
}

// Prints alarm messages.
void report_alarm_message(int8_t alarm_code)
{
  printPgmString( ("ALARM: "));
  #ifdef REPORT_GUI_MODE
    print_uint8_base10(alarm_code);
  #else
    switch (alarm_code) {
      case ALARM_HARD_LIMIT_ERROR: 
      printPgmString( ("Hard limit")); break;
      case ALARM_SOFT_LIMIT_ERROR:
      printPgmString( ("Soft limit")); break;
      case ALARM_ABORT_CYCLE: 
      printPgmString( ("Abort during cycle")); break;
      case ALARM_PROBE_FAIL:
      printPgmString( ("Probe fail")); break;
      case ALARM_HOMING_FAIL:
      printPgmString( ("Homing fail")); break;
    }
  #endif
  printPgmString( ("\r\n"));
  delay_ms(500); // Force delay to ensure message clears serial write buffer.
}

// Prints feedback messages. This serves as a centralized method to provide additional
// user feedback for things that are not of the status/alarm message protocol. These are
// messages such as setup warnings, switch toggling, and how to exit alarms.
// NOTE: For interfaces, messages are always placed within brackets. And if silent mode
// is installed, the message number codes are less than zero.
// TODO: Install silence feedback messages option in settings
void report_feedback_message(uint8_t message_code)
{
  printPgmString( ("["));
  switch(message_code) {
    case MESSAGE_CRITICAL_EVENT:
    printPgmString( ("Reset to continue")); break;
    case MESSAGE_ALARM_LOCK:
    printPgmString( ("'$H'|'$X' to unlock")); break;
    case MESSAGE_ALARM_UNLOCK:
    printPgmString( ("Caution: Unlocked")); break;
    case MESSAGE_ENABLED:
    printPgmString( ("Enabled")); break;
    case MESSAGE_DISABLED:
    printPgmString( ("Disabled")); break; 
    case MESSAGE_SAFETY_DOOR_AJAR:
    printPgmString( ("Check Door")); break;
    case MESSAGE_PROGRAM_END:
    printPgmString( ("Pgm End")); break;
    case MESSAGE_RESTORE_DEFAULTS:
    printPgmString( ("Restoring defaults")); break;
  }
  printPgmString( ("]\r\n"));
}


// Welcome message
void report_init_message()
{
  printPgmString( ("\r\nGrbl " GRBL_VERSION " ['$' for help]\r\n"));
}

// Grbl help message
void report_grbl_help() {
  #ifndef REPORT_GUI_MODE
    printPgmString( ("$$ (view Grbl settings)\r\n"
                        "$# (view # parameters)\r\n"
                        "$G (view parser state)\r\n"
                        "$I (view build info)\r\n"
                        "$N (view startup blocks)\r\n"
                        "$x=value (save Grbl setting)\r\n"
                        "$Nx=line (save startup block)\r\n"
                        "$C (check gcode mode)\r\n"
                        "$X (kill alarm lock)\r\n"
                        "$H (run homing cycle)\r\n"
                        "~ (cycle start)\r\n"
                        "! (feed hold)\r\n"
                        "? (current status)\r\n"
                        "ctrl-x (reset Grbl)\r\n"));
  #endif
}


// Grbl global settings print out.
// NOTE: The numbering scheme here must correlate to storing in settings.c
void report_grbl_settings() {
  // Print Grbl settings.
  #ifdef REPORT_GUI_MODE
    printPgmString( ("$0=")); print_uint8_base10(settings.pulse_microseconds);
    printPgmString( ("\r\n$1=")); print_uint8_base10(settings.stepper_idle_lock_time);
    printPgmString( ("\r\n$2=")); print_uint8_base10(settings.step_invert_mask); 
    printPgmString( ("\r\n$3=")); print_uint8_base10(settings.dir_invert_mask); 
    printPgmString( ("\r\n$4=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE));
    printPgmString( ("\r\n$5=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS));
    printPgmString( ("\r\n$6=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_INVERT_PROBE_PIN));
    printPgmString( ("\r\n$10=")); print_uint8_base10(settings.status_report_mask);
    printPgmString( ("\r\n$11=")); printFloat_SettingValue(settings.junction_deviation);
    printPgmString( ("\r\n$12=")); printFloat_SettingValue(settings.arc_tolerance);
    printPgmString( ("\r\n$13=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_REPORT_INCHES));
    printPgmString( ("\r\n$20=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE));
    printPgmString( ("\r\n$21=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE));
    printPgmString( ("\r\n$22=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE));
    printPgmString( ("\r\n$23=")); print_uint8_base10(settings.homing_dir_mask);
    printPgmString( ("\r\n$24=")); printFloat_SettingValue(settings.homing_feed_rate);
    printPgmString( ("\r\n$25=")); printFloat_SettingValue(settings.homing_seek_rate);
    printPgmString( ("\r\n$26=")); print_uint8_base10(settings.homing_debounce_delay);
    printPgmString( ("\r\n$27=")); printFloat_SettingValue(settings.homing_pulloff);

  // x-axis PID
    printPgmString( ("\r\n$40=")); print_uint32_base10(settings.x_PID_Kp);
    printPgmString( ("\r\n$41=")); print_uint32_base10(settings.x_PID_Ki);
    printPgmString( ("\r\n$42=")); print_uint32_base10(settings.x_PID_Kd);
    printPgmString( ("\r\n$43=")); print_uint32_base10(settings.x_PID_Imax);

  // y-axis PID
    printPgmString( ("\r\n$50=")); print_uint32_base10(settings.y_PID_Kp);
    printPgmString( ("\r\n$51=")); print_uint32_base10(settings.y_PID_Ki);
    printPgmString( ("\r\n$52=")); print_uint32_base10(settings.y_PID_Kd);
    printPgmString( ("\r\n$53=")); print_uint32_base10(settings.y_PID_Imax);

  // z-axis PID
    printPgmString( ("\r\n$60=")); print_uint32_base10(settings.z_PID_Kp);
    printPgmString( ("\r\n$61=")); print_uint32_base10(settings.z_PID_Ki);
    printPgmString( ("\r\n$62=")); print_uint32_base10(settings.z_PID_Kd);
    printPgmString( ("\r\n$63=")); print_uint32_base10(settings.z_PID_Imax);

    #ifdef MASLOWCNC
      printPgmString( ("\r\n$80=")); print_uint32_base10(settings.chainOverSprocket);
      printPgmString( ("\r\n$81=")); printFloat_SettingValue(settings.machineWidth);   /* Maslow specific settings */
      printPgmString( ("\r\n$82=")); printFloat_SettingValue(settings.machineHeight);
      printPgmString( ("\r\n$83=")); printFloat_SettingValue(settings.distBetweenMotors);
      printPgmString( ("\r\n$84=")); printFloat_SettingValue(settings.motorOffsetY);
      printPgmString( ("\r\n$85=")); printFloat_SettingValue(settings.XcorrScaling);
      printPgmString( ("\r\n$86=")); printFloat_SettingValue(settings.YcorrScaling);
    #endif

    printPgmString( ("\r\n"));
  #else      
    printPgmString( ("$0=")); print_uint8_base10(settings.pulse_microseconds);
    printPgmString( (" (step pulse, usec)\r\n$1=")); print_uint8_base10(settings.stepper_idle_lock_time);
    printPgmString( (" (step idle delay, msec)\r\n$2=")); print_uint8_base10(settings.step_invert_mask); 
    printPgmString( (" (step port invert mask:")); print_uint8_base2(settings.step_invert_mask);  
    printPgmString( (")\r\n$3=")); print_uint8_base10(settings.dir_invert_mask); 
    printPgmString( (" (dir port invert mask:")); print_uint8_base2(settings.dir_invert_mask);  
    printPgmString( (")\r\n$4=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE));
    printPgmString( (" (step enable invert, bool)\r\n$5=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS));
    printPgmString( (" (limit pins invert, bool)\r\n$6=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_INVERT_PROBE_PIN));
    printPgmString( (" (probe pin invert, bool)\r\n$10=")); print_uint8_base10(settings.status_report_mask);
    printPgmString( (" (status report mask:")); print_uint8_base2(settings.status_report_mask);
    printPgmString( (")\r\n$11=")); printFloat_SettingValue(settings.junction_deviation);
    printPgmString( (" (junction deviation, mm)\r\n$12=")); printFloat_SettingValue(settings.arc_tolerance);
    printPgmString( (" (arc tolerance, mm)\r\n$13=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_REPORT_INCHES));
    printPgmString( (" (report inches, bool)\r\n$20=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE));
    printPgmString( (" (soft limits, bool)\r\n$21=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE));
    printPgmString( (" (hard limits, bool)\r\n$22=")); print_uint8_base10(bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE));
    printPgmString( (" (homing cycle, bool)\r\n$23=")); print_uint8_base10(settings.homing_dir_mask);
    printPgmString( (" (homing dir invert mask:")); print_uint8_base2(settings.homing_dir_mask);  
    printPgmString( (")\r\n$24=")); printFloat_SettingValue(settings.homing_feed_rate);
    printPgmString( (" (homing feed, mm/min)\r\n$25=")); printFloat_SettingValue(settings.homing_seek_rate);
    printPgmString( (" (homing seek, mm/min)\r\n$26=")); print_uint8_base10(settings.homing_debounce_delay);
    printPgmString( (" (homing debounce, msec)\r\n$27=")); printFloat_SettingValue(settings.homing_pulloff);
    printPgmString( (" (homing pull-off, mm)\r\n$40="));
      // x-axis PID
       print_uint32_base10(settings.x_PID_Kp);
    printPgmString( (" (X-axis Kp) \r\n$41=")); print_uint32_base10(settings.x_PID_Ki);
    printPgmString( (" (X-axis Ki) \r\n$42=")); print_uint32_base10(settings.x_PID_Kd);
    printPgmString( (" (X-axis Kd) \r\n$43=")); print_uint32_base10(settings.x_PID_Imax);
    printPgmString( (" (X-axis Imax) \r\n$50="));

  // y-axis PID
      print_uint32_base10(settings.y_PID_Kp);
    printPgmString( (" (Y-axis Kp) \r\n$51=")); print_uint32_base10(settings.y_PID_Ki); 
    printPgmString( (" (Y-axis Ki) \r\n$52=")); print_uint32_base10(settings.y_PID_Kd);
    printPgmString( (" (Y-axis Kd) \r\n$53=")); print_uint32_base10(settings.y_PID_Imax);
    printPgmString( (" (Y-axis Imax) \r\n$60="));

  // z-axis PID
       print_uint32_base10(settings.z_PID_Kp);
    printPgmString( (" (Z-axis Kp) \r\n$61=")); print_uint32_base10(settings.z_PID_Ki);
    printPgmString( (" (Z-axis Ki) \r\n$62=")); print_uint32_base10(settings.z_PID_Kd);
    printPgmString( (" (Z-axis Kd) \r\n$63=")); print_uint32_base10(settings.z_PID_Imax);
    #ifndef MASLOWCNC
      printPgmString( (" (Z-axis Imax) \r\n"));
    #endif
    
    #ifdef MASLOWCNC
      printPgmString( (" (Z-axis Imax) \r\n$80=")); print_uint32_base10(settings.chainOverSprocket);
      printPgmString( (" (Over Sprocket (0/1)) \r\n$81=")); printFloat_SettingValue(settings.machineWidth);   /* Maslow specific settings */
      printPgmString( (" (Bed Width, mm) \r\n$82="));  printFloat_SettingValue(settings.machineHeight);
      printPgmString( (" (Bed Height, mm) \r\n$83=")); printFloat_SettingValue(settings.distBetweenMotors);
      printPgmString( (" (distBetweenMotors, mm) \r\n$84=")); printFloat_SettingValue(settings.motorOffsetY);
      printPgmString( (" (motorOffsetY, mm) \r\n$85=")); printFloat_SettingValue(settings.XcorrScaling);
      printPgmString( (" (XcorrScaling) \r\n$86=")); printFloat_SettingValue(settings.YcorrScaling);
      printPgmString( (" (YcorrScaling) \r\n"));
    #endif
    
  #endif
  
  // Print axis settings
  uint8_t idx, set_idx;
  uint8_t val = AXIS_SETTINGS_START_VAL;
  for (set_idx=0; set_idx<AXIS_N_SETTINGS; set_idx++) {
    for (idx=0; idx<N_AXIS; idx++) {
      printPgmString( ("$"));
      print_uint8_base10(val+idx);
      printPgmString( ("="));
      switch (set_idx) {
        case 0: printFloat_SettingValue(settings.steps_per_mm[idx]); break;
        case 1: printFloat_SettingValue(settings.max_rate[idx]); break;
        case 2: printFloat_SettingValue(settings.acceleration[idx]/(60*60)); break;
        case 3: printFloat_SettingValue(-settings.max_travel[idx]); break;
      }
      #ifdef REPORT_GUI_MODE
        printPgmString( ("\r\n"));
      #else
        printPgmString( (" ("));
        switch (idx) {
          case X_AXIS: printPgmString( ("x")); break;
          case Y_AXIS: printPgmString( ("y")); break;
          case Z_AXIS: printPgmString( ("z")); break;
        }
        switch (set_idx) {
          case 0: printPgmString( (", step/mm")); break;
          case 1: printPgmString( (" max rate, mm/min")); break;
          case 2: printPgmString( (" accel, mm/sec^2")); break;
          case 3: printPgmString( (" max travel, mm")); break;
        }      
        printPgmString( (")\r\n"));
      #endif
    }
    val += AXIS_SETTINGS_INCREMENT;
  }  
}


// Prints current probe parameters. Upon a probe command, these parameters are updated upon a
// successful probe or upon a failed probe with the G38.3 without errors command (if supported). 
// These values are retained until Grbl is power-cycled, whereby they will be re-zeroed.
void report_probe_parameters()
{
  uint8_t i;
  float print_position[N_AXIS];
 
  // Report in terms of machine position.
  printPgmString( ("[PRB:"));
  for (i=0; i< N_AXIS; i++) {
    print_position[i] = system_convert_axis_steps_to_mpos(sys.probe_position,i);
    printFloat_CoordValue(print_position[i]);
    if (i < (N_AXIS-1)) { printPgmString( (",")); }
  }
  printPgmString( (":"));
  print_uint8_base10(sys.probe_succeeded);
  printPgmString( ("]\r\n"));
}


// Prints Grbl NGC parameters (coordinate offsets, probing)
void report_ngc_parameters()
{
  float coord_data[N_AXIS];
  uint8_t coord_select, i;
  for (coord_select = 0; coord_select <= SETTING_INDEX_NCOORD; coord_select++) { 
    if (!(settings_read_coord_data(coord_select,coord_data))) { 
      report_status_message(STATUS_SETTING_READ_FAIL); 
      return;
    } 
    printPgmString( ("[G"));
    switch (coord_select) {
      case 6: printPgmString( ("28")); break;
      case 7: printPgmString( ("30")); break;
      default: print_uint8_base10(coord_select+54); break; // G54-G59
    }  
    printPgmString( (":"));         
    for (i=0; i<N_AXIS; i++) {
      printFloat_CoordValue(coord_data[i]);
      if (i < (N_AXIS-1)) { printPgmString( (",")); }
      else { printPgmString( ("]\r\n")); }
    } 
  }
  printPgmString( ("[G92:")); // Print G92,G92.1 which are not persistent in memory
  for (i=0; i<N_AXIS; i++) {
    printFloat_CoordValue(gc_state.coord_offset[i]);
    if (i < (N_AXIS-1)) { printPgmString( (",")); }
    else { printPgmString( ("]\r\n")); }
  } 
  printPgmString( ("[TLO:")); // Print tool length offset value
  printFloat_CoordValue(gc_state.tool_length_offset);
  printPgmString( ("]\r\n"));
  report_probe_parameters(); // Print probe parameters. Not persistent in memory.
}


// Print current gcode parser mode state
void report_gcode_modes()
{
  printPgmString( ("["));
  
  switch (gc_state.modal.motion) {
    case MOTION_MODE_SEEK : printPgmString( ("G0")); break;
    case MOTION_MODE_LINEAR : printPgmString( ("G1")); break;
    case MOTION_MODE_CW_ARC : printPgmString( ("G2")); break;
    case MOTION_MODE_CCW_ARC : printPgmString( ("G3")); break;
    case MOTION_MODE_NONE : printPgmString( ("G80")); break;
    default: 
      printPgmString( ("G38."));
      print_uint8_base10(gc_state.modal.motion - (MOTION_MODE_PROBE_TOWARD-2));
  }

  printPgmString( (" G"));
  print_uint8_base10(gc_state.modal.coord_select+54);
  
  switch (gc_state.modal.plane_select) {
    case PLANE_SELECT_XY : printPgmString( (" G17")); break;
    case PLANE_SELECT_ZX : printPgmString( (" G18")); break;
    case PLANE_SELECT_YZ : printPgmString( (" G19")); break;
  }
  
  if (gc_state.modal.units == UNITS_MODE_MM) { printPgmString( (" G21")); }
  else { printPgmString( (" G20")); }
  
  if (gc_state.modal.distance == DISTANCE_MODE_ABSOLUTE) { printPgmString( (" G90")); }
  else { printPgmString( (" G91")); }
  
  if (gc_state.modal.feed_rate == FEED_RATE_MODE_INVERSE_TIME) { printPgmString( (" G93")); }
  else { printPgmString( (" G94")); }
    
  switch (gc_state.modal.program_flow) {
    case PROGRAM_FLOW_RUNNING : printPgmString( (" M0")); break;
    case PROGRAM_FLOW_PAUSED : printPgmString( (" M1")); break;
    case PROGRAM_FLOW_COMPLETED : printPgmString( (" M2")); break;
  }

  switch (gc_state.modal.spindle) {
    case SPINDLE_ENABLE_CW : printPgmString( (" M3")); break;
    case SPINDLE_ENABLE_CCW : printPgmString( (" M4")); break;
    case SPINDLE_DISABLE : printPgmString( (" M5")); break;
  }
  
  switch (gc_state.modal.coolant) {
    case COOLANT_DISABLE : printPgmString( (" M9")); break;
    case COOLANT_FLOOD_ENABLE : printPgmString( (" M8")); break;
    #ifdef ENABLE_M7
      case COOLANT_MIST_ENABLE : printPgmString( (" M7")); break;
    #endif
  }
  
  printPgmString( (" T"));
  print_uint8_base10(gc_state.tool);
  
  printPgmString( (" F"));
  printFloat_RateValue(gc_state.feed_rate);
  
  #ifdef VARIABLE_SPINDLE
    printPgmString( (" S"));
    printFloat_RateValue(gc_state.spindle_speed);
  #endif

  printPgmString( ("]\r\n"));
}

// Prints specified startup line
void report_startup_line(uint8_t n, char *line)
{
  printPgmString( ("$N")); print_uint8_base10(n);
  printPgmString( ("=")); printString(line);
  printPgmString( ("\r\n"));
}


// Prints build info line
void report_build_info(char *line)
{
  printPgmString( ("[" GRBL_VERSION "." GRBL_VERSION_BUILD ":"));
  printString(line);
  printPgmString( ("]\r\n"));
}


// Prints the character string line Grbl has received from the user, which has been pre-parsed,
// and has been sent into protocol_execute_line() routine to be executed by Grbl.
void report_echo_line_received(char *line)
{
  printPgmString( ("[echo: ")); printString(line);
  printPgmString( ("]\r\n"));
}


 // Prints real-time data. This function grabs a real-time snapshot of the stepper subprogram 
 // and the actual location of the CNC machine. Users may change the following function to their
 // specific needs, but the desired real-time data report must be as short as possible. This is
 // requires as it minimizes the computational overhead and allows grbl to keep running smoothly, 
 // especially during g-code programs with fast, short line segments and high frequency reports (5-20Hz).
void report_realtime_status()
{
  // **Under construction** Bare-bones status report. Provides real-time machine position relative to 
  // the system power on location (0,0,0) and work coordinate position (G54 and G92 applied). Eventually
  // to be added are distance to go on block, processed block id, and feed rate. Also a settings bitmask
  // for a user to select the desired real-time data.
  uint8_t idx;
  int32_t current_position[N_AXIS]; // Copy current state of the system position variable
  memcpy(current_position,sys.position,sizeof(sys.position));
  float print_position[N_AXIS];
 
  // Report current machine state
  switch (sys.state) {
    case STATE_IDLE: printPgmString( ("<Idle")); break;
    case STATE_MOTION_CANCEL: // Report run state.
    case STATE_CYCLE: printPgmString( ("<Run")); break;
    case STATE_HOLD: printPgmString( ("<Hold")); break;
    case STATE_HOMING: printPgmString( ("<Home")); break;
    case STATE_ALARM: printPgmString( ("<Alarm")); break;
    case STATE_CHECK_MODE: printPgmString( ("<Check")); break;
    case STATE_SAFETY_DOOR: printPgmString( ("<Door")); break;
  }
 
  // If reporting a position, convert the current step count (current_position) to millimeters.
  if (bit_istrue(settings.status_report_mask,(BITFLAG_RT_STATUS_MACHINE_POSITION | BITFLAG_RT_STATUS_WORK_POSITION))) {
    system_convert_array_steps_to_mpos(print_position,current_position);
  }
  
  // Report machine position
  if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_MACHINE_POSITION)) {
    printPgmString( (",MPos:")); 
    for (idx=0; idx< N_AXIS; idx++) {
      printFloat_CoordValue(print_position[idx]);
      if (idx < (N_AXIS-1)) { printPgmString( (",")); }
    }
  }
  
  // Report work position
  if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_WORK_POSITION)) {
    printPgmString( (",WPos:")); 
    for (idx=0; idx< N_AXIS; idx++) {
      // Apply work coordinate offsets and tool length offset to current position.
      print_position[idx] -= gc_state.coord_system[idx]+gc_state.coord_offset[idx];
      if (idx == TOOL_LENGTH_OFFSET_AXIS) { print_position[idx] -= gc_state.tool_length_offset; }    
      printFloat_CoordValue(print_position[idx]);
      if (idx < (N_AXIS-1)) { printPgmString( (",")); }
    }
  }
        
  // Returns the number of active blocks are in the planner buffer.
  if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_PLANNER_BUFFER)) {
    printPgmString( (",Buf:"));
    print_uint8_base10(plan_get_block_buffer_count());
  }

  // Report serial read buffer status
  if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_SERIAL_RX)) {
    printPgmString( (",RX:"));
    print_uint8_base10(serial_get_rx_buffer_count());
  }
    
  #ifdef USE_LINE_NUMBERS
    // Report current line number
    printPgmString( (",Ln:")); 
    int32_t ln=0;
    plan_block_t * pb = plan_get_current_block();
    if(pb != NULL) {
      ln = pb->line_number;
    } 
    printInteger(ln);
  #endif
    
  #ifdef REPORT_REALTIME_RATE
    // Report realtime rate 
    printPgmString( (",F:")); 
    printFloat_RateValue(st_get_realtime_rate());
  #endif    
  
  if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_LIMIT_PINS)) {
    printPgmString( (",Lim:"));
    print_unsigned_int8(limits_get_state(),2,N_AXIS);
  }
  
  #ifdef REPORT_CONTROL_PIN_STATE 
    printPgmString( (",Ctl:"));
    print_uint8_base2(CONTROL_PIN & CONTROL_MASK);
  #endif
  
  printPgmString( (">\r\n"));
}
