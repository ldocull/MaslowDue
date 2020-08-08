/*
  system.c - Handles system level commands and real-time processes
  Part of Grbl

  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

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

#include "grbl.h"

#ifdef MASLOWCNC
  #include "MaslowDue.h"

  #define SPROCKET_RADIUS_MM      (10.1)

  #define KINEMATICSMAXGUESS 200
  // #define KINEMATICSDBG 1 // output to serial while computing kinematics.
  #define KINEMATICSMAXERR 0.1 // maximum error value in forward kinematics. bigger = faster.

  // Main kinematics functions.
  void  triangularInverse(float xTarget, float yTarget, float* aChainLength, float* bChainLength);
  void  forwardKinematics(float chainALength, float chainBLength, float* xPos, float* yPos);
  void  triangular(float aChainLength, float bChainLength, float *x,float *y );
  void _recomputeGeometry(void);
  void _verifyValidTarget(float* xTarget,float* yTarget);

  // Various cached pre-computed values.
  float height_to_bit; //distance between sled attach point and bit
  float R = SPROCKET_RADIUS_MM;                      //sprocket radius
  float halfWidth;                     //Half the machine width
  float halfHeight;                    //Half the machine height
  float _xCordOfMotor;
  float _yCordOfMotor;

  // Cached between forwardKinematic computations to provide a good guess (performance).
  float _xLastPosition;
  float _yLastPosition;
#endif

void system_init()
{
  #ifndef MASLOWCNC
    CONTROL_DDR &= ~(CONTROL_MASK); // Configure as input pins
    #ifdef DISABLE_CONTROL_PIN_PULL_UP
      CONTROL_PORT &= ~(CONTROL_MASK); // Normal low operation. Requires external pull-down.
    #else
      CONTROL_PORT |= CONTROL_MASK;   // Enable internal pull-up resistors. Normal high operation.
    #endif
    CONTROL_PCMSK |= CONTROL_MASK;  // Enable specific pins of the Pin Change Interrupt
    PCICR |= (1 << CONTROL_INT);   // Enable Pin Change Interrupt
  #endif
}


// Returns control pin state as a uint8 bitfield. Each bit indicates the input pin state, where
// triggered is 1 and not triggered is 0. Invert mask is applied. Bitfield organization is
// defined by the CONTROL_PIN_INDEX in the header file.
uint8_t system_control_get_state()
{
    uint8_t control_state = 0;
  #ifndef MASLOWCNC
    uint8_t pin = (CONTROL_PIN & CONTROL_MASK);
    #ifdef INVERT_CONTROL_PIN_MASK
      pin ^= INVERT_CONTROL_PIN_MASK;
    #endif
    if (pin) {
      if (bit_isfalse(pin,(1<<CONTROL_SAFETY_DOOR_BIT))) { control_state |= CONTROL_PIN_INDEX_SAFETY_DOOR; }
      if (bit_isfalse(pin,(1<<CONTROL_RESET_BIT))) { control_state |= CONTROL_PIN_INDEX_RESET; }
      if (bit_isfalse(pin,(1<<CONTROL_FEED_HOLD_BIT))) { control_state |= CONTROL_PIN_INDEX_FEED_HOLD; }
      if (bit_isfalse(pin,(1<<CONTROL_CYCLE_START_BIT))) { control_state |= CONTROL_PIN_INDEX_CYCLE_START; }
    }
  #endif
  return(control_state);
}


// Pin change interrupt for pin-out commands, i.e. cycle start, feed hold, and reset. Sets
// only the realtime command execute variable to have the main program execute these when
// its ready. This works exactly like the character-based realtime commands when picked off
// directly from the incoming serial data stream.
#ifndef MASLOWCNC
ISR(CONTROL_INT_vect)
{
  uint8_t pin = system_control_get_state();
  if (pin) {
    if (bit_istrue(pin,CONTROL_PIN_INDEX_RESET)) {
      mc_reset();
    } else if (bit_istrue(pin,CONTROL_PIN_INDEX_CYCLE_START)) {
      bit_true(sys_rt_exec_state, EXEC_CYCLE_START);
    } else if (bit_istrue(pin,CONTROL_PIN_INDEX_FEED_HOLD)) {
      bit_true(sys_rt_exec_state, EXEC_FEED_HOLD);
    } else if (bit_istrue(pin,CONTROL_PIN_INDEX_SAFETY_DOOR)) {
      bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
    }
  }
}
#endif

// Returns if safety door is ajar(T) or closed(F), based on pin state.
uint8_t system_check_safety_door_ajar()
{
  #ifdef MASLOWCNC
    return 0;   // no door on my Maslow..
  #endif
    return(system_control_get_state() & CONTROL_PIN_INDEX_SAFETY_DOOR);
}


// Executes user startup script, if stored.
void system_execute_startup(char *line)
{
  uint8_t n;
  for (n=0; n < N_STARTUP_LINE; n++) {
    if (!(settings_read_startup_line(n, line))) {
      line[0] = 0;
      report_execute_startup_message(line,STATUS_SETTING_READ_FAIL);
    } else {
      if (line[0] != 0) {
        uint8_t status_code = gc_execute_line(line);
        report_execute_startup_message(line,status_code);
      }
    }
  }
}


// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also executes Grbl internal commands, such as
// settings, initiating the homing cycle, and toggling switch states. This differs from
// the realtime command module by being susceptible to when Grbl is ready to execute the
// next line during a cycle, so for switches like block delete, the switch only effects
// the lines that are processed afterward, not necessarily real-time during a cycle,
// since there are motions already stored in the buffer. However, this 'lag' should not
// be an issue, since these commands are not typically used during a cycle.
uint8_t system_execute_line(char *line)
{
  uint8_t char_counter = 1;
  uint8_t helper_var = 0; // Helper variable
  float parameter, value;
  switch( line[char_counter] ) {
    case 0 : report_grbl_help(); break;
    case 'J' : // Jogging
      // Execute only if in IDLE or JOG states.
      if (sys.state != STATE_IDLE && sys.state != STATE_JOG) { return(STATUS_IDLE_ERROR); }
      if(line[2] != '=') { return(STATUS_INVALID_STATEMENT); }
      return(gc_execute_line(line)); // NOTE: $J= is ignored inside g-code parser and used to detect jog motions.
      break;
    case '$': case 'G': case 'C': case 'X':
      if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      switch( line[1] ) {
        case '$' : // Prints Grbl settings
          if ( sys.state & (STATE_CYCLE | STATE_HOLD) ) { return(STATUS_IDLE_ERROR); } // Block during cycle. Takes too long to print.
          else { report_grbl_settings(); }
          break;
        case 'G' : // Prints gcode parser state
          // TODO: Move this to realtime commands for GUIs to request this data during suspend-state.
          report_gcode_modes();
          break;
        case 'C' : // Set check g-code mode [IDLE/CHECK]
          // Perform reset when toggling off. Check g-code mode should only work if Grbl
          // is idle and ready, regardless of alarm locks. This is mainly to keep things
          // simple and consistent.
          if ( sys.state == STATE_CHECK_MODE ) {
            mc_reset();
            report_feedback_message(MESSAGE_DISABLED);
          } else {
            if (sys.state) { return(STATUS_IDLE_ERROR); } // Requires no alarm mode.
            sys.state = STATE_CHECK_MODE;
            report_feedback_message(MESSAGE_ENABLED);
          }
          break;
        case 'X' : // Disable alarm lock [ALARM]
          if (sys.state == STATE_ALARM) {
            // Block if safety door is ajar.
            if (system_check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); }
            report_feedback_message(MESSAGE_ALARM_UNLOCK);
            sys.state = STATE_IDLE;
            // Don't run startup script. Prevents stored moves in startup from causing accidents.
          } // Otherwise, no effect.
          break;
      }
      break;
    default :
      // Block any system command that requires the state as IDLE/ALARM. (i.e. EEPROM, homing)
      if ( !(sys.state == STATE_IDLE || sys.state == STATE_ALARM) ) { return(STATUS_IDLE_ERROR); }
      switch( line[1] ) {
        #ifdef MASLOWCNC
          case '|':
            // EEPROM diagnostic Viewer
            EEPROM_viewer();
            break;
        #endif
        case '#' : // Print Grbl NGC parameters
          if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); }
          else { report_ngc_parameters(); }
          break;
        case 'H' : // Perform homing cycle [IDLE/ALARM]
          if (bit_isfalse(settings.flags,BITFLAG_HOMING_ENABLE)) {return(STATUS_SETTING_DISABLED); }
          if (system_check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); } // Block if safety door is ajar.
          sys.state = STATE_HOMING; // Set system state variable
          if (line[2] == 0) {
            mc_homing_cycle(HOMING_CYCLE_ALL);
          #ifdef HOMING_SINGLE_AXIS_COMMANDS
            } else if (line[3] == 0) {
              switch (line[2]) {
                case 'X': mc_homing_cycle(HOMING_CYCLE_X); break;
                case 'Y': mc_homing_cycle(HOMING_CYCLE_Y); break;
                case 'Z': mc_homing_cycle(HOMING_CYCLE_Z); break;
                default: return(STATUS_INVALID_STATEMENT);
              }
          #endif
          } else { return(STATUS_INVALID_STATEMENT); }
          if (!sys.abort) {  // Execute startup scripts after successful homing.
            sys.state = STATE_IDLE; // Set to IDLE when complete.
            st_go_idle(); // Set steppers to the settings idle state before returning.
            if (line[2] == 0) { system_execute_startup(line); }
          }
          break;
        case 'S' : // Puts Grbl to sleep [IDLE/ALARM]
          if ((line[2] != 'L') || (line[3] != 'P') || (line[4] != 0)) { return(STATUS_INVALID_STATEMENT); }
          system_set_exec_state_flag(EXEC_SLEEP); // Set to execute sleep mode immediately
          #ifdef MASLOWCNC
          motorsDisabled();
          #endif
          break;
        case 'I' : // Print or store build info. [IDLE/ALARM]
          if ( line[++char_counter] == 0 ) {
            settings_read_build_info(line);
            report_build_info(line);
          #ifdef ENABLE_BUILD_INFO_WRITE_COMMAND
            } else { // Store startup line [IDLE/ALARM]
              if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
              helper_var = char_counter; // Set helper variable as counter to start of user info line.
              do {
                line[char_counter-helper_var] = line[char_counter];
              } while (line[char_counter++] != 0);
              settings_store_build_info(line);
          #endif
          }
          break;
        case 'R' : // Restore defaults [IDLE/ALARM]
          if ((line[2] != 'S') || (line[3] != 'T') || (line[4] != '=') || (line[6] != 0)) { return(STATUS_INVALID_STATEMENT); }
          switch (line[5]) {
            #ifdef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS
              case '$': settings_restore(SETTINGS_RESTORE_DEFAULTS); break;
            #endif
            #ifdef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS
              case '#': settings_restore(SETTINGS_RESTORE_PARAMETERS); break;
            #endif
            #ifdef ENABLE_RESTORE_EEPROM_WIPE_ALL
              case '*': settings_restore(SETTINGS_RESTORE_ALL); break;
            #endif
            default: return(STATUS_INVALID_STATEMENT);
          }
          report_feedback_message(MESSAGE_RESTORE_DEFAULTS);
          mc_reset(); // Force reset to ensure settings are initialized correctly.
          break;
        case 'N' : // Startup lines. [IDLE/ALARM]
          if ( line[++char_counter] == 0 ) { // Print startup lines
            for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++) {
              if (!(settings_read_startup_line(helper_var, line))) {
                report_status_message(STATUS_SETTING_READ_FAIL);
              } else {
                report_startup_line(helper_var,line);
              }
            }
            break;
          } else { // Store startup line [IDLE Only] Prevents motion during ALARM.
            if (sys.state != STATE_IDLE) { return(STATUS_IDLE_ERROR); } // Store only when idle.
            helper_var = true;  // Set helper_var to flag storing method.
            // No break. Continues into default: to read remaining command characters.
          }
        default :  // Storing setting methods [IDLE/ALARM]
          if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
          if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
          if (helper_var) { // Store startup line
            // Prepare sending gcode block to gcode parser by shifting all characters
            helper_var = char_counter; // Set helper variable as counter to start of gcode block
            do {
              line[char_counter-helper_var] = line[char_counter];
            } while (line[char_counter++] != 0);
            if (char_counter > EEPROM_LINE_SIZE) { return(STATUS_LINE_LENGTH_EXCEEDED); }
            // Execute gcode block to ensure block is valid.
            helper_var = gc_execute_line(line); // Set helper_var to returned status code.
            if (helper_var) { return(helper_var); }
            else {
              helper_var = trunc(parameter); // Set helper_var to int value of parameter
              settings_store_startup_line(helper_var,line);
            }
          } else { // Store global setting.
            if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
            if((line[char_counter] != 0) || (parameter > 255)) { return(STATUS_INVALID_STATEMENT); }
            return(settings_store_global_setting((uint8_t)parameter, value));
          }
      }
  }
  return(STATUS_OK); // If '$' command makes it to here, then everything's ok.
}



void system_flag_wco_change()
{
  #ifdef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE
    protocol_buffer_synchronize();
  #endif
  sys.report_wco_counter = 0;
}


// Returns machine position of axis 'idx'. Must be sent a 'step' array.
// NOTE: If motor steps and machine position are not in the same coordinate frame, this function
//   serves as a central place to compute the transformation.
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx)
{
  float pos;
  #ifdef COREXY
    if (idx==X_AXIS) {
      pos = (float)system_convert_corexy_to_x_axis_steps(steps) / settings.steps_per_mm[idx];
    } else if (idx==Y_AXIS) {
      pos = (float)system_convert_corexy_to_y_axis_steps(steps) / settings.steps_per_mm[idx];
    } else {
      pos = steps[idx]/settings.steps_per_mm[idx];
    }
  #else
    #ifdef MASLOWCNC
      if (idx==X_AXIS) {
        pos = (float)system_convert_maslow_to_x_axis_steps(steps) / settings.steps_per_mm[LEFT_MOTOR];
      } else if (idx==Y_AXIS) {
        pos = (float)system_convert_maslow_to_y_axis_steps(steps) / settings.steps_per_mm[RIGHT_MOTOR];
      } else {
        pos = steps[idx]/settings.steps_per_mm[idx];
      }
    #else
      pos = steps[idx]/settings.steps_per_mm[idx];
    #endif
  #endif
  return(pos);
}


void system_convert_array_steps_to_mpos(float *position, int32_t *steps)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    position[idx] = system_convert_axis_steps_to_mpos(steps, idx);
  }
  return;
}


// CoreXY calculation only. Returns x or y-axis "steps" based on CoreXY motor steps.
#ifdef COREXY
  int32_t system_convert_corexy_to_x_axis_steps(int32_t *steps)
  {
    return( (steps[A_MOTOR] + steps[B_MOTOR])/2 );
  }
  int32_t system_convert_corexy_to_y_axis_steps(int32_t *steps)
  {
    return( (steps[A_MOTOR] - steps[B_MOTOR])/2 );
  }
#endif


// Checks and reports if target array exceeds machine travel limits.
uint8_t system_check_travel_limits(float *target)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef HOMING_FORCE_SET_ORIGIN
      // When homing forced set origin is enabled, soft limits checks need to account for directionality.
      // NOTE: max_travel is stored as negative
      if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
        if (target[idx] < 0 || target[idx] > -settings.max_travel[idx]) { return(true); }
      } else {
        if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { return(true); }
      }
    #else
      // NOTE: max_travel is stored as negative
      if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { return(true); }
    #endif
  }
  return(false);
}


#ifdef MASLOWCNC

  void  chainToPosition(float aChainLength, float bChainLength, float *x,float *y ) {
    _recomputeGeometry();

    #if defined (KINEMATICSDBG) && KINEMATICSDBG > 0
      Serial.print(F("Message: chainToPosition(), chainLength: "));
      Serial.print(aChainLength);
      Serial.print(',');
      Serial.print(bChainLength);
      Serial.print(F("; (guess) position: "));
      Serial.print(*x);
      Serial.print(',');
      Serial.println(*y);
    #endif

    return forwardKinematics(aChainLength, bChainLength, x, y);
  }

  void  positionToChain(float xTarget, float yTarget, float* aChainLength, float* bChainLength) {
    _recomputeGeometry();

    return triangularInverse(xTarget, yTarget, aChainLength, bChainLength);
  }

  // recalculate machine base dimensions from settings (in mm)
  void _recomputeGeometry(void)
  {
      /*
      Some variables are computed on class creation for the geometry of the machine to reduce overhead,
      calling this function regenerates those values.
      */
      halfWidth = (settings.machineWidth / 2.0);
      halfHeight = (settings.machineHeight / 2.0);
      _xCordOfMotor = (settings.distBetweenMotors/2);
      _yCordOfMotor = (halfHeight + settings.motorOffsetY);

    #if defined (KINEMATICSDBG) && KINEMATICSDBG > 0
      Serial.print(F("Message: recomputeGeometry(), halfSize: "));
      Serial.print(halfWidth);
      Serial.print(',');
      Serial.print(halfHeight);
      Serial.print(F("; motor position: "));
      Serial.print(_xCordOfMotor);
      Serial.print(',');
      Serial.println(_yCordOfMotor);
    #endif
  }

  // Maslow math - coordinate system tranformation
  // calculate machine coordinate (x-y) postion from chain lengths in mm (pos in mm)
  void triangular(float aChainLength, float bChainLength, float *x,float *y )
  {
    //----------------------------------------------------------------------> arbitrary triangle method:
    //                   cos(B) = ((b^2 + c^2 - a^2) / (2 * b * c))
    //                   theta = arccos(B)
    //                   x = a * cos(theta)
    //                   y = a * sin(theta)
    //
    //     double theta = acos( (pow((_xCordOfMotor * 2), 2) + pow(aChainLength,2) - pow(bChainLength,2)) / (-2.0 * aChainLength * (_xCordOfMotor * 2)));
    //     double x_pos = (aChainLength * cos(theta));
    //     double y_pos = (aChainLength * sin(theta));
    //
    //----------------------------------------------------------------------> intersecting circle method:
    //                    x = (d^2 - R^2 + L^2) / 2 * d
    //                     where d is the distance between motors
    //                    R is right chain length
    //                    L is left chain length
    //                    y^2 = R^2 - x^2
    //
     double x_pos = ((pow((_xCordOfMotor * 2), 2) - pow(bChainLength,2) +  pow(aChainLength,2)) / (2.0 * (_xCordOfMotor * 2)));
     double y_pos = sqrt(pow(aChainLength,2) - pow(x_pos,2));

     x_pos = (float)(-1*_xCordOfMotor) + x_pos;  // apply table offsets to regain absolute position
     y_pos = (float)(_yCordOfMotor - y_pos);

     x_pos /= (double)settings.XcorrScaling;
     y_pos /= (double)settings.YcorrScaling;

     *x = (float) x_pos;
     *y = (float) y_pos;
  }

  // forwardKinematics() are able to compensate for chain sag, an improvement on triangular().
  // It takes an iterative approach to solving for chain sag, attempting to achieve
  // the desired KINEMATICSMAXERR. It is less performant, and care should be used to avoid
  // pegging the limited Arduino CPU.
  void forwardKinematics(float chainALength, float chainBLength, float* xPos, float* yPos)
  {
    float guessLengthA = 0, guessLengthB = 0;
    float xGuess = *xPos, yGuess = *yPos;
    int guessCount = 0;

    while(1){
        //check our guess
        triangularInverse(xGuess, yGuess, &guessLengthA, &guessLengthB);

        float aChainError = chainALength - guessLengthA;
        float bChainError = chainBLength - guessLengthB;

        //adjust the guess based on the result
        xGuess = xGuess + aChainError - bChainError;
        yGuess = yGuess - aChainError - bChainError;

        guessCount++;

        //if we've converged on the point...or it's time to give up, exit the loop
        if ((abs(aChainError) <= KINEMATICSMAXERR && abs(bChainError) <= KINEMATICSMAXERR) or
          guessCount > KINEMATICSMAXGUESS or
          guessLengthA > settings.chainLength or
          guessLengthB > settings.chainLength)
        {
            #if defined (KINEMATICSDBG) && KINEMATICSDBG > 0
              Serial.print(F("Message: forwardKinematics() complete; best guess: "));
              Serial.print(guessLengthA);
              Serial.print(',');
              Serial.print(guessLengthB);
              Serial.print(F("; guessCount: "));
              Serial.println(guessCount);
            #endif

            if((guessCount > KINEMATICSMAXGUESS) or guessLengthA > settings.chainLength or guessLengthB > settings.chainLength){
                Serial.print(F("Message: Unable to find valid machine position for chain lengths "));
                Serial.print(chainALength);
                Serial.print(", ");
                Serial.print(chainBLength);
                Serial.println(F(" . "));
                *xPos = 0;
                *yPos = 0;
            }
            else {
                #if defined (KINEMATICSDBG) && KINEMATICSDBG > 0
                  Serial.println("position loaded at:");
                  Serial.println(xGuess);
                  Serial.println(yGuess);
                #endif
                *xPos = xGuess;
                *yPos = yGuess;
            }
            break;
        }
    }
  }

  // Maslow CNC calculation only. Returns x or y-axis "steps" based on Maslow motor steps.
  // converts current position two-chain intersection (steps) into x / y cartesian in STEPS..
  void system_convert_maslow_to_xy_steps(int32_t *steps, int32_t *x_steps, int32_t *y_steps)
  {
    chainToPosition((float)(steps[LEFT_MOTOR]/settings.steps_per_mm[LEFT_MOTOR]),
                    (float)(steps[RIGHT_MOTOR]/settings.steps_per_mm[RIGHT_MOTOR]),
                    &_xLastPosition, &_yLastPosition);

    *x_steps = (int32_t) _xLastPosition * settings.steps_per_mm[X_AXIS];
    *y_steps = (int32_t) _yLastPosition * settings.steps_per_mm[Y_AXIS];
  }

  int32_t system_convert_maslow_to_x_axis_steps(int32_t *steps)
  {
    int32_t x_steps, y_steps;
    system_convert_maslow_to_xy_steps(steps, &x_steps, &y_steps);
    return(x_steps);
  }

  int32_t system_convert_maslow_to_y_axis_steps(int32_t *steps)
  {
    int32_t x_steps, y_steps;
    system_convert_maslow_to_xy_steps(steps, &x_steps, &y_steps);
    return(y_steps);
  }

  // limit motion to stay within table (in mm)
  void _verifyValidTarget(float* xTarget, float* yTarget)
  {
     // *xTarget = (*xTarget < -halfWidth) ? -halfWidth : (*xTarget > halfWidth) ? halfWidth : *xTarget;
     // *yTarget = (*yTarget < -halfHeight) ? -halfHeight : (*yTarget > halfHeight) ? halfHeight : *yTarget;
  }

  // calculate left and right (LEFT_MOTOR/RIGHT_MOTOR) chain lengths from X-Y cartesian coordinates  (in mm)
  // target is an absolute position in the frame
  void triangularInverse(float xTarget, float yTarget, float* aChainLength, float* bChainLength)
  {
      //Confirm that the coordinates are on the table
      _verifyValidTarget(&xTarget, &yTarget);

      // scale target (absolute position) by any correction factor
      double xxx = (double)xTarget * (double)settings.XcorrScaling;
      double yyy = (double)yTarget * (double)settings.YcorrScaling;

      //Calculate motor axes length to the bit
      float Motor1Distance = sqrt(pow((double)(-1*_xCordOfMotor) - (double)(xxx),2)+pow((double)(_yCordOfMotor) - (double(yyy)),2));
      float Motor2Distance = sqrt(pow((double)   (_xCordOfMotor) - (double)(xxx),2)+pow((double)(_yCordOfMotor) - (double)(yyy),2));

      // Assumes that the chain is over the sprocket (under is not supported)
      float Chain1Angle = asin((_yCordOfMotor - yTarget)/Motor1Distance) + asin(R/Motor1Distance);
      float Chain2Angle = asin((_yCordOfMotor - yTarget)/Motor2Distance) + asin(R/Motor2Distance);
      float Chain1AroundSprocket = R * Chain1Angle;
      float Chain2AroundSprocket = R * Chain2Angle;

      //Calculate the straight chain length from the sprocket to the bit
      float Chain1Straight = sqrt(pow(Motor1Distance,2)-pow(R,2));
      float Chain2Straight = sqrt(pow(Motor2Distance,2)-pow(R,2));

      //Correct the straight chain lengths to account for chain sag
      Chain1Straight *= (1 + ((settings.chainSagCorrection / 1000000000000) * pow(cos(Chain1Angle),2) * pow(Chain1Straight,2) * pow((tan(Chain2Angle) * cos(Chain1Angle)) + sin(Chain1Angle),2)));
      Chain2Straight *= (1 + ((settings.chainSagCorrection / 1000000000000) * pow(cos(Chain2Angle),2) * pow(Chain2Straight,2) * pow((tan(Chain1Angle) * cos(Chain2Angle)) + sin(Chain2Angle),2)));

      //Calculate total chain lengths accounting for sprocket geometry and chain sag
      float Chain1 = Chain1AroundSprocket + Chain1Straight / (1.0f + settings.leftChainTolerance / 100.0f);
      float Chain2 = Chain2AroundSprocket + Chain2Straight / (1.0f + settings.rightChainTolerance / 100.0f);

      //Subtract of the virtual length which is added to the chain by the rotation mechanism
      *aChainLength = Chain1 - settings.rotationDiskRadius;
      *bChainLength = Chain2 - settings.rotationDiskRadius;
  }

#endif

// Special handlers for setting and clearing Grbl's real-time execution flags.
void system_set_exec_state_flag(uint8_t mask) {
  #ifndef MASLOWCNC
    uint8_t sreg = SREG;
    cli();
    sys_rt_exec_state |= (mask);
    SREG = sreg;
  #else
      sys_rt_exec_state |= (mask);
  #endif
}

void system_clear_exec_state_flag(uint8_t mask) {
  #ifndef MASLOWCNC
    uint8_t sreg = SREG;
    cli();
    sys_rt_exec_state &= ~(mask);
    SREG = sreg;
  #else
    sys_rt_exec_state &= ~(mask);
  #endif
}

void system_set_exec_alarm(uint8_t code) {
  #ifndef MASLOWCNC
    uint8_t sreg = SREG;
    cli();
    sys_rt_exec_alarm = code;
    SREG = sreg;
  #else
    sys_rt_exec_alarm = code;
  #endif
}

void system_clear_exec_alarm() {
  #ifndef MASLOWCNC
    uint8_t sreg = SREG;
    cli();
    sys_rt_exec_alarm = 0;
    SREG = sreg;
  #else
    sys_rt_exec_alarm = 0;
  #endif
}

void system_set_exec_motion_override_flag(uint8_t mask) {
  #ifndef MASLOWCNC
    uint8_t sreg = SREG;
    cli();
    sys_rt_exec_motion_override |= (mask);
    SREG = sreg;
  #else
    sys_rt_exec_motion_override |= (mask);
  #endif
}

void system_set_exec_accessory_override_flag(uint8_t mask) {
  #ifndef MASLOWCNC
    uint8_t sreg = SREG;
    cli();
    sys_rt_exec_accessory_override |= (mask);
    SREG = sreg;
  #else
    sys_rt_exec_accessory_override |= (mask);
  #endif
}

void system_clear_exec_motion_overrides() {
  #ifndef MASLOWCNC
    uint8_t sreg = SREG;
    cli();
    sys_rt_exec_motion_override = 0;
    SREG = sreg;
  #else
    sys_rt_exec_motion_override = 0;
  #endif
}

void system_clear_exec_accessory_overrides() {
  #ifndef MASLOWCNC
    uint8_t sreg = SREG;
    cli();
    sys_rt_exec_accessory_override = 0;
    SREG = sreg;
  #else
    sys_rt_exec_accessory_override = 0;
  #endif
}
