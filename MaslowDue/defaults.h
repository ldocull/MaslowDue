/*
  defaults.h - defaults settings configuration file
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

/* The defaults.h file serves as a central default settings selector for different machine
   types, from DIY CNC mills to CNC conversions of off-the-shelf machines. The settings 
   files listed here are supplied by users, so your results may vary. However, this should
   give you a good starting point as you get to know your machine and tweak the settings for
   your nefarious needs.
   Ensure one and only one of these DEFAULTS_XXX values is defined in config.h */

#ifndef defaults_h

// Only define the DEFAULT_XXX with where to find the corresponding default_XXX.h file.
// Don't #define defaults_h here, let the selected file do it. Prevents including more than one.

#ifdef DEFAULTS_GENERIC
  // Grbl generic default settings. Should work across different machines.
  #include "defaults_generic.h"
#endif

#endif
