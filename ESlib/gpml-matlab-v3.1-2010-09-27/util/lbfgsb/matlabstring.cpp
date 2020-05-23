// Copyright 2020 Max Planck Society. All rights reserved.
// 
// Author: Alonso Marco Valle (amarcovalle/alonrot) amarco(at)tuebingen.mpg.de
// Affiliation: Max Planck Institute for Intelligent Systems, Autonomous Motion
// Department / Intelligent Control Systems
// 
// This file is part of aLQRtuning_tutorial.
// 
// aLQRtuning_tutorial is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option) any
// later version.
// 
// aLQRtuning_tutorial is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
// details.
// 
// You should have received a copy of the GNU General Public License along with
// aLQRtuning_tutorial.  If not, see <http://www.gnu.org/licenses/>.
//
//
#include "matlabstring.h"
#include "matlabexception.h"

// Function definitions.
// -----------------------------------------------------------------
char* copystring (const char* source) {
  int   n    = strlen(source);  // The length of the string.
  char* dest = new char[n+1];   // The return value.
  strcpy(dest,source);
  return dest;
}

// Function definitions for class MatlabString.
// -----------------------------------------------------------------
MatlabString::MatlabString (const mxArray* ptr) {
  s = 0;

  // Check to make sure the Matlab array is a string.
  if (!mxIsChar(ptr))
    throw MatlabException("Matlab array must be a string (of type CHAR)");
  
  // Get the string passed as a Matlab array.
  s = mxArrayToString(ptr);
  if (s == 0)
    throw MatlabException("Unable to obtain string from Matlab array");
}

MatlabString::MatlabString (const MatlabString& source) {
  s = 0;
  
  // Copy the source string.
  s = copystring(source.s);
}

MatlabString::~MatlabString() { 
  if (s) 
    mxFree(s); 
}

