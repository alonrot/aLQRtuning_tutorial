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
#ifndef INCLUDE_MATLABSTRING
#define INCLUDE_MATLABSTRING

#include "mex.h"
#include <string.h>

// Function declarations.
// -----------------------------------------------------------------
// Copy a C-style string (i.e. a null-terminated character array).
char* copystring (const char* source);

// Class MatlabString.
// -----------------------------------------------------------------
// This class encapsulates read-only access to a MATLAB character
// array.
class MatlabString {
public:
  
  // The constructor accepts as input a pointer to a Matlab array,
  // which must be a valid string (array of type CHAR).
  explicit MatlabString (const mxArray* ptr);
  
  // The copy constructor makes a full copy of the source string.
  MatlabString (const MatlabString& source);
  
  // The destructor.
  ~MatlabString();
  
  // Conversion operator for null-terminated string.
  operator const char* () const { return s; };
  
protected:
  char* s;  // The null-terminated string.
  
  // The copy assignment operator is not proper, thus remains
  // protected.
  MatlabString& operator= (const MatlabString& source) 
  { return *this; };
};

#endif
