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
#ifndef INCLUDE_MATLABSCALAR
#define INCLUDE_MATLABSCALAR

#include "mex.h"

// Class MatlabScalar
// -----------------------------------------------------------------
// The main appeal of this class is that one can create a scalar
// object that accesses a MATLAB array.
//
// Note that the copy assignment operator is not valid for this class
// because we cannot reassign a reference.
class MatlabScalar {
public:

  // This constructor accepts as input a pointer to a Matlab array
  // which must be a scalar in double precision.
  explicit MatlabScalar (const mxArray* ptr);
  
  // This constructor creates a new Matlab array which is a scalar
  // in double precision.
  MatlabScalar (mxArray*& ptr, double value);
  
  // The copy constructor.
  MatlabScalar (MatlabScalar& source);
  
  // The destructor.
  ~MatlabScalar() { };
  
  // Access the value of the scalar.
  operator const double () const { return x; };
  
  // Assign the value of the scalar.
  MatlabScalar& operator= (double value);
  
protected:
  double& x;
  
  // The copy assignment operator is kept protected because it is
  // invalid.
  MatlabScalar& operator= (const MatlabScalar& source) { return *this; };
};

#endif
