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
#include "matlabscalar.h"
#include "matlabexception.h"

// Function definitions.
// -----------------------------------------------------------------
double& getMatlabScalar (const mxArray* ptr) {
  if (!mxIsDouble(ptr))
    throw MatlabException("Matlab array must be of type double");
  if (mxGetNumberOfElements(ptr) != 1)
    throw MatlabException("The Matlab array must be a scalar");
  return *mxGetPr(ptr);
}

double& createMatlabScalar (mxArray*& ptr) {
  ptr = mxCreateDoubleScalar(0);
  return *mxGetPr(ptr);
}

// Function definitions for class MatlabScalar.
// ---------------------------------------------------------------
MatlabScalar::MatlabScalar (const mxArray* ptr) 
  : x(getMatlabScalar(ptr)) { }

MatlabScalar::MatlabScalar (mxArray*& ptr, double value) 
  : x(createMatlabScalar(ptr)) {
  x = value;
}

MatlabScalar::MatlabScalar (MatlabScalar& source) 
  : x(source.x) { }

MatlabScalar& MatlabScalar::operator= (double value) {
  x = value;
  return *this;
}
