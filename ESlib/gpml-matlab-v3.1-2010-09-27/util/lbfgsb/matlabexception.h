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
#ifndef INCLUDE_MATLABEXCEPTION
#define INCLUDE_MATLABEXCEPTION

#include <exception>

// Class MatlabException
// -----------------------------------------------------------------
// This class just makes it easier for me to throw exceptions. Its
// functionality really has nothing to do with MATLAB.
class MatlabException : public std::exception {
public:
  MatlabException (const char* message) throw();
  ~MatlabException()                    throw() { };
  
  // The copy constructor makes a shallow copy.
  MatlabException (const MatlabException& source) throw();
  
  // The copy assignment operator makes a shallow copy as well.
  MatlabException& operator= (const MatlabException& source);
  
  // Return the message string.
  virtual const char* what () const throw() { return message; };
  
private:
  const char* message;  // The error message.
};

#endif
