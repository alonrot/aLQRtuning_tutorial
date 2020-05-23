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
#ifndef INCLUDE_MATLABMATRIX
#define INCLUDE_MATLABMATRIX

#include "array.h"
#include "mex.h"

// Class Matrix
// ---------------------------------------------------------------
// A matrix object stores its elements in column-major format, as in
// Fortran and Matlab. This means that columns are stored one after
// another. For example, the matrix
//
//   1  2  3
//   4  5  6
//
// is stored in memory as
//
//   1  4  2  5  3  6
//
// Like Array objects, Matrix objects are not necessarily
// encapsulated, and they exhibit analogous behaviour.
class Matrix : public Array<double> {
public:
  
  // This constructor allocates memory for matrix of the specified
  // heigth and width.
  Matrix (int height, int width);
  
  // This constructor basically follows the lead of the analagous
  // constructor for the Array class.
  Matrix (double* data, int height, int width);

  // This constructor retrieves a matrix from a Matlab array. This
  // particular constructor is only defined for Matrix<double>.
  // Since Matlab handles storage, the object created by this
  // constructor is not encapsulated.
  explicit Matrix (const mxArray* ptr);

  // This constructor creates a new Matlab array as a side effect.
  // Since Matlab handles storage, the object created by this
  // constructor is not encapsulated.
  Matrix (mxArray*& ptr, int height, int width);

  // The copy constructor makes a shallow copy of the data.
  Matrix (const Matrix& source);
    
  // The destructor.
  ~Matrix() { };
    
  // Copy assignment operator that observes the same behaviour as
  // the Array copy assignment operator.
  Matrix& operator= (const Matrix& source);

  // Get the height and width of the matrix.
  int height() const { return h; };
  int width () const { return w; };

  // Returns true if the two matrices have the same dimensions
  // (i.e. the same height and width).
  bool operator== (const Matrix& X) const;
  bool operator!= (const Matrix& X) const { return !(*this == X); };

  // If X is an object of type Matrix, X.entry(r,c) accesses the
  // entry of the rth row and cth column.
  double  entry      (int r, int c) const;
  double& entry      (int r, int c);
  double  operator() (int r, int c) const { return entry(r,c); };
  double& operator() (int r, int c)       { return entry(r,c); };
    
protected:
  int h;  // The height of the matrix.
  int w;  // The width of the matrix.
};

#endif
