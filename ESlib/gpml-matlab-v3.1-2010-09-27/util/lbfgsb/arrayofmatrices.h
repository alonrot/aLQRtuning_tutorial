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
#ifndef INCLUDE_ARRAYOFMATRICES
#define INCLUDE_ARRAYOFMATRICES

#include "array.h"
#include "matlabmatrix.h"
#include "mex.h"

// Class ArrayOfMatrices.
// -----------------------------------------------------------------
class ArrayOfMatrices : public Array<Matrix*> {
public:
    
  // This version of the constructor behaves just like its parent.
  explicit ArrayOfMatrices (int length) 
    : Array<Matrix*>(length) { };
    
  // This constructor creates an array of matrices from a Matlab
  // array. It accepts either a matrix in double precision, or a cell
  // array with entries that are matrices.
  explicit ArrayOfMatrices (const mxArray* ptr);

  // This constructor creates an array of matrices from a collection
  // of Matlab arrays. The Matlab arrays must be matrices.
  ArrayOfMatrices (const mxArray* ptrs[], int numptrs);

  // This constructor creates an array of matrices and the
  // associated Matlab structures. The Matlab structures are
  // matrices. The second input argument acts as a template for the
  // creation of the matrices, but the data from "model" is not
  // actually copied into the new ArrayOfMatrices object. It is up
  // to the user to make sure that the array of mxArray pointers has
  // enough room for the pointers to the Matlab arrays.
  ArrayOfMatrices (mxArray* ptrs[], const ArrayOfMatrices& model);

  // This constructor creates an array of matrices using the second
  // input argument as a model. The input argument "data" contains
  // the element data. Note that the information is NOT copied from
  // the model!
  ArrayOfMatrices (double* data, const ArrayOfMatrices& model);
    
  // The copy constructor makes a shallow copy of the source object.
  ArrayOfMatrices (const ArrayOfMatrices& source);

  // The destructor.
  ~ArrayOfMatrices();
    
  // Copy assignment operator that observes the same behaviour as
  // the Array copy assignment operator.
  ArrayOfMatrices& operator= (const ArrayOfMatrices& source);

  // Return the total number of elements in all the matrices.
  int numelems() const;

protected:
  static int getnummatlabmatrices (const mxArray* ptr);
};

#endif
