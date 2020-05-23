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
#include "matlabmatrix.h"
#include "matlabexception.h"

// Function definitions.
// -----------------------------------------------------------------
double* getMatlabMatrixDouble (const mxArray* ptr) {
  if (mxGetNumberOfDimensions(ptr) != 2)
    throw MatlabException("Matlab array must be a matrix");
  if (!mxIsDouble(ptr))
    throw MatlabException("Matlab array must be of type double");
  return mxGetPr(ptr);
}

double* createMatlabMatrixDouble (mxArray*& ptr, int height, int width) {
  ptr = mxCreateDoubleMatrix(height,width,mxREAL);
  return mxGetPr(ptr);
}
  
// Function definitions for class Matrix.
// -----------------------------------------------------------------
Matrix::Matrix (int height, int width)
  : Array<double>(height*width) { 
  h = height;
  w = width;
}

Matrix::Matrix (double* data, int height, int width) 
  : Array<double>(data,height*width) {
  h = height;
  w = width;
}

Matrix::Matrix (const mxArray* ptr) 
  : Array<double>(getMatlabMatrixDouble(ptr),mxGetNumberOfElements(ptr)) {
  h = mxGetM(ptr);
  w = mxGetN(ptr);
}

Matrix::Matrix (mxArray*& ptr, int height, int width) 
  : Array<double>(createMatlabMatrixDouble(ptr,height,width),
		  height*width) {
  h = height;
  w = width;
}

Matrix::Matrix (const Matrix& source)
  : Array<double>(source) {
  h = source.h;
  w = source.w;
}

Matrix& Matrix::operator= (const Matrix& source) {
  inject(source);
  return *this;
}

bool Matrix::operator== (const Matrix& X) const {
  return (h == X.h) && (w == X.w);
}

double& Matrix::entry (int r, int c) {
  return elems[h*c + r];
}

double Matrix::entry (int r, int c) const {
  return elems[h*c + r];
}
