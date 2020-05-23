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
#ifndef INCLUDE_MATLABPROGRAM
#define INCLUDE_MATLABPROGRAM

#include "mex.h"
#include "program.h"
#include "matlabscalar.h"
#include "matlabstring.h"
#include "arrayofmatrices.h"

// Class MatlabProgram.
// -----------------------------------------------------------------
// This is an implementation of the abstract class Program.
class MatlabProgram: public Program {
public:

  // On input, "variables" should contain the initial point for the
  // optimization routine. If no iterative callback function is
  // specified, "iterFunc" may be set to 0. Also, if no auxiliary data
  // is needed, it may also be set to 0.
  MatlabProgram (ArrayOfMatrices& variables, 
		 const ArrayOfMatrices& lowerbounds, 
		 const ArrayOfMatrices& upperbounds, 
		 const MatlabString* objFunc, const MatlabString* gradFunc, 
		 const MatlabString* iterFunc, mxArray* auxData, 
		 int m = defaultm, int maxiter = defaultmaxiter,
		 double factr = defaultfactr, double pgtol = defaultpgtol);

  // The destructor.
  virtual ~MatlabProgram();

  // These provide definitions for the pure virtual functions of the
  // abstract parent class.
  virtual double computeObjective (int n, double* x);
  virtual void   computeGradient  (int n, double* x, double* g);  
  virtual void   iterCallback     (int t, double* x, double f);

  // Run the solver. Upon completion, the solution is stored in
  // "variables".
  SolverExitStatus runSolver();

protected:
  ArrayOfMatrices&    variables;  // Storage for the initial value and 
                                  // solution.
  const MatlabString* objFunc;    // The objective callback function.
  const MatlabString* gradFunc;   // The gradient callback function.
  const MatlabString* iterFunc;   // The iterative callback function.
  ArrayOfMatrices*    varMatlab;  // Inputs to the Matlab callback
  mxArray**           varInputs;  // functions representing the
				  // current values of the variables.
  MatlabScalar*       fMatlab;    // Input to the Matlab callback
				  // functions representing the
				  // current value of the objective.
  MatlabScalar*       tMatlab;    // Input to the Matlab callback
				  // functions representing the
				  // current iteration.

  int       numInputsObjFunc;  // The number of inputs passed to the
			       // objective callback function.
  int       numInputsGradFunc; // The number of inputs passed to the
			       // gradient callback function.
  int       numInputsIterFunc; // The number of inputs passed to the
			       // iterative callback function.
  mxArray** inputsObjFunc;     // The inputs passed to the objective 
                               // callback function.
  mxArray** inputsGradFunc;    // The inputs passed to the gradient
			       // callback function.
  mxArray** inputsIterFunc;    // The inputs passed to the iterative
			       // callback function.
};

#endif
