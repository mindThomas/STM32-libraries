//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fitExponentialDecay2_emxAPI.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 08-May-2020 23:23:34
//
#ifndef FITEXPONENTIALDECAY2_EMXAPI_H
#define FITEXPONENTIALDECAY2_EMXAPI_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include <MATLABCoderInit/rtwtypes.h>
#include "fitExponentialDecay2_types.h"

// Function Declarations
extern emxArray_real32_T *emxCreateND_real32_T(int numDimensions, int *size);
extern emxArray_real32_T *emxCreateWrapperND_real32_T(float *data, int
  numDimensions, int *size);
extern emxArray_real32_T *emxCreateWrapper_real32_T(float *data, int rows, int
  cols);
extern emxArray_real32_T *emxCreate_real32_T(int rows, int cols);
extern void emxDestroyArray_real32_T(emxArray_real32_T *emxArray);
extern void emxInitArray_real32_T(emxArray_real32_T **pEmxArray, int
  numDimensions);

#endif

//
// File trailer for fitExponentialDecay2_emxAPI.h
//
// [EOF]
//
