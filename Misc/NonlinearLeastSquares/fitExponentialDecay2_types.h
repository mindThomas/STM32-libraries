//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fitExponentialDecay2_types.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 08-May-2020 23:23:34
//
#ifndef FITEXPONENTIALDECAY2_TYPES_H
#define FITEXPONENTIALDECAY2_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
#ifndef struct_emxArray_real32_T
#define struct_emxArray_real32_T

struct emxArray_real32_T
{
  float *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real32_T

typedef struct {
  emxArray_real32_T *f1;
} cell_wrap_1;

#endif

//
// File trailer for fitExponentialDecay2_types.h
//
// [EOF]
//
