//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fitExponentialDecay2_emxutil.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 08-May-2020 23:23:34
//
#ifndef FITEXPONENTIALDECAY2_EMXUTIL_H
#define FITEXPONENTIALDECAY2_EMXUTIL_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "fitExponentialDecay2_types.h"

// Function Declarations
extern void emxCopyStruct_cell_wrap_1(cell_wrap_1 *dst, const cell_wrap_1 *src);
extern void emxEnsureCapacity_real32_T(emxArray_real32_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_real32_T1(emxArray_real32_T *emxArray, int
  oldNumel);
extern void emxFreeMatrix_cell_wrap_1(cell_wrap_1 pMatrix[2]);
extern void emxFree_real32_T(emxArray_real32_T **pEmxArray);
extern void emxInitMatrix_cell_wrap_1(cell_wrap_1 pMatrix[2]);
extern void emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions);
extern void emxInit_real32_T1(emxArray_real32_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for fitExponentialDecay2_emxutil.h
//
// [EOF]
//
