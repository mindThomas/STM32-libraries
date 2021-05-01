//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fitExponentialDecay2_emxAPI.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 08-May-2020 23:23:34
//

// Include Files
#include <stdlib.h>
#include "rt_nonfinite.h"
#include "fitExponentialDecay2.h"
#include "fitExponentialDecay2_emxAPI.h"
#include "fitExponentialDecay2_emxutil.h"

// Function Definitions

//
// Arguments    : int numDimensions
//                int *size
// Return Type  : emxArray_real32_T *
//
emxArray_real32_T *emxCreateND_real32_T(int numDimensions, int *size)
{
  emxArray_real32_T *emx;
  int numEl;
  int i;
  emxInit_real32_T(&emx, numDimensions);
  numEl = 1;
  for (i = 0; i < numDimensions; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = (float *)calloc((unsigned int)numEl, sizeof(float));
  emx->numDimensions = numDimensions;
  emx->allocatedSize = numEl;
  return emx;
}

//
// Arguments    : float *data
//                int numDimensions
//                int *size
// Return Type  : emxArray_real32_T *
//
emxArray_real32_T *emxCreateWrapperND_real32_T(float *data, int numDimensions,
  int *size)
{
  emxArray_real32_T *emx;
  int numEl;
  int i;
  emxInit_real32_T(&emx, numDimensions);
  numEl = 1;
  for (i = 0; i < numDimensions; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = data;
  emx->numDimensions = numDimensions;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

//
// Arguments    : float *data
//                int rows
//                int cols
// Return Type  : emxArray_real32_T *
//
emxArray_real32_T *emxCreateWrapper_real32_T(float *data, int rows, int cols)
{
  emxArray_real32_T *emx;
  int size[2];
  int numEl;
  int i;
  size[0] = rows;
  size[1] = cols;
  emxInit_real32_T(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = data;
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

//
// Arguments    : int rows
//                int cols
// Return Type  : emxArray_real32_T *
//
emxArray_real32_T *emxCreate_real32_T(int rows, int cols)
{
  emxArray_real32_T *emx;
  int size[2];
  int numEl;
  int i;
  size[0] = rows;
  size[1] = cols;
  emxInit_real32_T(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = (float *)calloc((unsigned int)numEl, sizeof(float));
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  return emx;
}

//
// Arguments    : emxArray_real32_T *emxArray
// Return Type  : void
//
void emxDestroyArray_real32_T(emxArray_real32_T *emxArray)
{
  emxFree_real32_T(&emxArray);
}

//
// Arguments    : emxArray_real32_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
void emxInitArray_real32_T(emxArray_real32_T **pEmxArray, int numDimensions)
{
  emxInit_real32_T(pEmxArray, numDimensions);
}

//
// File trailer for fitExponentialDecay2_emxAPI.cpp
//
// [EOF]
//
