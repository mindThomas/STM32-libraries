//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xaxpy.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 23:01:08
//
#ifndef XAXPY_5x5_H
#define XAXPY_5x5_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "svd_types.h"

// Function Declarations
extern void b_xaxpy_5x5(int n, float a, const float x[25], int ix0, float y[5], int
                    iy0);
extern void c_xaxpy_5x5(int n, float a, const float x[5], int ix0, float y[25], int
                    iy0);
extern void xaxpy_5x5(int n, float a, int ix0, float y[25], int iy0);

#endif

//
// File trailer for xaxpy.h
//
// [EOF]
//
