//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xaxpy.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 12-Feb-2019 21:10:49
//
#ifndef XAXPY_3x3_H
#define XAXPY_3x3_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include <MATLABCoderInit/rtwtypes.h>
#include <Matrix/SVD/svd_types.h>

// Function Declarations
extern void b_xaxpy_3x3(int n, float a, const float x[9], int ix0, float y[3], int
                    iy0);
extern void c_xaxpy_3x3(int n, float a, const float x[3], int ix0, float y[9], int
                    iy0);
extern void xaxpy_3x3(int n, float a, int ix0, float y[9], int iy0);

#endif

//
// File trailer for xaxpy.h
//
// [EOF]
//
