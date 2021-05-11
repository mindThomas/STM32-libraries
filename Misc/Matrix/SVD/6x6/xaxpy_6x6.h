//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xaxpy.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 23:01:31
//
#ifndef XAXPY_6x6_H
#define XAXPY_6x6_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include <MATLABCoderInit/rtwtypes.h>
#include <Matrix/SVD/svd_types.h>

// Function Declarations
extern void b_xaxpy_6x6(int n, float a, const float x[36], int ix0, float y[6], int
                    iy0);
extern void c_xaxpy_6x6(int n, float a, const float x[6], int ix0, float y[36], int
                    iy0);
extern void xaxpy_6x6(int n, float a, int ix0, float y[36], int iy0);

#endif

//
// File trailer for xaxpy.h
//
// [EOF]
//
