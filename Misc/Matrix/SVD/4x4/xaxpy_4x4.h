//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xaxpy.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 23:00:32
//
#ifndef XAXPY_4x4_H
#define XAXPY_4x4_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "svd_types.h"

// Function Declarations
extern void b_xaxpy_4x4(int n, float a, const float x[16], int ix0, float y[4], int
                    iy0);
extern void c_xaxpy_4x4(int n, float a, const float x[4], int ix0, float y[16], int
                    iy0);
extern void xaxpy_4x4(int n, float a, int ix0, float y[16], int iy0);

#endif

//
// File trailer for xaxpy.h
//
// [EOF]
//
