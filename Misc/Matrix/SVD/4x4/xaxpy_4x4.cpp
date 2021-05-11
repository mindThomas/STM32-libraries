//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xaxpy.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 23:00:32
//

// Include Files
#include "xaxpy_4x4.h"
#include <MATLABCoderInit/rt_nonfinite.h>
#include "svd_4x4.h"

// Function Definitions

//
// Arguments    : int n
//                float a
//                const float x[16]
//                int ix0
//                float y[4]
//                int iy0
// Return Type  : void
//
void b_xaxpy_4x4(int n, float a, const float x[16], int ix0, float y[4], int iy0)
{
    int ix;
    int iy;
    int k;
    if ((n < 1) || (a == 0.0F)) {
    } else {
        ix = ix0 - 1;
        iy = iy0 - 1;
        for (k = 0; k < n; k++) {
            y[iy] += a * x[ix];
            ix++;
            iy++;
        }
    }
}

//
// Arguments    : int n
//                float a
//                const float x[4]
//                int ix0
//                float y[16]
//                int iy0
// Return Type  : void
//
void c_xaxpy_4x4(int n, float a, const float x[4], int ix0, float y[16], int iy0)
{
    int ix;
    int iy;
    int k;
    if ((n < 1) || (a == 0.0F)) {
    } else {
        ix = ix0 - 1;
        iy = iy0 - 1;
        for (k = 0; k < n; k++) {
            y[iy] += a * x[ix];
            ix++;
            iy++;
        }
    }
}

//
// Arguments    : int n
//                float a
//                int ix0
//                float y[16]
//                int iy0
// Return Type  : void
//
void xaxpy_4x4(int n, float a, int ix0, float y[16], int iy0)
{
    int ix;
    int iy;
    int k;
    if ((n < 1) || (a == 0.0F)) {
    } else {
        ix = ix0 - 1;
        iy = iy0 - 1;
        for (k = 0; k < n; k++) {
            y[iy] += a * y[ix];
            ix++;
            iy++;
        }
    }
}

//
// File trailer for xaxpy.cpp
//
// [EOF]
//
