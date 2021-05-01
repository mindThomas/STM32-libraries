//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xaxpy.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 23:00:06
//

// Include Files
#include "xaxpy_2x2.h"
#include "rt_nonfinite.h"
#include "svd_2x2.h"

// Function Definitions

//
// Arguments    : float a
//                float y[4]
// Return Type  : void
//
void xaxpy_2x2(float a, float y[4])
{
    int ix;
    int iy;
    int k;
    if (!(a == 0.0F)) {
        ix = 0;
        iy = 2;
        for (k = 0; k < 2; k++) {
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
