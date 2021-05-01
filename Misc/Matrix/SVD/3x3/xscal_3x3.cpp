//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xscal.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 12-Feb-2019 21:10:49
//

// Include Files
#include "xscal_3x3.h"
#include "rt_nonfinite.h"
#include "svd_3x3.h"

// Function Definitions

//
// Arguments    : float a
//                float x[9]
//                int ix0
// Return Type  : void
//
void xscal_3x3(float a, float x[9], int ix0)
{
    int k;
    for (k = ix0; k <= ix0 + 2; k++) {
        x[k - 1] *= a;
    }
}

//
// File trailer for xscal.cpp
//
// [EOF]
//
