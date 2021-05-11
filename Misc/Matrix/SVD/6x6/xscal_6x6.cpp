//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xscal.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 23:01:31
//

// Include Files
#include "xscal_6x6.h"
#include <MATLABCoderInit/rt_nonfinite.h>
#include "svd_6x6.h"

// Function Definitions

//
// Arguments    : float a
//                float x[36]
//                int ix0
// Return Type  : void
//
void xscal_6x6(float a, float x[36], int ix0)
{
    int k;
    for (k = ix0; k <= ix0 + 5; k++) {
        x[k - 1] *= a;
    }
}

//
// File trailer for xscal.cpp
//
// [EOF]
//
