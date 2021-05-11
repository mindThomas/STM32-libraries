//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdotc.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 23:00:06
//

// Include Files
#include "xdotc_2x2.h"
#include <MATLABCoderInit/rt_nonfinite.h>
#include "svd_2x2.h"

// Function Definitions

//
// Arguments    : const float x[4]
//                const float y[4]
// Return Type  : float
//
float xdotc_2x2(const float x[4], const float y[4])
{
    float d;
    int   ix;
    int   iy;
    int   k;
    d  = 0.0F;
    ix = 0;
    iy = 2;
    for (k = 0; k < 2; k++) {
        d += x[ix] * y[iy];
        ix++;
        iy++;
    }

    return d;
}

//
// File trailer for xdotc.cpp
//
// [EOF]
//
