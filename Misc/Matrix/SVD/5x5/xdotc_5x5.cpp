//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdotc.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 23:01:08
//

// Include Files
#include "xdotc_5x5.h"
#include <MATLABCoderInit/rt_nonfinite.h>
#include "svd_5x5.h"

// Function Definitions

//
// Arguments    : int n
//                const float x[25]
//                int ix0
//                const float y[25]
//                int iy0
// Return Type  : float
//
float xdotc_5x5(int n, const float x[25], int ix0, const float y[25], int iy0)
{
    float d;
    int   ix;
    int   iy;
    int   k;
    d = 0.0F;
    if (!(n < 1)) {
        ix = ix0;
        iy = iy0;
        for (k = 1; k <= n; k++) {
            d += x[ix - 1] * y[iy - 1];
            ix++;
            iy++;
        }
    }

    return d;
}

//
// File trailer for xdotc.cpp
//
// [EOF]
//
