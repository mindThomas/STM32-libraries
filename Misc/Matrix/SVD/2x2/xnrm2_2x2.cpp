//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xnrm2.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 23:00:06
//

// Include Files
#include "xnrm2_2x2.h"
#include <MATLABCoderInit/mw_cmsis.h>
#include <MATLABCoderInit/rt_nonfinite.h>
#include "svd_2x2.h"
#include <math.h>

// Function Definitions

//
// Arguments    : const float x[4]
// Return Type  : float
//
float xnrm2_2x2(const float x[4])
{
    float y;
    float scale;
    int   k;
    float f0;
    float absxk;
    float t;
    y     = 0.0F;
    scale = 1.29246971E-26F;
    for (k = 0; k < 2; k++) {
        absxk = (float)fabs((double)x[k]);
        if (absxk > scale) {
            t     = scale / absxk;
            y     = 1.0F + y * t * t;
            scale = absxk;
        } else {
            t = absxk / scale;
            y += t * t;
        }
    }

    mw_arm_sqrt_f32(y, &f0);
    return scale * f0;
}

//
// File trailer for xnrm2.cpp
//
// [EOF]
//
