//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: det.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 22:03:21
//

// Include Files
#include <math.h>
//#include "rt_nonfinite.h"
#include "det2x2.h"

// Function Definitions

//
// Arguments    : const float x[4]
// Return Type  : float
//
float det2x2_robust(const float x[4])
{
    float       y;
    int         ix;
    float       b_x[4];
    signed char ipiv[2];
    int         iy;
    boolean_T   isodd;
    int         k;
    float       temp;
    for (ix = 0; ix < 4; ix++) {
        b_x[ix] = x[ix];
    }

    for (ix = 0; ix < 2; ix++) {
        ipiv[ix] = (signed char)(1 + ix);
    }

    ix = 0;
    if ((float)fabs((double)x[1]) > (float)fabs((double)x[0])) {
        ix = 1;
    }

    if (x[ix] != 0.0F) {
        if (ix != 0) {
            ipiv[0] = 2;
            ix      = 0;
            iy      = 1;
            for (k = 0; k < 2; k++) {
                temp    = b_x[ix];
                b_x[ix] = b_x[iy];
                b_x[iy] = temp;
                ix += 2;
                iy += 2;
            }
        }

        b_x[1] /= b_x[0];
    }

    if (b_x[2] != 0.0F) {
        b_x[3] += b_x[1] * -b_x[2];
    }

    y     = b_x[0] * b_x[3];
    isodd = false;
    if (ipiv[0] > 1) {
        isodd = true;
    }

    if (isodd) {
        y = -y;
    }

    return y;
}

float det2x2_fast(const float x[4])
{
    const float& a = x[0];
    const float& b = x[1];
    const float& c = x[2];
    const float& d = x[3];
    return a * d - b * c;
}

//
// File trailer for det.cpp
//
// [EOF]
//
