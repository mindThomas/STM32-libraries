//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrot.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 23:00:32
//

// Include Files
#include "xrot_4x4.h"
#include "rt_nonfinite.h"
#include "svd_4x4.h"

// Function Definitions

//
// Arguments    : float x[16]
//                int ix0
//                int iy0
//                float c
//                float s
// Return Type  : void
//
void xrot_4x4(float x[16], int ix0, int iy0, float c, float s)
{
    int   ix;
    int   iy;
    int   k;
    float temp;
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < 4; k++) {
        temp  = c * x[ix] + s * x[iy];
        x[iy] = c * x[iy] - s * x[ix];
        x[ix] = temp;
        iy++;
        ix++;
    }
}

//
// File trailer for xrot.cpp
//
// [EOF]
//
